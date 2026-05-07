import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from smart_traffic_interfaces.msg import TrafficFrame


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid_node')

        self.resolution = 2.0

        self.origin_x = -376.8214
        self.origin_y = -87.7518

        self.grid_width = 360
        self.grid_height = 115

        self.num_channels = 9

        self.OCCUPANCY = 0
        self.VX = 1
        self.VY = 2
        self.AX = 3
        self.AY = 4
        self.SPEED = 5
        self.EVENT = 6
        self.TTC = 7
        self.YAW_EVENT = 8

        self.distance_threshold = 0.3
        self.ttc_threshold = 1.5
        self.min_closing_speed = 0.1

        self.near_collision_ids = set()
        self.yaw_anomaly_ids = set()

        self.create_subscription(
            TrafficFrame,
            'traffic_frame',
            self.traffic_frame_callback,
            10
        )

        self.create_subscription(
            String,
            'near_collision_ids',
            self.near_collision_callback,
            10
        )

        self.create_subscription(
            String,
            'yaw_anomaly_ids',
            self.yaw_anomaly_callback,
            10
        )

        self.grid_pub = self.create_publisher(
            Float32MultiArray,
            'multi_channel_occupancy_grid',
            10
        )

        self.get_logger().info('Occupancy Grid Node started with yaw event channel')

    def near_collision_callback(self, msg):
        if msg.data == "":
            self.near_collision_ids = set()
        else:
            self.near_collision_ids = set(msg.data.split(","))

    def yaw_anomaly_callback(self, msg):
        if msg.data == "":
            self.yaw_anomaly_ids = set()
        else:
            self.yaw_anomaly_ids = set(msg.data.split(","))

    def traffic_frame_callback(self, msg):
        grid = self.build_grid(msg)
        ros_msg = self.grid_to_msg(grid)
        self.grid_pub.publish(ros_msg)

    def build_grid(self, frame_msg):
        grid = np.zeros(
            (self.num_channels, self.grid_height, self.grid_width),
            dtype=np.float32
        )

        counts = np.zeros(
            (self.grid_height, self.grid_width),
            dtype=np.float32
        )

        grid[self.TTC, :, :] = -1.0

        object_cell_map = {}
        vehicles = frame_msg.vehicles

        for vehicle in vehicles:
            cell = self.world_to_grid(
                vehicle.position.x,
                vehicle.position.y
            )

            if cell is None:
                continue

            cell_x, cell_y = cell
            object_cell_map[str(vehicle.track_id)] = (cell_x, cell_y)

            grid[self.OCCUPANCY, cell_y, cell_x] = 1.0

            grid[self.VX, cell_y, cell_x] += vehicle.velocity_x
            grid[self.VY, cell_y, cell_x] += vehicle.velocity_y
            grid[self.AX, cell_y, cell_x] += vehicle.acceleration_x
            grid[self.AY, cell_y, cell_x] += vehicle.acceleration_y
            grid[self.SPEED, cell_y, cell_x] += vehicle.speed

            counts[cell_y, cell_x] += 1.0

        occupied_cells = counts > 0

        grid[self.VX][occupied_cells] /= counts[occupied_cells]
        grid[self.VY][occupied_cells] /= counts[occupied_cells]
        grid[self.AX][occupied_cells] /= counts[occupied_cells]
        grid[self.AY][occupied_cells] /= counts[occupied_cells]
        grid[self.SPEED][occupied_cells] /= counts[occupied_cells]

        for track_id in self.near_collision_ids:
            if track_id in object_cell_map:
                cell_x, cell_y = object_cell_map[track_id]
                grid[self.EVENT, cell_y, cell_x] = 1.0

        for track_id in self.yaw_anomaly_ids:
            if track_id in object_cell_map:
                cell_x, cell_y = object_cell_map[track_id]
                grid[self.YAW_EVENT, cell_y, cell_x] = 1.0

        self.fill_ttc_channel(grid, vehicles, object_cell_map)

        return grid

    def world_to_grid(self, x, y):
        cell_x = int((x - self.origin_x) / self.resolution)
        cell_y = int((y - self.origin_y) / self.resolution)

        if 0 <= cell_x < self.grid_width and 0 <= cell_y < self.grid_height:
            return cell_x, cell_y

        return None

    def grid_to_msg(self, grid):
        msg = Float32MultiArray()

        msg.layout.dim = [
            MultiArrayDimension(
                label='channels',
                size=self.num_channels,
                stride=self.num_channels * self.grid_height * self.grid_width
            ),
            MultiArrayDimension(
                label='height',
                size=self.grid_height,
                stride=self.grid_height * self.grid_width
            ),
            MultiArrayDimension(
                label='width',
                size=self.grid_width,
                stride=self.grid_width
            )
        ]

        msg.data = grid.flatten().tolist()
        return msg

    def fill_ttc_channel(self, grid, vehicles, object_cell_map):
        for i in range(len(vehicles)):
            for j in range(i + 1, len(vehicles)):
                a = vehicles[i]
                b = vehicles[j]

                if self.should_skip_pair(a, b):
                    continue

                box_distance, closest_a, closest_b = self.compute_box_distance(a, b)

                closing_speed = self.compute_closing_speed_between_closest_points(
                    a, b, closest_a, closest_b
                )

                if closing_speed <= self.min_closing_speed:
                    continue

                if box_distance <= 1e-6:
                    ttc = 0.0
                else:
                    ttc = box_distance / closing_speed

                if ttc > self.ttc_threshold:
                    continue

                for track_id in [str(a.track_id), str(b.track_id)]:
                    if track_id not in object_cell_map:
                        continue

                    cell_x, cell_y = object_cell_map[track_id]
                    current_ttc = grid[self.TTC, cell_y, cell_x]

                    if current_ttc < 0:
                        grid[self.TTC, cell_y, cell_x] = ttc
                    else:
                        grid[self.TTC, cell_y, cell_x] = min(current_ttc, ttc)

    def should_skip_pair(self, a, b):
        if a.category == 2 and b.category == 2:
            return True

        if (a.category == 2 and b.category == 3) or (a.category == 3 and b.category == 2):
            return True

        if (a.category == 6 and b.category == 4) or (a.category == 4 and b.category == 6):
            return True

        if (a.category == 7 and b.category == 4) or (a.category == 4 and b.category == 7):
            return True

        return False

    def compute_closing_speed_between_closest_points(self, a, b, closest_a, closest_b):
        gap_x = closest_b[0] - closest_a[0]
        gap_y = closest_b[1] - closest_a[1]

        gap_length = math.sqrt(gap_x**2 + gap_y**2)

        if gap_length < 1e-6:
            return float('inf')

        gap_dir_x = gap_x / gap_length
        gap_dir_y = gap_y / gap_length

        dvx = b.velocity_x - a.velocity_x
        dvy = b.velocity_y - a.velocity_y

        closing_speed = -(dvx * gap_dir_x + dvy * gap_dir_y)

        return closing_speed

    def compute_box_distance(self, a, b):
        rect_a = self.get_rectangle_corners(a)
        rect_b = self.get_rectangle_corners(b)

        if self.polygons_overlap(rect_a, rect_b):
            return 0.0, (a.position.x, a.position.y), (b.position.x, b.position.y)

        min_distance = float('inf')
        closest_a = None
        closest_b = None

        for p in rect_a:
            for k in range(len(rect_b)):
                q1 = rect_b[k]
                q2 = rect_b[(k + 1) % len(rect_b)]

                d, closest_on_b = self.point_to_segment_distance_with_point(
                    p, q1, q2
                )

                if d < min_distance:
                    min_distance = d
                    closest_a = p
                    closest_b = closest_on_b

        for p in rect_b:
            for k in range(len(rect_a)):
                q1 = rect_a[k]
                q2 = rect_a[(k + 1) % len(rect_a)]

                d, closest_on_a = self.point_to_segment_distance_with_point(
                    p, q1, q2
                )

                if d < min_distance:
                    min_distance = d
                    closest_a = closest_on_a
                    closest_b = p

        return min_distance, closest_a, closest_b

    def get_rectangle_corners(self, obj):
        cx = obj.position.x
        cy = obj.position.y

        length = obj.dimension_x
        width = obj.dimension_y

        yaw = self.yaw_from_quaternion(obj.orientation)

        half_l = length / 2.0
        half_w = width / 2.0

        local_corners = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w),
        ]

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_corners = []

        for x, y in local_corners:
            wx = cx + x * cos_yaw - y * sin_yaw
            wy = cy + x * sin_yaw + y * cos_yaw
            world_corners.append((wx, wy))

        return world_corners

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def polygons_overlap(self, poly_a, poly_b):
        for polygon in [poly_a, poly_b]:
            for i in range(len(polygon)):
                p1 = polygon[i]
                p2 = polygon[(i + 1) % len(polygon)]

                edge_x = p2[0] - p1[0]
                edge_y = p2[1] - p1[1]

                axis_x = -edge_y
                axis_y = edge_x

                length = math.sqrt(axis_x**2 + axis_y**2)

                if length < 1e-6:
                    continue

                axis_x /= length
                axis_y /= length

                min_a, max_a = self.project_polygon(poly_a, axis_x, axis_y)
                min_b, max_b = self.project_polygon(poly_b, axis_x, axis_y)

                if max_a < min_b or max_b < min_a:
                    return False

        return True

    def project_polygon(self, polygon, axis_x, axis_y):
        values = []

        for p in polygon:
            projection = p[0] * axis_x + p[1] * axis_y
            values.append(projection)

        return min(values), max(values)

    def point_to_segment_distance_with_point(self, p, a, b):
        px, py = p
        ax, ay = a
        bx, by = b

        abx = bx - ax
        aby = by - ay

        apx = px - ax
        apy = py - ay

        ab_len_sq = abx**2 + aby**2

        if ab_len_sq < 1e-6:
            distance = math.sqrt((px - ax)**2 + (py - ay)**2)
            return distance, (ax, ay)

        t = (apx * abx + apy * aby) / ab_len_sq
        t = max(0.0, min(1.0, t))

        closest_x = ax + t * abx
        closest_y = ay + t * aby

        distance = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

        return distance, (closest_x, closest_y)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down occupancy grid node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()