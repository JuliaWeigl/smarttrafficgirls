import math
from collections import deque

import rclpy
from rclpy.node import Node
from smart_traffic_interfaces.msg import TrafficFrame
from std_msgs.msg import String


class EventDetector(Node):

    def __init__(self):
        super().__init__('event_detector')

        self.subscription = self.create_subscription(
            TrafficFrame,
            'traffic_frame',
            self.callback,
            10
        )

        self.id_pub = self.create_publisher(
            String,
            'near_collision_ids',
            10
        )

        self.distance_threshold = 0.3
        self.ttc_threshold = 1.5
        self.min_closing_speed = 0.1

        self.active_pairs = set()

        self.event_times = deque()
        self.rate_window_seconds = 60.0
        self.rate_timer = self.create_timer(5.0, self.report_rate)

    def callback(self, msg):
        current_pairs = set()
        near_collision_ids = set()
        vehicles = msg.vehicles

        for i in range(len(vehicles)):
            for j in range(i + 1, len(vehicles)):
                a = vehicles[i]
                b = vehicles[j]

                if a.category == 2 and b.category == 2:
                    continue

                if (a.category == 2 and b.category == 3) or (a.category == 3 and b.category == 2):
                    continue
                if (a.category == 6 and b.category == 4) or (a.category == 4 and b.category == 6):
                    continue
                if (a.category == 7 and b.category == 4) or (a.category == 4 and b.category == 7):
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

                if box_distance < self.distance_threshold and ttc < self.ttc_threshold:
                    pair = tuple(sorted([str(a.track_id), str(b.track_id)]))
                    current_pairs.add(pair)

                    near_collision_ids.add(str(a.track_id))
                    near_collision_ids.add(str(b.track_id))

                    if pair not in self.active_pairs:
                        self.report_event(
                            a,
                            b,
                            box_distance,
                            ttc,
                            closing_speed,
                            msg.timestamp
                        )

        self.active_pairs = current_pairs
        self.publish_near_collision_ids(near_collision_ids)

    def publish_near_collision_ids(self, ids):
        msg = String()
        msg.data = ",".join(sorted(ids))
        self.id_pub.publish(msg)

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

    def report_event(self, a, b, box_distance, ttc, closing_speed, timestamp):
        self.event_times.append(timestamp)

        self.get_logger().warn(
            f'⚠️ Near Collision detected! '
            f'Time: {timestamp:.2f} | '
            f'Actors: {a.track_id} & {b.track_id} | '
            f'Box distance: {box_distance:.2f} m | '
            f'TTC: {ttc:.2f} s | '
            f'Closing speed: {closing_speed:.2f} m/s'
        )

    def report_rate(self):
        if not self.event_times:
            return

        latest_time = self.event_times[-1]
        cutoff = latest_time - self.rate_window_seconds

        while self.event_times and self.event_times[0] < cutoff:
            self.event_times.popleft()

        rate = len(self.event_times) / (self.rate_window_seconds / 60.0)

        self.get_logger().info(
            f'Near collision rate over last {self.rate_window_seconds:.0f}s: '
            f'{rate:.1f} events/min'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EventDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down detector...')
    finally:
        empty_msg = String()
        empty_msg.data = ""
        node.id_pub.publish(empty_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()