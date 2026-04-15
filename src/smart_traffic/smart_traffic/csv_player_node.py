import rclpy
from rclpy.node import Node
import pandas as pd
import math
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from smart_traffic_interfaces.msg import VehicleStatus
import atexit


class DatasetPublisher(Node):

    def __init__(self):
        super().__init__('dataset_publisher')

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'traffic_objects', 10)
        self.status_pub = self.create_publisher(VehicleStatus, 'vehicle_status', 10)

        # Category + colors
        self.color_map = {
            1: (0.0, 0.0, 1.0),  # Car
            2: (0.0, 1.0, 0.0),  # Pedestrian
            3: (1.0, 1.0, 0.0),  # Bicycle
            4: (1.0, 0.5, 0.0),
            5: (1.0, 0.0, 1.0),
            6: (1.0, 0.0, 0.0),
            7: (0.5, 0.5, 0.5),
        }

        # Load CSV
        self.get_logger().info('Loading CSV...')
        csv_path = '/YOUR/PATH/TO/CSV.csv'  # 🔥 CHANGE THIS

        cols = [
            'timestamp', 'category', 'track_id',
            'translation_x', 'translation_y', 'translation_z',
            'dimension_x', 'dimension_y', 'dimension_z',
            'rotation_x', 'rotation_y', 'rotation_z',
            'velocity_x', 'velocity_y',
            'acceleration_x', 'acceleration_y',
        ]

        df = pd.read_csv(csv_path, usecols=cols)

        # Group by timestamp
        self.grouped_data = dict(list(df.groupby('timestamp')))
        self.timestamps = sorted(self.grouped_data.keys())
        self.current_step = 0

        # Timer (12.5 Hz)
        self.timer = self.create_timer(0.08, self.timer_callback)

        atexit.register(self.cleanup_markers)

        self.get_logger().info('Dataset playback started')

    # ================================
    # MAIN LOOP
    # ================================
    def timer_callback(self):

        if self.current_step >= len(self.timestamps):
            self.get_logger().info('Restarting dataset...')
            self.current_step = 0
            return

        ts = self.timestamps[self.current_step]
        current_frame = self.grouped_data[ts]

        marker_array = MarkerArray()

        for _, row in current_frame.iterrows():

            cube = self.create_cube_marker(row)
            marker_array.markers.append(cube)

            arrow = self.create_velocity_arrow(row, cube)
            marker_array.markers.append(arrow)

            status_msg = self.create_vehicle_status(row, cube)
            self.status_pub.publish(status_msg)

        self.marker_pub.publish(marker_array)
        self.current_step += 1

    # ================================
    # CREATE MARKERS
    # ================================
    def create_cube_marker(self, row):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"
        marker.id = int(row['track_id'])
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(row['translation_x'])
        marker.pose.position.y = float(row['translation_y'])
        marker.pose.position.z = float(row['translation_z'])

        # Rotation → quaternion
        roll = float(row['rotation_x'])
        pitch = float(row['rotation_y'])
        yaw = float(row['rotation_z'])
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # Size
        marker.scale.x = float(row['dimension_x'])
        marker.scale.y = float(row['dimension_y'])
        marker.scale.z = float(row['dimension_z'])

        # Color
        cat_id = int(row['category'])
        r, g, b = self.color_map.get(cat_id, (1.0, 1.0, 1.0))

        # Acceleration-based coloring
        ax = float(row['acceleration_x'])
        ay = float(row['acceleration_y'])
        a_total = math.sqrt(ax**2 + ay**2)

        if a_total > 2.0:
            r, g, b = (1.0, 0.0, 0.0)  # strong acceleration
        elif ax < -2.0:
            r, g, b = (1.0, 0.0, 1.0)  # braking

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8

        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        return marker

    def create_velocity_arrow(self, row, parent_marker):

        arrow = Marker()
        arrow.header = parent_marker.header
        arrow.ns = "velocity_vectors"
        arrow.id = int(row['track_id']) + 1000000
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        start = Point(
            x=parent_marker.pose.position.x,
            y=parent_marker.pose.position.y,
            z=parent_marker.pose.position.z
        )

        end = Point(
            x=start.x + float(row['velocity_x']) * 0.5,
            y=start.y + float(row['velocity_y']) * 0.5,
            z=start.z
        )

        arrow.points = [start, end]
        arrow.scale.x = 0.2
        arrow.scale.y = 0.4
        arrow.scale.z = 0.5

        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        arrow.lifetime = parent_marker.lifetime

        return arrow

    # ================================
    # VEHICLE STATUS (IMPORTANT)
    # ================================
    def create_vehicle_status(self, row, marker):

        msg = VehicleStatus()

        msg.track_id = str(row['track_id'])
        msg.category = int(row['category'])

        msg.position = marker.pose.position
        msg.orientation = marker.pose.orientation

        msg.dimension_x = float(row['dimension_x'])
        msg.dimension_y = float(row['dimension_y'])
        msg.dimension_z = float(row['dimension_z'])

        # 🚨 IMPORTANT FOR COLLISION DETECTION
        msg.velocity_x = float(row['velocity_x'])
        msg.velocity_y = float(row['velocity_y'])

        msg.speed = math.sqrt(
            msg.velocity_x**2 + msg.velocity_y**2
        )

        msg.acceleration_x = float(row['acceleration_x'])
        msg.acceleration_y = float(row['acceleration_y'])

        return msg

    # ================================
    # CLEANUP
    # ================================
    def cleanup_markers(self):
        self.get_logger().info('Cleaning markers...')
        cleanup = MarkerArray()

        for ns in ["objects", "velocity_vectors"]:
            m = Marker()
            m.header.frame_id = "map"
            m.ns = ns
            m.action = Marker.DELETEALL
            cleanup.markers.append(m)

        self.marker_pub.publish(cleanup)


# ================================
# MAIN
# ================================
def main(args=None):
    rclpy.init(args=args)
    node = DatasetPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')

    finally:
        node.cleanup_markers()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()