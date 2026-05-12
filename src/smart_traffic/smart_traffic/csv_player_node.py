import os
import math
import atexit

import rclpy
import pandas as pd
import tf_transformations

from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from smart_traffic_interfaces.msg import (
    VehicleStatus,
    TrafficFrame
)


class DatasetPublisher(Node):

    def __init__(self):
        super().__init__('dataset_publisher')

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'traffic_objects',
            10
        )

        self.status_pub = self.create_publisher(
            VehicleStatus,
            'vehicle_status',
            10
        )

        self.frame_pub = self.create_publisher(
            TrafficFrame,
            'traffic_frame',
            10
        )

        self.near_collision_ids = set()

        self.create_subscription(
            String,
            'near_collision_ids',
            self.near_collision_callback,
            10
        )

        self.color_map = {
            1: (0.0, 0.0, 1.0), # Car - Blue
            2: (0.0, 1.0, 0.0), # Pedestrian - Green
            3: (1.0, 1.0, 0.0), # Bike - Yellow
            4: (1.0, 0.5, 0.0), # Trailer - Orange
            5: (1.0, 0.0, 1.0), # Motorcycle - Purple
            6: (1.0, 0.0, 0.0), # Truck - Red
            7: (0.5, 0.5, 0.5), # Bus - Gray
        }

        self.get_logger().info('Loading CSV...')

        package_path = get_package_share_directory('smart_traffic')
        csv_path = os.path.join(package_path, 'data', 'tumdot_muc_part_1.csv')

        cols = [
            'timestamp', 'category', 'track_id',
            'translation_x', 'translation_y', 'translation_z',
            'dimension_x', 'dimension_y', 'dimension_z',
            'rotation_x', 'rotation_y', 'rotation_z',
            'velocity_x', 'velocity_y',
            'acceleration_x', 'acceleration_y',
        ]

        df = pd.read_csv(csv_path, usecols=cols)

        self.grouped_data = dict(list(df.groupby('timestamp')))
        self.timestamps = sorted(self.grouped_data.keys())
        self.current_step = 0

        self.timer = self.create_timer(0.08, self.timer_callback)

        atexit.register(self.cleanup_markers)

        self.get_logger().info('Dataset playback started')

    def near_collision_callback(self, msg):
        if msg.data == "":
            self.near_collision_ids = set()
        else:
            self.near_collision_ids = set(msg.data.split(","))

    def timer_callback(self):
        if self.current_step >= len(self.timestamps):
            self.get_logger().info('Restarting dataset...')
            self.current_step = 0
            return

        ts = self.timestamps[self.current_step]
        current_frame = self.grouped_data[ts]

        marker_array = MarkerArray()

        frame_msg = TrafficFrame()
        frame_msg.timestamp = float(ts)
        frame_msg.vehicles = []

        for _, row in current_frame.iterrows():
            cube = self.create_cube_marker(row)
            marker_array.markers.append(cube)

            status_msg = self.create_vehicle_status(row, cube)

            self.status_pub.publish(status_msg)
            frame_msg.vehicles.append(status_msg)

        self.marker_pub.publish(marker_array)
        self.frame_pub.publish(frame_msg)

        self.current_step += 1

    def create_cube_marker(self, row):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"
        marker.id = int(row['track_id'])
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(row['translation_x'])
        marker.pose.position.y = float(row['translation_y'])
        marker.pose.position.z = float(row['translation_z'])

        roll = float(row['rotation_x'])
        pitch = float(row['rotation_y'])
        yaw = float(row['rotation_z'])

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.scale.x = float(row['dimension_x'])
        marker.scale.y = float(row['dimension_y'])
        marker.scale.z = float(row['dimension_z'])

        cat_id = int(row['category'])
        r, g, b = self.color_map.get(cat_id, (1.0, 1.0, 1.0))

        if str(row['track_id']) in self.near_collision_ids:
            r, g, b = (0.3, 0.9, 1.0)

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.85

        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        return marker

    def create_vehicle_status(self, row, marker):
        msg = VehicleStatus()

        msg.track_id = str(row['track_id'])
        msg.category = int(row['category'])
        msg.timestamp = float(row['timestamp'])

        msg.position = marker.pose.position
        msg.orientation = marker.pose.orientation

        msg.dimension_x = float(row['dimension_x'])
        msg.dimension_y = float(row['dimension_y'])
        msg.dimension_z = float(row['dimension_z'])

        msg.velocity_x = float(row['velocity_x'])
        msg.velocity_y = float(row['velocity_y'])

        msg.speed = math.sqrt(msg.velocity_x**2 + msg.velocity_y**2)

        msg.acceleration_x = float(row['acceleration_x'])
        msg.acceleration_y = float(row['acceleration_y'])

        return msg

    def cleanup_markers(self):
        if not rclpy.ok():
            return

        self.get_logger().info('Cleaning markers...')

        cleanup = MarkerArray()

        for ns in ["objects", "velocity_vectors"]:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = ns
            marker.action = Marker.DELETEALL
            cleanup.markers.append(marker)

        self.marker_pub.publish(cleanup)


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