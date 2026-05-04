import rclpy
from rclpy.node import Node
from smart_traffic_interfaces.msg import VehicleStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import pandas as pd
import numpy as np
import os

def calculate_dynamic_threshold(csv_path):
    cols = ['track_id', 'timestamp', 'rotation_z']
    df = pd.read_csv(csv_path, usecols=cols)
    
    # grouped by track_id
    df = df.sort_values(['track_id', 'timestamp'])
    df['prev_yaw'] = df.groupby('track_id')['rotation_z'].shift(1)
    df = df.dropna(subset=['prev_yaw'])
    delta_angles = np.arctan2(np.sin(df['rotation_z'] - df['prev_yaw']), 
                              np.cos(df['rotation_z'] - df['prev_yaw']))
    
    yaw_rates = (np.abs(delta_angles) / 0.08)
    
    # Calculate the 97.5th percentile
    threshold = np.percentile(yaw_rates, 97.5)
    return threshold

class YawRateMonitorNode(Node):
    def __init__(self):
        super().__init__('yaw_rate_monitor')
        
        package_path = get_package_share_directory('smart_traffic')
        csv_path = os.path.join(package_path, 'data', 'tumdot_muc_part_1.csv')

        # Dynamic threshold calculation (performed once at startup then print it out)
        self.get_logger().info(f'Processing {csv_path} for dynamic threshold...')
        self.YAW_RATE_THRESHOLD = calculate_dynamic_threshold(csv_path)
        self.get_logger().info(f'Dynamic Threshold set to: {self.YAW_RATE_THRESHOLD:.2f} rad/s')

        self.subscription = self.create_subscription(
            VehicleStatusArray,
            'vehicle_status_array',
            self.listener_callback,
            10)
        
        # Rviz Marker
        self.marker_pub = self.create_publisher(MarkerArray, 'yaw_alerts', 10)
            
        self.last_angles = {}
        self.anomaly_counts = {} 
        self.last_log_time = {}

    def listener_callback(self, msg):
        alert_markers = MarkerArray()
        current_ros_time = self.get_clock().now().nanoseconds / 1e9
        
        for vehicle in msg.vehicles:
            vehicle_id = vehicle.track_id
            
            # Convert quaternions
            quaternion = [
                vehicle.orientation.x, vehicle.orientation.y,
                vehicle.orientation.z, vehicle.orientation.w
            ]
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
            current_angle = yaw
            
            # Rate is calculated only if this is not the first frame of the vehicle
            if vehicle_id in self.last_angles:
                last_angle = self.last_angles[vehicle_id]
                delta_angle = np.arctan2(np.sin(current_angle - last_angle), 
                                        np.cos(current_angle - last_angle))
                dt = 0.08 
                yaw_rate = abs(delta_angle/dt)
                
                #  If the calculated threshold is too extreme (e.g., > 2.5), we force it to 2.5
                effective_threshold = min(self.YAW_RATE_THRESHOLD, 2.5)

                if effective_threshold < yaw_rate < 20.0:
                    self.anomaly_counts[vehicle_id] = self.anomaly_counts.get(vehicle_id, 0) + 1
                    
                    if self.anomaly_counts[vehicle_id] >= 3:
                        last_time = self.last_log_time.get(vehicle_id, 0)
                        if (current_ros_time - last_time) > 3.0:
                            self.get_logger().error(
                                f"SKIDDING: Vehicle {vehicle_id} | Rate: {yaw_rate:.2f} rad/s"
                            )
                            self.last_log_time[vehicle_id] = current_ros_time
                        
                        alert_markers.markers.append(self.create_alert_marker(vehicle, msg.header))
                else:
                    self.anomaly_counts[vehicle_id] = 0

            # Angle updated for comparison in the next frame
            self.last_angles[vehicle_id] = current_angle
            
        if len(alert_markers.markers) > 0:
            self.marker_pub.publish(alert_markers)

    def create_alert_marker(self, vehicle, header):
        marker = Marker()
        marker.header = header
        marker.ns = "yaw_alerts"
        marker.id = int(float(vehicle.track_id))
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = vehicle.position
        
        # transparent red ball
        marker.color.r = 0.8
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.7 # transparency
        marker.scale.x = 2.5
        marker.scale.y = 2.5
        marker.scale.z = 2.5
        
        # Lifecycle: Slightly longer than the frame interval to ensure smooth display
        marker.lifetime = rclpy.duration.Duration(seconds=0.15).to_msg()
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = YawRateMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()