import rclpy
from rclpy.node import Node
from smart_traffic_interfaces.msg import VehicleStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import pandas as pd
import numpy as np
import os


def calculate_category_thresholds(csv_path):
    """
    Advanced preprocessing: Calculates separate thresholds for each category 
    using IQR filtering to remove outliers before percentile calculation.
    """
    cols = ['track_id', 'timestamp', 'rotation_z', 'category']
    df = pd.read_csv(csv_path, usecols=cols)
    
    # grouped by track_id
    df = df.sort_values(['track_id', 'timestamp'])
    df['prev_yaw'] = df.groupby('track_id')['rotation_z'].shift(1)
    df = df.dropna(subset=['prev_yaw'])
    delta_angles = np.arctan2(np.sin(df['rotation_z'] - df['prev_yaw']), 
                              np.cos(df['rotation_z'] - df['prev_yaw']))
    
    df['yaw_rate'] = np.abs(delta_angles) / 0.08
    
    thresholds_dict = {}
    unique_categories = df['category'].unique()

    for cat in unique_categories:
        if cat in [2, 3]: # Skip Pedestrians and bicycles during calibration
            continue
        cat_data = df[df['category'] == cat]['yaw_rate']
        
        # Initial cleaning: Ignore obvious sensor errors/jumps
        cat_data = cat_data[cat_data < 20.0]
        
        if not cat_data.empty:
            # --- OUTLIER DETECTION (IQR Method) ---
            Q1 = np.percentile(cat_data, 25)
            Q3 = np.percentile(cat_data, 75)
            IQR = Q3 - Q1
            
            # Set the multiplier to 2.2, which in statistics falls between “mild” and “extreme” outliers
            iqr_boundary = Q3 + 2.2 * IQR
            
            # Lowering the percentile could fetch more warnings
            percentile = np.percentile(cat_data, 96)
            final_val = min(iqr_boundary, percentile)

            # Set a floor number in case of too low threshold, since the majority of objects are driving straight
            if cat == 1: # Car
                final_val = max(final_val, 0.7)
            else:
                final_val = max(final_val, 0.5)
            thresholds_dict[int(cat)] = final_val
        else:
            thresholds_dict[int(cat)] = 2.5

    return thresholds_dict


class YawRateMonitorNode(Node):
    def __init__(self):
        super().__init__('yaw_rate_monitor')

        self.category_names = {
            1: "Car",
            2: "Pedestrian",
            3: "Bicycle",
            4: "Trailer",
            5: "Motorcycle",
            6: "Truck",
            7: "Bus",
            8: "Scooter",
            9: "Streetcar"
        }
        
        package_path = get_package_share_directory('smart_traffic')
        csv_path = os.path.join(package_path, 'data', 'tumdot_muc_part_1.csv')

        # Calculate specific thresholds for each category
        self.get_logger().info('Calibrating category-specific thresholds...')
        self.thresholds_dict = calculate_category_thresholds(csv_path)
        for cat_id, threshold in self.thresholds_dict.items():
            name = self.category_names.get(cat_id, f"Unknown({cat_id})")
            self.get_logger().info(f"Set threshold for {name}: {threshold:.2f} rad/s")

        self.subscription = self.create_subscription(
            VehicleStatusArray,
            'vehicle_status_array',
            self.listener_callback,
            10)
        
        self.marker_pub = self.create_publisher(MarkerArray, 'yaw_alerts', 10)
            
        self.last_angles = {}
        self.anomaly_counts = {} 
        self.last_log_time = {}

    def listener_callback(self, msg):
        alert_markers = MarkerArray()
        current_ros_time = self.get_clock().now().nanoseconds / 1e9
        
        for vehicle in msg.vehicles:
            vehicle_id = vehicle.track_id
            cat_id = vehicle.category
            if cat_id in [2, 3]: # Ignore pedetrians and bicycles
                continue
            quaternion = [
                vehicle.orientation.x, vehicle.orientation.y,
                vehicle.orientation.z, vehicle.orientation.w
            ]
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
            current_angle = yaw
            
            if vehicle_id in self.last_angles:
                last_angle = self.last_angles[vehicle_id]
                delta_angle = np.arctan2(np.sin(current_angle - last_angle), 
                                         np.cos(current_angle - last_angle))
                
                yaw_rate = abs(delta_angle / 0.08)
                
                # Retrieve threshold specific to this category
                current_threshold = self.thresholds_dict.get(cat_id, 2.5)

                if current_threshold < yaw_rate < 20.0:
                    self.anomaly_counts[vehicle_id] = self.anomaly_counts.get(vehicle_id, 0) + 1
                    
                    if self.anomaly_counts[vehicle_id] >= 3:
                        last_time = self.last_log_time.get(vehicle_id, 0)
                        
                        # Cooldown mechanism for logging
                        if (current_ros_time - last_time) > 3.0:
                            # Look up the name using the ID
                            cat_name = self.category_names.get(cat_id, f"Unknown({cat_id})")
                            self.get_logger().error(
                                f"🚨 SKIDDING: ID {vehicle_id} ({cat_name}) | Rate: {yaw_rate:.2f} rad/s"
                            )
                            self.last_log_time[vehicle_id] = current_ros_time
                        
                        # RViz Interaction
                        alert_markers.markers.append(self.create_alert_marker(vehicle, msg.header))
                else:
                    self.anomaly_counts[vehicle_id] = 0

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
        marker.scale.x = marker.scale.y = marker.scale.z = 2.5
        
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