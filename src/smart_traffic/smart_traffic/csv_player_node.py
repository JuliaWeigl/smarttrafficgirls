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
        self.publisher_ = self.create_publisher(MarkerArray, 'traffic_objects', 10)
        self.status_pub = self.create_publisher(VehicleStatus, 'vehicle_status', 10)

        # 1. Load metadata (based on your JSON content)
        self.categories = {"1": "car", "2": "pedestrian", "3": "bicycle", "4": "trailer", 
                           "5": "motorcycle", "6": "truck", "7": "bus", "8": "scooter", "9": "streetcar"}
        self.color_map = {
            1: (0.0, 0.0, 1.0), # Car - Blue
            2: (0.0, 1.0, 0.0), # Pedestrian - Green
            3: (1.0, 1.0, 0.0), # Bike - Yellow
            4: (1.0, 0.5, 0.0), # Trailer - Orange
            5: (1.0, 0.0, 1.0), # Motorcycle - Purple
            6: (1.0, 0.0, 0.0), # Truck - Red
            7: (0.5, 0.5, 0.5), # Bus - Gray
            # You can continue to add color mappings for other categories
        }

        # 2. Load the first CSV file and group by timestamp (we recommend testing with a single file first)
        self.get_logger().info('Loading CSV data... This may take a while.')
        csv_path = '/home/watan/ros2_ws/src/smart_traffic/data/tumdot_muc_part_1.csv' # Replace with your path
        # Optimize the reading method to save memory
        cols = ['timestamp', 'category', 'track_id', 
        'translation_x', 'translation_y', 'translation_z', 
        'dimension_x', 'dimension_y', 'dimension_z', 
        'rotation_x', 'rotation_y', 'rotation_z', 
        'velocity_x', 'velocity_y', 'acceleration_x', 'acceleration_y',
        ]
        df = pd.read_csv(csv_path, usecols=cols)
        # # read the whole csv
        # df = pd.read_csv(csv_path)
        self.grouped_data = dict(list(df.groupby('timestamp')))
        self.timestamps = sorted(self.grouped_data.keys())
        self.current_step = 0
        
        # 3. create timer (12.5Hz = 0.08s interval)
        self.timer = self.create_timer(0.08, self.timer_callback)
        atexit.register(self.cleanup_markers)
        self.get_logger().info('Start playing dataset...')


    def timer_callback(self):
        if self.current_step >= len(self.timestamps):
            self.get_logger().info('Rewinding dataset...')
            self.current_step = 0 # rewind when the dataset playing over
            return

        ts = self.timestamps[self.current_step]
        current_frame = self.grouped_data[ts]
        marker_array = MarkerArray()
        
        # Iterate through all objects under this timestamp
        for _, row in current_frame.iterrows():
            # generate CUBEs - Marker
            cube = self._create_cube_marker(row)
            marker_array.markers.append(cube)

            # generate velocity ARROWs - Marker
            arrow = self._create_velocity_arrow(row, cube)
            marker_array.markers.append(arrow)

            # Post a custom status message
            status_msg = self._create_vehicle_status(row, cube)
            self.status_pub.publish(status_msg)

        self.publisher_.publish(marker_array)
        self.current_step += 1

    def _create_cube_marker(self, row):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = int(row['track_id'])
            marker.type = Marker.CUBE  # make objects cube
            marker.action = Marker.ADD

            # Position (from “translation” column)
            marker.pose.position.x = float(row['translation_x'])
            marker.pose.position.y = float(row['translation_y'])
            #marker.pose.position.z = 0.0 # To attach the grid to the cars
            marker.pose.position.z = float(row['translation_z'])

            # Rotation
            roll = float(row['rotation_x']) 
            pitch = float(row['rotation_y']) 
            yaw = float(row['rotation_z']) 
            # The rotation vector needs to be converted to a quaternion here.
            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            # Dimensions
            marker.scale.x = float(row['dimension_x'])
            marker.scale.y = float(row['dimension_y'])
            marker.scale.z = float(row['dimension_z'])

            # Set colors based on category ID
            cat_id = int(row['category'])
            r, g, b = self.color_map.get(cat_id, (1.0, 1.0, 1.0)) # default color: white
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.8 # Transparency
            
            # Acceleration-based color-changing logic
            a_total = math.sqrt(float(row['acceleration_x'])**2 + float(row['acceleration_y'])**2)
            if a_total > 2.0: # Sudden acceleration/lane change results in a red color
                marker.color.r, marker.color.g, marker.color.b = (1.0, 0.0, 0.0)
            elif float(row['acceleration_x']) < -2.0: # turns purple at hard braking
                marker.color.r, marker.color.g, marker.color.b = (1.0, 0.0, 1.0)
            else:
                marker.color.r, marker.color.g, marker.color.b = r, g, b
                
            marker.color.a = 0.8
            marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
            return marker


    def _create_velocity_arrow(self, row, parent_marker):
        arrow = Marker()
        arrow.header = parent_marker.header
        arrow.ns = "velocity_vectors"
        arrow.id = int(row['track_id']) + 2000000
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        start = Point(x=parent_marker.pose.position.x, 
                      y=parent_marker.pose.position.y, 
                      z=parent_marker.pose.position.z)
        end = Point(x=parent_marker.pose.position.x + float(row['velocity_x']) * 0.5,
                    y=parent_marker.pose.position.y + float(row['velocity_y']) * 0.5,
                    z=parent_marker.pose.position.z)

        arrow.points = [start, end]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = (0.2, 0.4, 0.5)
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (1.0, 1.0, 0.0, 1.0)
        arrow.lifetime = parent_marker.lifetime
        return arrow


    def _create_vehicle_status(self, row, marker):
        msg = VehicleStatus()
        msg.track_id = str(row['track_id'])
        msg.category = int(row['category'])
        msg.position = marker.pose.position
        msg.orientation = marker.pose.orientation
        msg.dimension_x = float(row['dimension_x'])
        msg.dimension_y = float(row['dimension_y'])
        msg.dimension_z = float(row['dimension_z'])
        msg.speed = math.sqrt(float(row['velocity_x'])**2 + float(row['velocity_y'])**2)
        msg.acceleration_x = float(row['acceleration_x'])
        msg.acceleration_y = float(row['acceleration_y'])
        return msg

    # TODO: Velocity/Acceleration data need to be incorporated in a better way.

    def cleanup_markers(self):
        self.get_logger().info('Cleaning up markers...')
        cleanup_array = MarkerArray()
        for ns in ["objects", "velocity_vectors", "metadata"]:
            m = Marker()
            m.header.frame_id, m.ns, m.action = "map", ns, Marker.DELETEALL
            cleanup_array.markers.append(m)
        for _ in range(3):
            self.publisher_.publish(cleanup_array)


def main(args=None):
    rclpy.init(args=args)
    node = DatasetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, cleaning up...')
    finally:
        # Manually call cleanup function
        node.cleanup_markers()
        node.destroy_node()
        rclpy.shutdown()
