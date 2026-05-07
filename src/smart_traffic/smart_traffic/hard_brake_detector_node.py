import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smart_traffic_interfaces.msg import TrafficFrame


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HardBrakeDetector(Node):

    def __init__(self):
        super().__init__('hard_brake_detector')

        self.declare_parameter('brake_threshold', -3.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('event_duration', 1.0)

        self.brake_threshold = float(self.get_parameter('brake_threshold').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.event_duration = float(self.get_parameter('event_duration').value)

        self.subscription = self.create_subscription(
            TrafficFrame,
            'traffic_frame',
            self.traffic_frame_callback,
            10
        )

        self.hard_brake_pub = self.create_publisher(
            String,
            'hard_brake_ids',
            10
        )

        self.track_counts = {}
        self.last_log_time = {}
        self.active_events = {}

        self.get_logger().info(
            'Hard brake detector started | '
            f'longitudinal_acc < {self.brake_threshold}, '
            f'confirm_frames={self.confirm_frames}'
        )

    def traffic_frame_callback(self, msg):
        current_ros_time = self.get_clock().now().nanoseconds / 1e9

        for vehicle in msg.vehicles:
            track_id = str(vehicle.track_id)

            ax = vehicle.acceleration_x
            ay = vehicle.acceleration_y

            yaw = yaw_from_quaternion(vehicle.orientation)

            acc_norm = math.sqrt(ax**2 + ay**2)
            longitudinal_acc = ax * math.cos(yaw) + ay * math.sin(yaw)

            is_hard_brake = longitudinal_acc < self.brake_threshold

            if is_hard_brake:
                self.track_counts[track_id] = self.track_counts.get(track_id, 0) + 1

                if self.track_counts[track_id] >= self.confirm_frames:
                    self.active_events[track_id] = current_ros_time

                    last_time = self.last_log_time.get(track_id, 0.0)

                    if current_ros_time - last_time > 3.0:
                        self.get_logger().warn(
                            f'HARD BRAKE DETECTED | '
                            f'track_id={track_id} | '
                            f'category={vehicle.category} | '
                            f'speed={vehicle.speed:.2f} m/s | '
                            f'long_acc={longitudinal_acc:.2f} m/s^2 | '
                            f'acc_norm={acc_norm:.2f} m/s^2 | '
                            f'global_acc=({ax:.2f}, {ay:.2f}) m/s^2 | '
                            f'yaw={yaw:.2f} rad | '
                            f'pos=({vehicle.position.x:.2f}, {vehicle.position.y:.2f})'
                        )

                        self.last_log_time[track_id] = current_ros_time

            else:
                self.track_counts[track_id] = 0

        active_ids = []

        for track_id, last_seen_time in list(self.active_events.items()):
            if current_ros_time - last_seen_time <= self.event_duration:
                active_ids.append(track_id)
            else:
                del self.active_events[track_id]

        out_msg = String()
        out_msg.data = ",".join(sorted(active_ids))
        self.hard_brake_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardBrakeDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        empty_msg = String()
        empty_msg.data = ""
        node.hard_brake_pub.publish(empty_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()