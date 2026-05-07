import math

import rclpy
from rclpy.node import Node

from smart_traffic_interfaces.msg import VehicleStatus


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HardBrakeDetector(Node):
    def __init__(self):
        super().__init__('hard_brake_detector')

        self.declare_parameter('brake_threshold', -3.0)
        self.declare_parameter('confirm_frames', 3)

        self.brake_threshold = float(self.get_parameter('brake_threshold').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)

        self.subscription = self.create_subscription(
            VehicleStatus,
            '/vehicle_status',
            self.vehicle_status_callback,
            10
        )

        self.track_counts = {}
        self.reported_tracks = set()

        self.get_logger().info(
            'Hard brake detector started | '
            f'longitudinal_acc < {self.brake_threshold}, '
            f'confirm_frames={self.confirm_frames}'
        )

    def vehicle_status_callback(self, msg: VehicleStatus):
        track_id = msg.track_id

        if track_id in self.reported_tracks:
            return

        if track_id not in self.track_counts:
            self.track_counts[track_id] = 0

        ax = msg.acceleration_x
        ay = msg.acceleration_y

        yaw = yaw_from_quaternion(msg.orientation)

        acc_norm = math.sqrt(ax**2 + ay**2)

        longitudinal_acc = ax * math.cos(yaw) + ay * math.sin(yaw)

        is_hard_brake = longitudinal_acc < self.brake_threshold

        if is_hard_brake:
            self.track_counts[track_id] += 1

            if self.track_counts[track_id] >= self.confirm_frames:
                self.get_logger().warn(
                    f'HARD BRAKE DETECTED | '
                    f'track_id={track_id} | '
                    f'category={msg.category} | '
                    f'speed={msg.speed:.2f} m/s | '
                    f'long_acc={longitudinal_acc:.2f} m/s^2 | '
                    f'acc_norm={acc_norm:.2f} m/s^2 | '
                    f'global_acc=({ax:.2f}, {ay:.2f}) m/s^2 | '
                    f'yaw={yaw:.2f} rad | '
                    f'pos=({msg.position.x:.2f}, {msg.position.y:.2f})'
                )

                self.reported_tracks.add(track_id)
                del self.track_counts[track_id]
        else:
            self.track_counts[track_id] = 0


def main(args=None):
    rclpy.init(args=args)

    node = HardBrakeDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
