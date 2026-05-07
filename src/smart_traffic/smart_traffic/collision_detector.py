import rclpy
from rclpy.node import Node
from smart_traffic_interfaces.msg import VehicleStatus
import math


class EventDetector(Node):

    def __init__(self):
        super().__init__('event_detector')

        self.subscription = self.create_subscription(
            VehicleStatus,
            'vehicle_status',
            self.callback,
            10
        )

        # store objects per frame
        self.current_frame = []

        # timing control (to detect new frame)
        self.last_time = self.get_clock().now()

    # ================================
    # CALLBACK
    # ================================
    def callback(self, msg):

        now = self.get_clock().now()

        # If time gap → new frame
        if (now - self.last_time).nanoseconds > 1e8:  # ~0.1 sec
            if len(self.current_frame) > 1:
                self.detect_events(self.current_frame)

            self.current_frame = []
            self.last_time = now

        self.current_frame.append(msg)

    # ================================
    # EVENT DETECTION
    # ================================
    def detect_events(self, objects):

        for i in range(len(objects)):
            for j in range(i + 1, len(objects)):

                a = objects[i]
                b = objects[j]

                # Position difference
                dx = a.position.x - b.position.x
                dy = a.position.y - b.position.y
                distance = math.sqrt(dx**2 + dy**2)

                # Relative velocity
                dvx = a.velocity_x - b.velocity_x
                dvy = a.velocity_y - b.velocity_y

                # Check if moving toward each other
                dot = dx * dvx + dy * dvy

                rel_speed = math.sqrt(dvx**2 + dvy**2)
                if rel_speed < 0.1:
                    continue

                ttc = distance / rel_speed

                # 🚨 NEAR COLLISION CONDITION
                if distance < 2.0 and dot < 0 and ttc < 1.5:
                    self.report_event(a, b, distance, ttc)

    # ================================
    # REPORT EVENT
    # ================================
    def report_event(self, a, b, distance, ttc):

        self.get_logger().warn(
            f"⚠️ Near Collision detected!\n"
            f"Actors: {a.track_id} & {b.track_id}\n"
            f"Distance: {distance:.2f} m\n"
            f"TTC: {ttc:.2f} s\n"
        )


# ================================
# MAIN
# ================================
def main(args=None):
    rclpy.init(args=args)
    node = EventDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down detector...')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()