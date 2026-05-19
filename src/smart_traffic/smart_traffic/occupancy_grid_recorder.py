import os
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class OccupancyGridRecorder(Node):

    def __init__(self):
        super().__init__('occupancy_grid_recorder')

        # =====================================================
        # SETTINGS
        # =====================================================

        self.num_channels = 10
        self.grid_height = 115
        self.grid_width = 360

        # save every nth frame
        self.save_every_n = 1

        self.frame_counter = 0
        self.saved_counter = 0

        # =====================================================
        # OUTPUT DIRECTORY
        # =====================================================

        self.output_dir = os.path.expanduser(
            '~/ros2_ws/grid_dataset'
        )

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(
            f'Saving occupancy grids to: {self.output_dir}'
        )

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        self.create_subscription(
            Float32MultiArray,
            'multi_channel_occupancy_grid',
            self.grid_callback,
            10
        )

        self.get_logger().info(
            'Occupancy Grid Recorder started'
        )

    def grid_callback(self, msg):

        self.frame_counter += 1

        if self.frame_counter % self.save_every_n != 0:
            return

        try:
            grid = np.array(
                msg.data,
                dtype=np.float32
            ).reshape(
                (
                    self.num_channels,
                    self.grid_height,
                    self.grid_width
                )
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to reshape grid: {e}'
            )
            return

        filename = os.path.join(
            self.output_dir,
            f'grid_{self.saved_counter:06d}.npy'
        )

        np.save(filename, grid)

        if self.saved_counter % 100 == 0:
            self.get_logger().info(
                f'Saved {self.saved_counter} grids'
            )

        self.saved_counter += 1


def main(args=None):

    rclpy.init(args=args)

    node = OccupancyGridRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Shutting down occupancy grid recorder...'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()