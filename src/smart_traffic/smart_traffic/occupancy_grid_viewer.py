import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class OccupancyGridViewer(Node):

    def __init__(self):
        super().__init__('occupancy_grid_viewer')

        self.create_subscription(
            Float32MultiArray,
            'multi_channel_occupancy_grid',
            self.callback,
            10
        )

        self.channel_names = [
            "occupancy", "vx", "vy", "ax",
            "ay", "speed", "event", "ttc"
        ]

        self.fig, self.axes = plt.subplots(4, 2, figsize=(16, 14))
        plt.ion()
        plt.show()

        self.get_logger().info("Improved occupancy grid viewer started")

    def callback(self, msg):
        channels = msg.layout.dim[0].size
        height = msg.layout.dim[1].size
        width = msg.layout.dim[2].size

        grid = np.array(msg.data, dtype=np.float32).reshape(
            (channels, height, width)
        )

        occupancy = grid[0]

        for i, ax in enumerate(self.axes.flat):
            ax.clear()

            name = self.channel_names[i]
            data = grid[i].copy()

            # leere Zellen ausblenden, außer bei occupancy
            if name != "occupancy":
                data[occupancy == 0] = np.nan

            if name == "occupancy":
                im = ax.imshow(data, origin="lower", cmap="gray", vmin=0, vmax=1)

            elif name in ["vx", "vy"]:
                max_abs = np.nanmax(np.abs(data)) if np.any(~np.isnan(data)) else 1
                max_abs = max(max_abs, 0.1)
                im = ax.imshow(data, origin="lower", cmap="coolwarm",
                               vmin=-max_abs, vmax=max_abs)

            elif name in ["ax", "ay"]:
                max_abs = np.nanmax(np.abs(data)) if np.any(~np.isnan(data)) else 1
                max_abs = max(max_abs, 0.01)
                im = ax.imshow(data, origin="lower", cmap="coolwarm",
                               vmin=-max_abs, vmax=max_abs)

            elif name == "speed":
                max_val = np.nanmax(data) if np.any(~np.isnan(data)) else 1
                max_val = max(max_val, 0.1)
                im = ax.imshow(data, origin="lower", cmap="hot",
                               vmin=0, vmax=max_val)

            elif name == "event":
                im = ax.imshow(data, origin="lower", cmap="Reds", vmin=0, vmax=1)

            elif name == "ttc":
                data[data < 0] = np.nan
                im = ax.imshow(data, origin="lower", cmap="RdYlGn_r",
                               vmin=0, vmax=1.5)

            ax.set_title(name, fontsize=14)
            ax.set_xlabel("grid x")
            ax.set_ylabel("grid y")

        plt.tight_layout()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()