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
            "occupancy", "vx", "vy",
            "ax", "ay", "speed",
            "event", "ttc", "yaw_event"
        ]

        self.fig, self.axes = plt.subplots(3, 3, figsize=(16, 14))
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
        ys, xs = np.where(occupancy > 0)

        for i, ax in enumerate(self.axes.flat):
            ax.clear()

            name = self.channel_names[i]
            data = grid[i]

            ax.set_title(name, fontsize=14)
            ax.set_xlim(0, width)
            ax.set_ylim(0, height)
            ax.set_xlabel("grid x")
            ax.set_ylabel("grid y")

            if name == "occupancy":
                ax.imshow(
                    occupancy,
                    origin="lower",
                    cmap="gray",
                    vmin=0,
                    vmax=1
                )

            elif name in ["vx", "vy"]:
                values = data[ys, xs]
                max_abs = max(np.max(np.abs(values)), 0.1) if len(values) > 0 else 1

                ax.scatter(
                    xs,
                    ys,
                    c=values,
                    cmap="coolwarm",
                    vmin=-max_abs,
                    vmax=max_abs,
                    s=25
                )

            elif name in ["ax", "ay"]:
                values = data[ys, xs]
                max_abs = max(np.max(np.abs(values)), 0.01) if len(values) > 0 else 1

                ax.scatter(
                    xs,
                    ys,
                    c=values,
                    cmap="coolwarm",
                    vmin=-max_abs,
                    vmax=max_abs,
                    s=25
                )

            elif name == "speed":
                values = data[ys, xs]
                max_val = max(np.max(values), 0.1) if len(values) > 0 else 1

                ax.scatter(
                    xs,
                    ys,
                    c=values,
                    cmap="hot",
                    vmin=0,
                    vmax=max_val,
                    s=25
                )

            elif name == "event":
                event_y, event_x = np.where(data > 0)

                ax.imshow(
                    np.zeros_like(data),
                    origin="lower",
                    cmap="gray",
                    vmin=0,
                    vmax=1
                )

                ax.scatter(
                    event_x,
                    event_y,
                    c="red",
                    s=35
                )

            elif name == "ttc":
                valid_y, valid_x = np.where(data >= 0)
                values = data[valid_y, valid_x]

                ax.imshow(
                    np.zeros_like(data),
                    origin="lower",
                    cmap="gray",
                    vmin=0,
                    vmax=1
                )

                if len(values) > 0:
                    ax.scatter(
                        valid_x,
                        valid_y,
                        c=values,
                        cmap="RdYlGn_r",
                        vmin=0,
                        vmax=1.5,
                        s=35
                    )

            elif name == "yaw_event":
                yaw_y, yaw_x = np.where(data > 0)

                ax.imshow(
                    np.zeros_like(data),
                    origin="lower",
                    cmap="gray",
                    vmin=0,
                    vmax=1
                )

                ax.scatter(
                    yaw_x,
                    yaw_y,
                    c="magenta",
                    s=45
                )

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