import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .lidar_plot_data import prepare_plot_data


VALID_COLOR = '#00e5ff'
INVALID_COLOR = '#202830'
MIN_VALID_RANGE = 0.01  # meters; None uses the scan's range_min
MAX_VALID_RANGE = 1.00  # meters; None uses the scan's range_max


class LidarVisualizerNode(Node):
    """Display TurtleBot LaserScan data as a polar profile and rays."""

    def __init__(self):
        super().__init__('lidar_visualizer')
        self.latest_scan = None
        self.initial_range_set = False
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        plt.ion()
        self.figure, self.axes = plt.subplots(
            subplot_kw={'projection': 'polar'},
        )
        self.figure.canvas.manager.set_window_title('TurtleBot LiDAR')
        self.axes.set_theta_zero_location('N')
        self.axes.set_theta_direction(1)
        self.axes.set_title('TurtleBot LiDAR - /scan', color='white')
        self.axes.set_facecolor('#0b0f14')
        self.figure.set_facecolor('#0b0f14')
        self.axes.tick_params(colors='#aab4be')
        self.axes.grid(color='#39434d', alpha=0.7)
        self.profile, = self.axes.plot(
            [],
            [],
            color=VALID_COLOR,
            linewidth=1.6,
            zorder=2,
        )
        self.rays = LineCollection([], linewidths=0.8, zorder=1)
        self.axes.add_collection(self.rays)
        self.axes.plot(0.0, 0.0, marker='o', color='#ffffff', markersize=3)
        self.figure.tight_layout()

    def scan_callback(self, scan):
        """Keep the newest scan for rendering from the GUI event loop."""
        self.latest_scan = scan

    def update_plot(self):
        """Refresh the plot from the newest scan, if one has arrived."""
        if self.latest_scan is None:
            return

        angles, profile_ranges, ray_ranges, valid_ranges, display_range = (
            prepare_plot_data(
                self.latest_scan,
                min_range=MIN_VALID_RANGE,
                max_range=MAX_VALID_RANGE,
            )
        )
        segments = [
            ((angle, 0.0), (angle, ray_range))
            for angle, ray_range in zip(angles, ray_ranges)
        ]
        self.rays.set_segments(segments)
        self.rays.set_color([
            VALID_COLOR if is_valid else INVALID_COLOR
            for is_valid in valid_ranges
        ])
        self.profile.set_data(angles, profile_ranges)
        if not self.initial_range_set:
            min_range = max(
                self.latest_scan.range_min,
                MIN_VALID_RANGE
                if MIN_VALID_RANGE is not None
                else self.latest_scan.range_min,
            )
            self.axes.set_ylim(min_range, display_range)
            self.initial_range_set = True
        self.figure.canvas.draw_idle()
        self.latest_scan = None

    def window_is_open(self):
        """Return whether the user has left the plot window open."""
        return plt.fignum_exists(self.figure.number)


def main(args=None):
    """Run the LiDAR visualizer and its GUI event loop."""
    rclpy.init(args=args)
    node = None
    try:
        node = LidarVisualizerNode()
        while rclpy.ok() and node.window_is_open():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.update_plot()
            plt.pause(0.001)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
