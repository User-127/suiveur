import math
from types import SimpleNamespace

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .lidar_plot_data import prepare_plot_data
from .metier import (
    CalculateurCerveauDistance,
    CalculateurCerveauOrientation,
    deform_distance,
    minimum_distance,
    minimum_distance_angle,
    to_polarcoord,
)


VALID_COLOR = '#00e5ff'
INVALID_COLOR = '#202830'
MIN_VALID_RANGE = 0.01  # meters; None uses the scan's range_min
MAX_VALID_RANGE = 1.00  # meters; None uses the scan's range_max
FIELD_OF_VIEW = math.radians(60.0)
PIPELINE_STAGES = (
    ('raw', 'Raw scan'),
    ('polar', 'Polar conversion'),
    ('field_of_view', '60° field-of-view filter'),
    ('orientation_deformation', 'Orientation: distance deformation'),
    ('orientation_minimum', 'Orientation: closest point'),
    ('distance_minimum', 'Distance: closest point'),
    ('orientation_correction', 'Orientation: angular correction'),
    ('distance_correction', 'Distance: linear correction'),
)


def prepare_pipeline_plot_data(scan, stage):
    """Prepare plot data for one of the brain node's LiDAR processing steps."""
    if stage == 'raw':
        return (*prepare_plot_data(
            scan,
            min_range=MIN_VALID_RANGE,
            max_range=MAX_VALID_RANGE,
        ), None, 'LiDAR measurements')

    polar_coords = to_polarcoord(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
    )
    finite_indices = [
        index
        for index, distance in enumerate(scan.ranges)
        if math.isfinite(distance)
    ]
    indexed_polars = list(zip(finite_indices, polar_coords))
    field_of_view_polars = [
        (index, polar)
        for index, polar in indexed_polars
        if abs(polar.angle) <= FIELD_OF_VIEW / 2
    ]
    orientation_coords = to_polarcoord(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
    )
    orientation_polars = [
        (index, polar)
        for index, polar in zip(finite_indices, orientation_coords)
        if abs(polar.angle) <= FIELD_OF_VIEW / 2
    ]
    distance_coords = to_polarcoord(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
    )
    distance_polars = [
        (index, polar)
        for index, polar in zip(finite_indices, distance_coords)
        if abs(polar.angle) <= FIELD_OF_VIEW / 2
    ]

    orientation_calculator = CalculateurCerveauOrientation(
        gain_proportionnel=3.0,
    )
    orientation_calculator.angle_vue = FIELD_OF_VIEW
    orientation_angle = orientation_calculator.mettre_a_jour_erreur(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
    )

    distance_calculator = CalculateurCerveauDistance(distance_cible=0.3)
    distance_calculator.angle_vue = FIELD_OF_VIEW
    distance_measurement = distance_calculator.mettre_a_jour_erreur(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
    )

    deform_distance(
        [polar for _, polar in orientation_polars],
        facteur_min=0.5,
        facteur_max=1.0,
        champ_vision=FIELD_OF_VIEW,
    )
    orientation_minimum_angle = minimum_distance_angle(
        [polar for _, polar in orientation_polars],
    )
    orientation_minimum = next(
        (
            (index, polar)
            for index, polar in orientation_polars
            if polar.angle == orientation_minimum_angle
        ),
        None,
    )
    distance_minimum_value = minimum_distance(
        [polar for _, polar in distance_polars],
    )
    distance_minimum_pair = next(
        (
            (index, polar)
            for index, polar in distance_polars
            if polar.distance == distance_minimum_value
        ),
        None,
    )

    if stage == 'polar':
        selected_polars = indexed_polars
        highlighted = None
        detail = 'Finite measurements converted to polar coordinates'
    elif stage == 'field_of_view':
        selected_polars = field_of_view_polars
        highlighted = None
        detail = 'Measurements outside ±30° are removed'
    elif stage in (
        'orientation_deformation',
        'orientation_minimum',
        'orientation_correction',
    ):
        selected_polars = orientation_polars
        highlighted = (
            orientation_minimum
            if stage in ('orientation_minimum', 'orientation_correction')
            else None
        )
        detail = 'Distances are weighted from 0.5× at the center to 1× at ±30°'
        if stage == 'orientation_correction':
            correction = orientation_calculator.calculer_correction()
            detail = (
                f'Closest angle: {orientation_angle:.3f} rad; '
                f'angular correction: {correction:.3f} rad/s'
                if orientation_angle is not None
                else 'No finite measurement in the field of view'
            )
    elif stage in ('distance_minimum', 'distance_correction'):
        selected_polars = distance_polars
        highlighted = distance_minimum_pair
        detail = 'Closest measurement in the field of view'
        if stage == 'distance_correction':
            correction = distance_calculator.calculer_correction()
            detail = (
                f'Closest distance: {distance_measurement:.3f} m; '
                f'linear correction: {correction:.3f} m/s'
                if distance_measurement is not None
                else 'No finite measurement in the field of view'
            )
    else:
        raise ValueError(f'Unknown LiDAR pipeline stage: {stage}')

    stage_ranges = [math.inf] * len(scan.ranges)
    for index, polar in selected_polars:
        stage_ranges[index] = polar.distance

    stage_scan = SimpleNamespace(
        ranges=stage_ranges,
        angle_min=scan.angle_min,
        angle_increment=scan.angle_increment,
        range_min=-math.inf,
        range_max=math.inf,
    )
    plot_data = prepare_plot_data(stage_scan)
    angles, profile_ranges, ray_ranges, valid_ranges, display_range = plot_data
    highlighted_point = None
    if highlighted is not None:
        index, polar = highlighted
        if 0 <= polar.distance <= display_range and valid_ranges[index]:
            highlighted_point = (polar.angle, polar.distance)
    return (
        angles,
        profile_ranges,
        ray_ranges,
        valid_ranges,
        display_range,
        highlighted_point,
        detail,
    )


class LidarVisualizerNode(Node):
    """Display TurtleBot LaserScan data as a polar profile and rays."""

    def __init__(self):
        super().__init__('lidar_visualizer')
        self.latest_scan = None
        self.displayed_scan = None
        self.plot_needs_update = True
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
        self.figure.subplots_adjust(top=0.80)
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
        self.highlight, = self.axes.plot(
            [],
            [],
            marker='o',
            color='#ffcc00',
            markersize=7,
            linestyle='none',
            zorder=3,
        )
        self.rays = LineCollection([], linewidths=0.8, zorder=1)
        self.axes.add_collection(self.rays)
        self.axes.plot(0.0, 0.0, marker='o', color='#ffffff', markersize=3)
        self.pipeline_stage = 'raw'
        self._create_stage_dropdown()

    def _create_stage_dropdown(self):
        """Add an interactive stage selector above the polar plot."""
        self.dropdown_axes = self.figure.add_axes([0.04, 0.87, 0.46, 0.07])
        self.dropdown_axes.set_axis_off()
        self.dropdown_axes.add_patch(Rectangle(
            (0.0, 0.0),
            1.0,
            1.0,
            facecolor='#151b22',
            edgecolor='#687582',
        ))
        self.dropdown_label = self.dropdown_axes.text(
            0.04,
            0.5,
            'Pipeline step: Raw scan  ▼',
            color='white',
            va='center',
        )

        self.dropdown_menu = self.figure.add_axes([0.04, 0.58, 0.46, 0.29])
        self.dropdown_menu.set_axis_off()
        self.dropdown_menu.set_facecolor('#151b22')
        stage_count = len(PIPELINE_STAGES)
        for menu_index in range(stage_count):
            y = menu_index / stage_count
            self.dropdown_menu.add_patch(Rectangle(
                (0.0, y),
                1.0,
                1.0 / stage_count,
                facecolor='#202830' if menu_index % 2 == 0 else '#151b22',
                edgecolor='#39434d',
            ))
            _, label = PIPELINE_STAGES[stage_count - 1 - menu_index]
            self.dropdown_menu.text(
                0.04,
                y + 0.5 / stage_count,
                label,
                color='white',
                va='center',
                fontsize=9,
            )
        self.dropdown_menu.set_visible(False)
        self.figure.canvas.mpl_connect(
            'button_press_event',
            self._on_dropdown_click,
        )

    def _on_dropdown_click(self, event):
        """Toggle the stage menu or select a processing step."""
        if event.inaxes is self.dropdown_axes:
            self.dropdown_menu.set_visible(not self.dropdown_menu.get_visible())
            self.figure.canvas.draw_idle()
            return

        if event.inaxes is self.dropdown_menu and event.ydata is not None:
            menu_index = min(
                int(event.ydata * len(PIPELINE_STAGES)),
                len(PIPELINE_STAGES) - 1,
            )
            stage_index = len(PIPELINE_STAGES) - 1 - menu_index
            self.pipeline_stage = PIPELINE_STAGES[stage_index][0]
            self.dropdown_label.set_text(
                f'Pipeline step: {PIPELINE_STAGES[stage_index][1]}  ▼'
            )
            self.dropdown_menu.set_visible(False)
            self.plot_needs_update = True
            self.update_plot()

    def scan_callback(self, scan):
        """Keep the newest scan for rendering from the GUI event loop."""
        self.latest_scan = scan
        self.plot_needs_update = True

    def update_plot(self):
        """Refresh the plot from the newest scan, if one has arrived."""
        if self.latest_scan is not None:
            self.displayed_scan = self.latest_scan
            self.latest_scan = None
        elif not self.plot_needs_update:
            return

        if self.displayed_scan is None:
            return

        (
            angles,
            profile_ranges,
            ray_ranges,
            valid_ranges,
            display_range,
            highlighted_point,
            detail,
        ) = prepare_pipeline_plot_data(
            self.displayed_scan,
            self.pipeline_stage,
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
        if highlighted_point is None:
            self.highlight.set_data([], [])
        else:
            self.highlight.set_data(
                [highlighted_point[0]],
                [highlighted_point[1]],
            )
        stage_label = dict(PIPELINE_STAGES)[self.pipeline_stage]
        self.axes.set_title(
            f'TurtleBot LiDAR - {stage_label}\n{detail}',
            color='white',
            fontsize=10,
        )
        if not self.initial_range_set:
            min_range = (
                max(
                    self.displayed_scan.range_min,
                    MIN_VALID_RANGE
                    if MIN_VALID_RANGE is not None
                    else self.displayed_scan.range_min,
                )
                if self.pipeline_stage == 'raw'
                else 0.0
            )
            self.axes.set_ylim(min_range, display_range)
            self.initial_range_set = True
        self.figure.canvas.draw_idle()
        self.plot_needs_update = False

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
