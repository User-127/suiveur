import math


def prepare_plot_data(scan, min_range=None, max_range=None):
    """Prepare angles, valid profile points, and ray lengths for plotting."""
    ranges = scan.ranges
    valid_min = max(
        scan.range_min,
        min_range if min_range is not None else scan.range_min,
    )
    valid_max = min(
        scan.range_max,
        max_range if max_range is not None else scan.range_max,
    )
    valid_ranges = [
        math.isfinite(distance)
        and valid_min <= distance <= valid_max
        for distance in ranges
    ]

    if math.isfinite(valid_max) and valid_max > 0.0:
        display_range = valid_max
    else:
        display_range = max(
            (
                distance
                for distance, is_valid in zip(ranges, valid_ranges)
                if is_valid
            ),
            default=1.0,
        )
        display_range = max(display_range, 1.0)

    angles = [
        scan.angle_min + index * scan.angle_increment
        for index in range(len(ranges))
    ]
    profile_ranges = [
        distance if is_valid else math.nan
        for distance, is_valid in zip(ranges, valid_ranges)
    ]
    ray_ranges = [
        distance if is_valid else display_range
        for distance, is_valid in zip(ranges, valid_ranges)
    ]
    return angles, profile_ranges, ray_ranges, valid_ranges, display_range
