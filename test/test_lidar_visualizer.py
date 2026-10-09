import math
from types import SimpleNamespace

import pytest

from suiveur.lidar_plot_data import prepare_plot_data


def test_prepare_plot_data_marks_out_of_range_measurements_invalid():
    scan = SimpleNamespace(
        ranges=[1.0, math.inf, 0.05, 2.0],
        angle_min=-math.pi / 2,
        angle_increment=math.pi / 4,
        range_min=0.1,
        range_max=1.5,
    )

    angles, profile, rays, validity, display_range = prepare_plot_data(scan)

    assert angles == pytest.approx(
        [-math.pi / 2, -math.pi / 4, 0.0, math.pi / 4]
    )
    assert profile[0] == 1.0
    assert all(math.isnan(value) for value in profile[1:])
    assert rays == [1.0, 1.5, 1.5, 1.5]
    assert validity == [True, False, False, False]
    assert display_range == 1.5


def test_prepare_plot_data_uses_furthest_valid_range_when_max_is_infinite():
    scan = SimpleNamespace(
        ranges=[0.5, 2.0],
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.1,
        range_max=math.inf,
    )

    _, profile, rays, validity, display_range = prepare_plot_data(scan)

    assert profile == [0.5, 2.0]
    assert rays == [0.5, 2.0]
    assert validity == [True, True]
    assert display_range == 2.0


def test_prepare_plot_data_applies_configured_valid_range():
    scan = SimpleNamespace(
        ranges=[0.2, 0.5, 1.0, 1.8],
        angle_min=0.0,
        angle_increment=math.pi / 4,
        range_min=0.1,
        range_max=2.0,
    )

    _, profile, rays, validity, display_range = prepare_plot_data(
        scan,
        min_range=0.4,
        max_range=1.2,
    )

    assert math.isnan(profile[0])
    assert profile[1:3] == [0.5, 1.0]
    assert math.isnan(profile[3])
    assert rays == [1.2, 0.5, 1.0, 1.2]
    assert validity == [False, True, True, False]
    assert display_range == 1.2
