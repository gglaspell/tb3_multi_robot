"""Regression tests for the follower's trailing-goal geometry."""

import math

from multi_robot_scripts.tb3_follow_tb1 import compute_trailing_goal

import pytest


def test_goal_trails_horizontal_heading() -> None:
    pose = compute_trailing_goal((-1.5, -0.5, 0.0), 0.5)

    assert pose == pytest.approx((-2.0, -0.5, 0.0))


def test_goal_trails_vertical_heading() -> None:
    pose = compute_trailing_goal((4.0, -3.0, math.pi / 2.0), 0.5)

    assert pose == pytest.approx((4.0, -3.5, math.pi / 2.0))


def test_goal_preserves_arbitrary_heading() -> None:
    x, y, yaw = compute_trailing_goal((2.0, 1.0, -1.2), 0.75)

    assert x == pytest.approx(2.0 - 0.75 * math.cos(-1.2))
    assert y == pytest.approx(1.0 - 0.75 * math.sin(-1.2))
    assert yaw == pytest.approx(-1.2)
