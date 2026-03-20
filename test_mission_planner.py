#!/usr/bin/env python3
"""
Unit tests for generate_lawnmower_waypoints().
Run with: pytest test_mission_planner.py -v
No ROS2 or Gazebo required.
"""
import sys
import os
import pytest
import math

# Add the parent directory so we can import from mission_planner.py
sys.path.insert(0, os.path.dirname(__file__))
from mission_planner import generate_lawnmower_waypoints


# ── TC-WP-01: Correct waypoint count ─────────────────────────────────────
def test_waypoint_count_default_params():
    """
    TC-WP-01
    Precondition : default parameters (4 lines, 24m length, 3m spacing)
    Stimulus     : call generate_lawnmower_waypoints with default values
    Expected     : (24/3)+1 = 9 waypoints per line * 4 lines = 36 waypoints
    Pass criterion: len(waypoints) == 36
    """
    wps = generate_lawnmower_waypoints(
        origin_x=0.0, origin_y=0.0, altitude=10.0,
        line_spacing=5.0, waypoint_spacing=3.0,
        num_lines=4, line_length=24.0
    )
    expected = (int(24.0 / 3.0) + 1) * 4
    assert len(wps) == expected, f'Expected {expected} waypoints, got {len(wps)}'


# ── TC-WP-02: All waypoints at correct altitude ───────────────────────────
def test_all_waypoints_at_correct_altitude():
    """
    TC-WP-02
    Precondition : altitude=7.5 (non-default to catch hardcoding)
    Stimulus     : generate waypoints
    Expected     : every waypoint has z == 7.5
    Pass criterion: all(wp[2] == 7.5 for wp in wps)
    """
    altitude = 7.5
    wps = generate_lawnmower_waypoints(0, 0, altitude, 5.0, 3.0, 4, 24.0)
    assert all(wp[2] == altitude for wp in wps), (
        f'Not all waypoints at z={altitude}. Found: {set(wp[2] for wp in wps)}'
    )


# ── TC-WP-03: Lawnmower direction alternates ─────────────────────────────
def test_lawnmower_direction_alternates():
    """
    TC-WP-03
    Precondition : 2 lines, 2 waypoints per line (spacing = line_length)
    Stimulus     : generate waypoints
    Expected     : line 0 goes in +y direction, line 1 in -y direction
    Pass criterion: first line's last y > first y; second line's last y < first y
    """
    wps = generate_lawnmower_waypoints(
        origin_x=0.0, origin_y=0.0, altitude=10.0,
        line_spacing=5.0, waypoint_spacing=10.0,
        num_lines=2, line_length=10.0
    )
    # Line 0: indices 0,1 — should go positive y
    assert wps[1][1] > wps[0][1], 'Line 0 should travel in +y direction'
    # Line 1: indices 2,3 — should go negative y (back)
    assert wps[3][1] < wps[2][1], 'Line 1 should travel in -y direction'


# ── TC-WP-04: Correct line spacing in x ──────────────────────────────────
def test_line_spacing_in_x():
    """
    TC-WP-04
    Precondition : line_spacing=5.0, num_lines=4
    Stimulus     : generate waypoints
    Expected     : x values of first waypoint on each line are 0, 5, 10, 15
    Pass criterion: line start x values match expected sequence
    """
    spacing = 5.0
    num_lines = 4
    wps = generate_lawnmower_waypoints(0, 0, 10.0, spacing, 3.0, num_lines, 9.0)
    wps_per_line = int(9.0 / 3.0) + 1  # 4
    for i in range(num_lines):
        first_wp_of_line = wps[i * wps_per_line]
        expected_x = i * spacing
        assert abs(first_wp_of_line[0] - expected_x) < 1e-9, (
            f'Line {i}: expected x={expected_x}, got x={first_wp_of_line[0]}'
        )


# ── TC-WP-05: Custom origin is respected ─────────────────────────────────
def test_custom_origin():
    """
    TC-WP-05
    Precondition : origin_x=10.0, origin_y=5.0
    Stimulus     : generate waypoints
    Expected     : first waypoint is (10.0, 5.0, altitude)
    Pass criterion: wps[0] == (10.0, 5.0, 10.0)
    """
    wps = generate_lawnmower_waypoints(10.0, 5.0, 10.0, 5.0, 3.0, 2, 10.0)
    assert wps[0][0] == 10.0, f'Expected origin_x=10.0, got {wps[0][0]}'
    assert wps[0][1] == 5.0, f'Expected origin_y=5.0, got {wps[0][1]}'


# ── TC-WP-06: Single line, single waypoint ───────────────────────────────
def test_single_line_single_waypoint():
    """
    TC-WP-06 Edge case: num_lines=1, waypoint_spacing >= line_length
    Expected : exactly 1 waypoint at origin altitude
    """
    wps = generate_lawnmower_waypoints(0, 0, 10.0, 5.0, 20.0, 1, 10.0)
    assert len(wps) == 1, f'Expected 1 waypoint, got {len(wps)}'
    assert wps[0] == (0.0, 0.0, 10.0)


# ── TC-WP-07: Zero lines returns empty list ───────────────────────────────
def test_zero_lines_returns_empty():
    """
    TC-WP-07 Edge case: num_lines=0
    Expected : empty list, no exception
    """
    wps = generate_lawnmower_waypoints(0, 0, 10.0, 5.0, 3.0, 0, 24.0)
    assert wps == [], f'Expected empty list, got {wps}'


# ── TC-WP-08: All waypoints are 3-tuples of floats ───────────────────────
def test_waypoints_are_float_tuples():
    """
    TC-WP-08 Type correctness test
    Expected : every element is a tuple of 3 floats
    """
    wps = generate_lawnmower_waypoints(0, 0, 10.0, 5.0, 3.0, 3, 15.0)
    for i, wp in enumerate(wps):
        assert len(wp) == 3, f'WP {i} has {len(wp)} elements, expected 3'
        for j, val in enumerate(wp):
            assert isinstance(val, (int, float)), (
                f'WP {i}[{j}] is {type(val)}, expected numeric'
            )