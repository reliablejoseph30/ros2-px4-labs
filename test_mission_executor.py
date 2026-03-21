#!/usr/bin/env python3
"""
Unit tests for MissionState enum and mission logic.
Run with: pytest test_mission_executor.py -v
No ROS2 running required.
"""
import sys, os
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from mission_executor import MissionState


# ── TC-SM-01: All required states exist ──────────────────────────────────
def test_all_states_exist():
    """
    TC-SM-01
    Precondition : MissionState enum imported
    Expected     : all 7 states are present
    Pass criterion: no AttributeError, all values distinct
    """
    required = ['IDLE', 'ARMED', 'TAKEOFF', 'NAVIGATING',
                'STATIONKEEPING', 'RTL', 'COMPLETE', 'FAILSAFE'] # added
    for name in required:
        assert hasattr(MissionState, name), f'Missing state: {name}'


# ── TC-SM-02: States are distinct (no value collision) ────────────────────
def test_states_are_distinct():
    """
    TC-SM-02
    Expected : all state values are unique
    """
    values = [s.value for s in MissionState]
    assert len(values) == len(set(values)), (
        f'Duplicate state values detected: {values}'
    )


# ── TC-SM-03: State comparison works as expected ──────────────────────────
def test_state_comparison():
    """
    TC-SM-03
    Expected : states compare equal to themselves, unequal to others
    """
    assert MissionState.IDLE == MissionState.IDLE
    assert MissionState.IDLE != MissionState.NAVIGATING
    assert MissionState.COMPLETE != MissionState.IDLE


# ── TC-SM-04: State name attribute is correct ─────────────────────────────
def test_state_name():
    """
    TC-SM-04
    Expected : .name returns the string used in logs and /mission/state topic
    """
    assert MissionState.NAVIGATING.name == 'NAVIGATING'
    assert MissionState.STATIONKEEPING.name == 'STATIONKEEPING'
    assert MissionState.COMPLETE.name == 'COMPLETE'


# ── TC-SM-05: Service rejection logic (no waypoints) ─────────────────────
class MockResponse:
    """Minimal mock of std_srvs/Trigger response."""
    def __init__(self):
        self.success = None
        self.message = ''


class MockExecutor:
    """
    Minimal mock that reproduces the handle_start_mission logic without
    instantiating a full ROS2 node.
    """
    def __init__(self, state=MissionState.IDLE, waypoints=None):
        self.state = state
        self.waypoints = waypoints or []

    def handle_start_mission(self, request, response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Cannot start — currently in state {self.state.name}'
            return response
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints received yet.'
            return response
        self.state = MissionState.ARMED
        response.success = True
        response.message = 'Mission accepted.'
        return response


def test_service_rejected_when_no_waypoints():
    """
    TC-SM-05
    Precondition : executor in IDLE, no waypoints loaded
    Stimulus     : call handle_start_mission
    Expected     : response.success == False with informative message
    """
    executor = MockExecutor(state=MissionState.IDLE, waypoints=[])
    response = MockResponse()
    executor.handle_start_mission(None, response)
    assert response.success == False
    assert 'waypoints' in response.message.lower()


# ── TC-SM-06: Service rejected when not in IDLE ───────────────────────────
def test_service_rejected_when_not_idle():
    """
    TC-SM-06
    Precondition : executor in NAVIGATING state
    Stimulus     : call handle_start_mission
    Expected     : response.success == False, message mentions current state
    """
    executor = MockExecutor(state=MissionState.NAVIGATING, waypoints=['wp1'])
    response = MockResponse()
    executor.handle_start_mission(None, response)
    assert response.success == False
    assert 'NAVIGATING' in response.message


# ── TC-SM-07: Service accepted when IDLE with waypoints ───────────────────
def test_service_accepted_when_ready():
    """
    TC-SM-07
    Precondition : executor in IDLE, waypoints loaded
    Stimulus     : call handle_start_mission
    Expected     : response.success == True, state transitions to ARMED
    """
    executor = MockExecutor(state=MissionState.IDLE, waypoints=['wp1', 'wp2'])
    response = MockResponse()
    executor.handle_start_mission(None, response)
    assert response.success == True
    assert executor.state == MissionState.ARMED


# ── TC-SM-08: Calling service twice is rejected on second call ────────────
def test_service_idempotent_rejection():
    """
    TC-SM-08
    Precondition : call service once successfully (transitions to ARMED)
    Stimulus     : call service again
    Expected     : second call is rejected, state does not change
    """
    executor = MockExecutor(state=MissionState.IDLE, waypoints=['wp1'])
    r1, r2 = MockResponse(), MockResponse()
    executor.handle_start_mission(None, r1)
    assert r1.success == True
    executor.handle_start_mission(None, r2)
    assert r2.success == False
    assert executor.state == MissionState.ARMED  # unchanged