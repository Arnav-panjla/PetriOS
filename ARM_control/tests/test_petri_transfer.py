"""Geometry and orientation checks for the one-slot transfer preview."""

import math
import threading

import numpy as np
import pytest

from robot_arm.interfaces.web_gui import ArmService, IKBridge
from robot_arm.motion.petri_transfer import (
    DISH_NORMAL_LOCAL, SIMULATION_SCENE, Station, plan_transfer,
)


def test_station_boundary_offsets() -> None:
    source = Station.from_dict(SIMULATION_SCENE["source"])
    pregrasp = source.point(0.050)
    extracted_center = source.point(source.inside_m + 0.060)
    assert np.linalg.norm(pregrasp - np.array(source.center_m)) == pytest.approx(0.050)
    assert np.linalg.norm(extracted_center - np.array(source.center_m)) == pytest.approx(0.105)


def test_complete_simulation_route_stays_level_and_places_center() -> None:
    mod = IKBridge().mod
    assert mod is not None
    source = Station.from_dict(SIMULATION_SCENE["source"])
    destination = Station.from_dict(SIMULATION_SCENE["destination"])
    home = [88.5, 36.0, 56.5, 151.0, 116.25, 41.5]
    steps = plan_transfer(mod, home, home, source, destination)
    assert [step.event for step in steps if step.event] == ["attach", "detach"]
    assert steps[0].stage == "pregrasp"
    assert steps[-1].stage == "home"
    assert steps[0].servo_deg[4] == pytest.approx(26.25, abs=0.01)
    assert [step.servo_deg[5] for step in steps if step.stage in ("open", "grip", "release")] == [10.0, 95.0, 10.0]
    pregrasp = steps[0]
    grasp = [step for step in steps if step.stage == "enter_grasp"][-1]
    entry_distance = np.dot(
        np.array(pregrasp.tcp_m[:2]) - np.array(grasp.tcp_m[:2]),
        np.array(source.outward_xy),
    )
    assert entry_distance == pytest.approx(0.030, abs=0.002)
    pickup = [step for step in steps if step.stage == "pickup_lift"][-1]
    assert pickup.dish_center_m is not None
    assert pickup.dish_center_m[2] == pytest.approx(source.center_m[2] + 0.003, abs=0.002)
    assert [step.stage for step in steps].index("grip") < [step.stage for step in steps].index("pickup_lift")
    extraction = [step for step in steps if step.stage == "extract"][-1]
    assert extraction.dish_center_m is not None
    signed_clearance = np.dot(
        np.array(extraction.dish_center_m[:2]) - np.array(source.center_m[:2]),
        np.array(source.outward_xy),
    ) - source.inside_m
    assert signed_clearance == pytest.approx(0.060, abs=0.002)
    # Once extracted, every source-side carry waypoint must remain at least as
    # far from the carousel as the extracted dish. This prevents the old path
    # from reversing into a vertical divider before turning to the camera.
    source_side_carry = [step for step in steps if step.stage in ("carry_out", "carry_arc")]
    extracted_projection = np.dot(
        np.array(extraction.dish_center_m[:2]) - np.array(source.center_m[:2]),
        np.array(source.outward_xy),
    )
    for step in source_side_carry:
        assert step.dish_center_m is not None
        projection = np.dot(
            np.array(step.dish_center_m[:2]) - np.array(source.center_m[:2]),
            np.array(source.outward_xy),
        )
        assert projection >= extracted_projection - 0.002
    placed = [step for step in steps if step.stage == "insert"][-1]
    assert placed.dish_center_m is not None
    assert np.linalg.norm(np.array(placed.dish_center_m) - destination.center_m) < 0.002
    insertion = [step for step in steps if step.stage == "insert"]
    assert insertion
    assert all(abs(step.dish_center_m[2] - destination.center_m[2]) < 0.002
               for step in insertion if step.dish_center_m is not None)
    outside = [step for step in steps if step.stage == "lower_outside"][-1]
    assert outside.dish_center_m is not None
    assert outside.dish_center_m[1] > destination.center_m[1] + destination.inside_m + 0.045
    released = [step for step in steps if step.stage == "release"][-1]
    cleared = [step for step in steps if step.stage == "clear_dish"][-1]
    assert cleared.tcp_m[2] == pytest.approx(released.tcp_m[2] - 0.005, abs=0.002)
    for step in steps[:-1]:
        transform, _ = mod.fk(mod.to_q(step.servo_deg[:5]), mod.TCP_ESTIMATE)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        tilt = math.degrees(math.acos(float(np.clip(normal[2], -1, 1))))
        assert tilt < 1.0


def test_unreachable_fixture_is_rejected() -> None:
    mod = IKBridge().mod
    assert mod is not None
    source = Station.from_dict({"center_m": [0.7, -0.7, 0.2], "outward_xy": [-1, 1]})
    destination = Station.from_dict(SIMULATION_SCENE["destination"])
    home = [88.5, 36.0, 56.5, 151.0, 116.25, 41.5]
    with pytest.raises(ValueError, match="no level IK"):
        plan_transfer(mod, home, home, source, destination)


def test_hardware_transfer_can_start_after_normal_startup(monkeypatch) -> None:
    service = ArmService.__new__(ArmService)
    service.simulation_only = False
    service.arm = type("Arm", (), {"ready": True})()
    service._petri_lock = threading.Lock()
    service._lock = threading.Lock()
    service._busy = False
    service._pending = {}
    service._petri_active = False
    service._petri_bridge_seen = 0.0
    service._petri_mirror_gazebo = False
    service._petri_stage = "idle"
    service._petri_message = "Petri path is idle."
    service._petri_event_seq = 0
    service._petri_event_kind = None
    service._petri_event_ack = 0
    monkeypatch.setattr(threading.Thread, "start", lambda _thread: None)
    service.start_petri_transfer(1.0)
    assert service._petri_active
