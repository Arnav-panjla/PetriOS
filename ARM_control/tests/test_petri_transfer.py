"""Geometry and orientation checks for the one-slot transfer preview."""

import math
import struct
import threading
from pathlib import Path

import numpy as np
import pytest

from robot_arm.interfaces.web_gui import ArmService, IKBridge
from robot_arm.motion.petri_transfer import (
    DISH_NORMAL_LOCAL, SIMULATION_SCENE, Station, plan_transfer,
)
from robot_arm.motion.session import SessionRecorder


def test_qr_hold_recording_matches_current_transfer() -> None:
    home = [88.5, 69.0, 56.475, 152.0, 116.25, 41.4]
    scene = SIMULATION_SCENE
    steps = plan_transfer(
        IKBridge().mod, home, home,
        Station.from_dict(scene["source"]), Station.from_dict(scene["destination"]),
    )
    recordings = Path(__file__).resolve().parents[1] / "recordings"
    points = SessionRecorder(recordings).load("session_petri_transfer_24_qr_hold.csv")
    assert len(steps) == len(points) == 24
    for step, point in zip(steps, points, strict=True):
        assert point["time"] == step.stage
        assert point["angles"] == pytest.approx(step.servo_deg, abs=0.0005)
    assert points[0]["angles"][:5] == points[1]["angles"][:5]
    assert [p["hold_s"] for p in points] == [0, 6] + [0] * 22
    assert [(p["n"], p["event"]) for p in points if p["event"]] == [
        (4, "attach"), (21, "detach"),
    ]


def test_station_boundary_offsets() -> None:
    source = Station.from_dict(SIMULATION_SCENE["source"])
    pregrasp = source.point(0.050)
    extracted_center = source.point(source.inside_m + 0.060)
    assert np.linalg.norm(pregrasp - np.array(source.center_m)) == pytest.approx(0.050)
    assert np.linalg.norm(extracted_center - np.array(source.center_m)) == pytest.approx(0.105)


def test_complete_simulation_route_stays_level_and_places_center() -> None:
    mod = IKBridge().mod
    assert mod is not None
    # The saved working route, before the fixture position changes.
    source = Station.from_dict({"center_m": [0.234052398, -0.234052398, 0.2400],
                                "outward_xy": [-0.70710678, 0.70710678],
                                "inside_m": 0.045})
    destination = Station.from_dict({**SIMULATION_SCENE["destination"],
                                     "center_m": [0.060, -0.470, 0.0955]})
    home = [88.5, 69.0, 56.475, 152.0, 116.25, 41.4]
    steps = plan_transfer(mod, home, home, source, destination,
                          outside_m=0.060, camera_approach_m=0.145)
    assert len(steps) <= 35
    assert [step.event for step in steps if step.event] == ["attach", "detach"]
    assert steps[0].stage == "pregrasp"
    assert steps[-1].stage == "home"
    assert steps[0].servo_deg[4] == pytest.approx(26.25, abs=0.01)
    assert [step.servo_deg[5] for step in steps if step.stage in ("open", "grip", "release")] == [16.0, 134.5, 11.5]
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
    assert any(step.stage == "carry_side" for step in steps)
    placed = [step for step in steps if step.stage == "insert"][-1]
    assert placed.dish_center_m is not None
    assert np.linalg.norm(np.array(placed.dish_center_m) - destination.center_m) < 0.002
    above = [step for step in steps if step.stage == "approach_camera"][-1]
    assert above.dish_center_m is not None
    outside = destination.point(0.145)
    assert np.linalg.norm(np.array(above.dish_center_m[:2]) - outside[:2]) < 0.002
    assert above.dish_center_m[2] > 0.185
    lowering = [step for step in steps if step.stage == "lower_camera"]
    assert lowering
    assert all(np.linalg.norm(np.array(step.dish_center_m[:2]) - outside[:2]) < 0.002
               for step in lowering if step.dish_center_m is not None)
    assert lowering[-1].dish_center_m[2] == pytest.approx(destination.center_m[2], abs=0.002)
    insertion = [step for step in steps if step.stage == "insert"]
    assert insertion
    assert all(step.dish_center_m[2] == pytest.approx(destination.center_m[2], abs=0.002)
               for step in insertion if step.dish_center_m is not None)
    assert np.linalg.norm(np.array(insertion[-1].dish_center_m) - destination.center_m) < 0.002
    released = [step for step in steps if step.stage == "release"][-1]
    retreat = [step for step in steps if step.stage == "retreat"]
    assert retreat
    assert all(step.tcp_m[2] == pytest.approx(released.tcp_m[2], abs=0.002)
               for step in retreat)
    assert retreat[-1].tcp_m[1] > outside[1]
    clear = [step for step in steps if step.stage == "clear_camera"]
    assert len(clear) == 1
    assert clear[0].tcp_m[1] > retreat[-1].tcp_m[1]
    assert clear[0].tcp_m[2] < 0.160
    for step in steps[:-1]:
        transform, _ = mod.fk(mod.to_q(step.servo_deg[:5]), mod.TCP_ESTIMATE)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        tilt = math.degrees(math.acos(float(np.clip(normal[2], -1, 1))))
        assert tilt < 1.5


def test_unreachable_fixture_is_rejected() -> None:
    mod = IKBridge().mod
    assert mod is not None
    source = Station.from_dict({"center_m": [0.7, -0.7, 0.2], "outward_xy": [-1, 1]})
    destination = Station.from_dict(SIMULATION_SCENE["destination"])
    home = [88.5, 36.0, 56.5, 151.0, 116.25, 41.5]
    with pytest.raises(ValueError, match="no level IK"):
        plan_transfer(mod, home, home, source, destination)


def test_current_layout_uses_camera_roof_opening() -> None:
    mod = IKBridge().mod
    assert mod is not None
    assert SIMULATION_SCENE["destination"]["center_m"] == [0.060, -0.360, 0.0955]
    home = [88.5, 69.0, 56.475, 152.0, 116.25, 41.4]
    destination = Station.from_dict(SIMULATION_SCENE["destination"])
    steps = plan_transfer(mod, home, home,
                          Station.from_dict(SIMULATION_SCENE["source"]), destination)
    assert len(steps) <= 30
    assert [step.event for step in steps if step.event] == ["attach", "detach"]
    assert [step.servo_deg[5] for step in steps if step.stage in ("open", "grip", "release")] == [16.0, 134.5, 11.5]
    opening = destination.point(0.035)
    lower = [step for step in steps if step.stage == "lower_camera"]
    assert lower
    assert all(np.linalg.norm(np.array(step.dish_center_m[:2]) - opening[:2]) < 0.003
               for step in lower)
    assert lower[-1].dish_center_m[2] == pytest.approx(opening[2], abs=0.003)
    placed = [step for step in steps if step.stage == "insert"][-1]
    assert np.linalg.norm(np.array(placed.dish_center_m) - destination.center_m) < 0.003
    released = [step for step in steps if step.stage == "release"][-1]
    retreat = [step for step in steps if step.stage == "retreat"][-1]
    assert retreat.tcp_m[2] == pytest.approx(released.tcp_m[2], abs=0.003)
    assert retreat.tcp_m[1] > released.tcp_m[1]
    clear = [step for step in steps if step.stage == "clear_camera"]
    assert len(clear) == 1
    assert clear[0].tcp_m[1] > retreat.tcp_m[1] + 0.045
    assert clear[0].tcp_m[2] < 0.160
    assert steps[-1].stage == "home"
    assert not any(step.stage == "lift_clear_camera" for step in steps)
    # Check the actual joint interpolation, including the sparse carry turn.
    # The carousel divider reaches about 100 mm from its center and the dish
    # radius is 45 mm. The redesigned roof leaves a front U opening.
    carousel_xy = np.array([0.307591398, -0.100])
    for first, second in zip(steps, steps[1:]):
        if second.stage not in ("carry_side", "carry_camera", "approach_camera", "lower_camera"):
            continue
        a = np.array(first.servo_deg[:5]); b = np.array(second.servo_deg[:5])
        count = max(2, math.ceil(float(np.max(np.abs(b - a))) / 1.0))
        for fraction in np.linspace(0, 1, count + 1):
            transform, _ = mod.fk(mod.to_q(a + (b - a) * fraction), mod.TCP_ESTIMATE)
            center = transform[:3, 3] + transform[:3, :3] @ mod.TOOL_AXIS_LOCAL * 0.020
            if second.stage in ("carry_side", "carry_camera"):
                assert np.linalg.norm(center[:2] - carousel_xy) >= 0.150
            if 0.170 <= center[2] <= 0.195:
                assert 0.050 <= center[0] <= 0.070
                assert center[1] >= -0.345
    for first, second in zip(steps, steps[1:]):
        if second.stage not in ("clear_camera", "home"):
            continue
        a = np.array(first.servo_deg[:5]); b = np.array(second.servo_deg[:5])
        count = max(2, math.ceil(float(np.max(np.abs(b - a))) / 1.0))
        for fraction in np.linspace(0, 1, count + 1):
            transform, _ = mod.fk(mod.to_q(a + fraction * (b - a)),
                                   mod.TCP_ESTIMATE)
            tcp = transform[:3, 3]
            if tcp[1] < -0.280:
                assert tcp[2] >= 0.095 - 0.003
            if second.stage == "clear_camera":
                assert tcp[2] < 0.160


def test_pickup_moves_away_from_nearest_vertical_plate() -> None:
    mod = IKBridge().mod
    assert mod is not None
    source = Station.from_dict(SIMULATION_SCENE["source"])
    home = [88.5, 69.0, 56.475, 152.0, 116.25, 41.4]
    steps = plan_transfer(mod, home, home, source,
                          Station.from_dict(SIMULATION_SCENE["destination"]))
    mesh_path = (Path(__file__).resolve().parents[2] / "Simulation" /
                 "carousel_description/meshes/carousal_stand.stl")
    data = mesh_path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    triangles = np.array([
        [struct.unpack_from("<fff", data, 84 + index * 50 + 12 + vertex * 12)
         for vertex in range(3)] for index in range(triangle_count)
    ]) * 0.001
    # holder_4 is the divider beside the occupied slot. Apply the same mesh,
    # joint, and carousel rotations as the URDF, then slice it at dish height.
    mesh_rotation = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    def yaw(angle):
        return np.array([[math.cos(angle), -math.sin(angle), 0],
                         [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    triangles = ((triangles @ mesh_rotation.T) @ yaw(math.pi / 2).T
                 + [-0.070, 0, 0.200]) @ yaw(math.pi / 12).T
    triangles += [0.307591398, -0.100, 0]

    def plate_distance(center):
        nearest = math.inf
        for triangle in triangles:
            intersections = []
            for first, second in ((0, 1), (1, 2), (2, 0)):
                height = triangle[second, 2] - triangle[first, 2]
                if abs(height) < 1e-9:
                    continue
                fraction = (center[2] - triangle[first, 2]) / height
                if 0 <= fraction <= 1:
                    intersections.append(triangle[first, :2] + fraction *
                                         (triangle[second, :2] - triangle[first, :2]))
            if len(intersections) < 2:
                continue
            start, end = intersections[:2]
            direction = end - start
            fraction = float(np.clip(np.dot(center[:2] - start, direction) /
                                     (np.dot(direction, direction) + 1e-20), 0, 1))
            nearest = min(nearest, float(np.linalg.norm(
                center[:2] - start - fraction * direction)))
        return nearest

    grip = next(step for step in steps if step.stage == "grip")
    grip_transform, _ = mod.fk(mod.to_q(grip.servo_deg[:5]), mod.TCP_ESTIMATE)
    grip_estimate = (grip_transform[:3, 3] + grip_transform[:3, :3] @
                     mod.TOOL_AXIS_LOCAL * 0.020)
    attachment_offset = np.array(source.center_m) - grip_estimate
    first_distance = plate_distance(np.array(source.center_m))
    assert 0.045 < first_distance < 0.050
    pickup_distances = []
    for first, second in zip(steps, steps[1:]):
        if second.stage not in ("pickup_lift", "extract"):
            continue
        a = np.array(first.servo_deg[:5]); b = np.array(second.servo_deg[:5])
        count = max(2, math.ceil(float(np.max(np.abs(b - a))) / 1.0))
        for fraction in np.linspace(0, 1, count + 1):
            transform, _ = mod.fk(mod.to_q(a + fraction * (b - a)),
                                   mod.TCP_ESTIMATE)
            center = (transform[:3, 3] + transform[:3, :3] @
                      mod.TOOL_AXIS_LOCAL * 0.020 + attachment_offset)
            pickup_distances.append(plate_distance(center))
    assert min(pickup_distances) >= first_distance - 0.0005
    assert pickup_distances[-1] > 0.090
    assert plate_distance(np.array([*steps[4].dish_center_m])) > 0.050


def test_complete_carousel_sweep_clears_all_six_vertical_plates() -> None:
    mod = IKBridge().mod
    assert mod is not None
    home = [88.5, 69.0, 56.475, 152.0, 116.25, 41.4]
    steps = plan_transfer(mod, home, home,
                          Station.from_dict(SIMULATION_SCENE["source"]),
                          Station.from_dict(SIMULATION_SCENE["destination"]))
    mesh_path = (Path(__file__).resolve().parents[2] / "Simulation" /
                 "carousel_description/meshes/carousal_stand.stl")
    data = mesh_path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    triangles = np.array([
        [struct.unpack_from("<fff", data, 84 + i * 50 + 12 + j * 12)
         for j in range(3)] for i in range(count)
    ]) * 0.001
    mesh_rotation = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    def yaw(angle):
        return np.array([[math.cos(angle), -math.sin(angle), 0],
                         [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    holders = ((.070, 0, -math.pi / 2), (.035, .0606218, -math.pi / 6),
               (-.035, .0606218, math.pi / 6), (-.070, 0, math.pi / 2),
               (-.035, -.0606218, 5 * math.pi / 6),
               (.035, -.0606218, -5 * math.pi / 6))
    world_triangles = np.concatenate([
        ((triangles @ mesh_rotation.T) @ yaw(angle).T + [x, y, .200])
        @ yaw(math.pi / 12).T + [.307591398, -.100, 0]
        for x, y, angle in holders
    ])

    def nearest_plate(center, height):
        # Slice every collision triangle through the top, middle, and bottom
        # of the 15 mm dish. Distances are vectorized over all six meshes.
        edge_points = []
        edge_masks = []
        for first, second in ((0, 1), (1, 2), (2, 0)):
            a = world_triangles[:, first, :]
            b = world_triangles[:, second, :]
            vertical = b[:, 2] - a[:, 2]
            valid = np.abs(vertical) > 1e-9
            fraction = np.zeros(len(vertical))
            fraction[valid] = (height - a[valid, 2]) / vertical[valid]
            valid &= (fraction >= 0) & (fraction <= 1)
            edge_points.append(a[:, :2] + fraction[:, None] * (b[:, :2] - a[:, :2]))
            edge_masks.append(valid)
        intersections = np.stack(edge_points, axis=1)
        masks = np.stack(edge_masks, axis=1)
        has_segment = np.sum(masks, axis=1) >= 2
        selected = np.where(masks[:, :, None], intersections, np.nan)
        selected = selected[has_segment]
        first = selected[np.arange(len(selected)), np.argmax(~np.isnan(selected[:, :, 0]), axis=1)]
        last = selected[np.arange(len(selected)), 2 - np.argmax(~np.isnan(selected[:, ::-1, 0]), axis=1)]
        direction = last - first
        fraction = np.clip(np.sum((center[:2] - first) * direction, axis=1) /
                           (np.sum(direction * direction, axis=1) + 1e-20), 0, 1)
        return float(np.min(np.linalg.norm(center[:2] - first -
                                           fraction[:, None] * direction, axis=1)))

    grip = next(step for step in steps if step.stage == "grip")
    grip_transform, _ = mod.fk(mod.to_q(grip.servo_deg[:5]), mod.TCP_ESTIMATE)
    grip_estimate = (grip_transform[:3, 3] + grip_transform[:3, :3] @
                     mod.TOOL_AXIS_LOCAL * .020)
    attachment_offset = np.array(SIMULATION_SCENE["source"]["center_m"]) - grip_estimate
    extracted = [step for step in steps if step.stage == "extract"][-1]
    extracted_center = np.array(extracted.dish_center_m) + attachment_offset
    extracted_gap = min(nearest_plate(extracted_center, extracted_center[2] + vertical)
                        for vertical in (-.009, 0, .009)) - .045
    assert extracted_gap >= .040  # whole dish is outside before the turn
    min_after_extraction = math.inf
    for first, second in zip(steps, steps[1:]):
        if second.stage not in ("pickup_lift", "extract", "lift",
                                "carry_side", "carry_camera"):
            continue
        a = np.array(first.servo_deg[:5]); b = np.array(second.servo_deg[:5])
        count = max(2, math.ceil(float(np.max(np.abs(b - a))) / .5))
        for fraction in np.linspace(0, 1, count + 1):
            transform, _ = mod.fk(mod.to_q(a + fraction * (b - a)),
                                   mod.TCP_ESTIMATE)
            center = (transform[:3, 3] + transform[:3, :3] @
                      mod.TOOL_AXIS_LOCAL * .020 + attachment_offset)
            gap = min(nearest_plate(center, center[2] + vertical)
                      for vertical in (-.009, 0, .009)) - .045
            assert gap >= 0, (second.stage, center, gap)
            if second.stage in ("extract", "lift", "carry_side", "carry_camera"):
                min_after_extraction = min(min_after_extraction, gap)
    assert min_after_extraction >= .005


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
