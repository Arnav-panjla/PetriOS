#!/usr/bin/env python3
"""PetriOS 5-axis arm IK, targeting midpoint of closed gripper fingertips.

Based on PetriOS robot_arm_final_urdf_v4.urdf. The TCP offset is estimated
from the two finger STL meshes at URDF finger angle zero; verify that this
represents the REAL CLOSED GRIPPER before actuating hardware.

Examples:
  python arm_ik_closed_gripper.py --reference
  python arm_ik_closed_gripper.py --x 0.02 --y -0.20 --z 0.20
  python arm_ik_closed_gripper.py --reference --tcp -0.01 -0.12 0

--x/y/z and --tcp are METRES in URDF base_link and link_5 frames respectively.
Only prints results. Does not move hardware.

Requires: numpy scipy trimesh
"""
from pathlib import Path
import argparse
import math
import numpy as np
from scipy.optimize import least_squares

REF_SERVO = np.array([90., 39., 58., 66.5, 116.25])
GRIPPER_SERVO = 162.
# IMPORTANT: +1 is a placeholder. Calibrate servo sign for each physical joint.
SERVO_SIGNS = np.ones(5)
PULSE_LIMITS = [(500., 2900.), (500., 2700.), (500., 2900.),
                (500., 2900.), (500., 2900.), (1100., 1600.)]
JOINT_ORIGINS = np.array([
    [0., -.029745, .071600],
    [.008600, -.010000, .019250],
    [0., 0., .110490],
    [0., -.130491, 0.],
    [.010001, -.067246, -.000001],
])
JOINT_AXES = np.array([
    [0., 0., 1.], [1., 0., 0.], [1., 0., 0.],
    [1., 0., 0.], [0., 1., 0.],
])
# These are <visual><origin> positions from the URDF, in finger-link frames.
FINGER_VISUAL_ORIGINS = np.array([
    [-.002620, .292298, -.199839],
    [-.034280, .292093, -.202839],
])
# These are gripper hinge positions in link_5 frame.
FINGER_JOINT_ORIGINS = np.array([
    [-.005981, -.068816, -.001500],
    [ .025679, -.068611,  .001500],
])

def rotation(axis, theta):
    x, y, z = axis / np.linalg.norm(axis)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
        [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)],
    ])

def stl_estimated_tcp(mesh_dir):
    """Estimate midpoint of frontmost finger vertices, URDF finger q=0.

    This is a geometric EXTREMITY estimate, not verified closed contact.
    In URDF, fingers rotate about local Z and extend towards local -Y.
    """
    try:
        import trimesh
    except ImportError as e:
        raise RuntimeError('Install trimesh: pip install trimesh') from e
    mesh_dir = Path(mesh_dir)
    tips = []
    for i, side in enumerate(('left', 'right')):
        mesh_file = mesh_dir / f'{side}_finger.stl'
        if not mesh_file.is_file():
            raise FileNotFoundError(f'Missing {mesh_file}. Provide --meshes DIR or --tcp X Y Z.')
        mesh = trimesh.load_mesh(mesh_file, process=False)
        points = np.asarray(mesh.vertices, dtype=float) * 0.001
        # Per URDF: parent(link_5) -> joint origin -> finger link frame;
        # then finger visual origin, with finger q=0.
        points = points + FINGER_VISUAL_ORIGINS[i] + FINGER_JOINT_ORIGINS[i]
        # Average the thin frontmost 1.5 mm of tip geometry rather than one vertex.
        front = np.min(points[:, 1])
        tip_points = points[points[:, 1] <= front + .0015]
        tips.append(np.mean(tip_points, axis=0))
    return .5 * (tips[0] + tips[1]), tips

def forward(q, tcp):
    T = np.eye(4)
    for origin, axis, angle in zip(JOINT_ORIGINS, JOINT_AXES, q):
        H = np.eye(4)
        H[:3, :3] = rotation(axis, angle)
        H[:3, 3] = origin
        T = T @ H
    H = np.eye(4)
    H[:3, 3] = tcp
    return T @ H

def servo_angles(q):
    return REF_SERVO + SERVO_SIGNS * np.rad2deg(q)

def servo_pulse(i, degree):
    if degree < -1e-6 or degree > 180.000001:
        raise ValueError(f'J{i} exceeds 0..180 deg: {degree:.2f}')
    lo, hi = PULSE_LIMITS[i]
    return round(lo + (hi-lo) * degree/180.)

def bounds():
    a = np.deg2rad((0.-REF_SERVO)*SERVO_SIGNS)
    b = np.deg2rad((180.-REF_SERVO)*SERVO_SIGNS)
    return np.minimum(a,b), np.maximum(a,b)

def solve(target, tcp):
    lo, hi = bounds()
    def residual(q):
        error = forward(q, tcp)[:3,3] - target
        return np.concatenate([error/.10, q*.001])
    seeds = [np.zeros(5), .5*(lo+hi)]
    for i in (1,2,3):
        for delta in (-.6,.6):
            s = np.zeros(5)
            s[i] = np.clip(delta, lo[i]+1e-6, hi[i]-1e-6)
            seeds.append(s)
    sols = [least_squares(residual, np.clip(seed, lo+1e-7, hi-1e-7),
                          bounds=(lo,hi), max_nfev=1500) for seed in seeds]
    best = min(sols, key=lambda sol: np.linalg.norm(forward(sol.x,tcp)[:3,3]-target))
    T = forward(best.x, tcp)
    err = np.linalg.norm(T[:3,3]-target)
    if err > .003:
        raise ValueError(f'No solution within 3 mm; closest error = {err*1000:.1f} mm')
    return best.x, T, err

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reference', action='store_true')
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--z', type=float)
    ap.add_argument('--tcp', nargs=3, type=float, metavar=('X','Y','Z'),
                    help='Measured closed fingertip midpoint in link_5 frame (m)')
    default_meshes = Path(__file__).resolve().parent.parent / 'simul files/robot_arm_description/meshes'
    ap.add_argument('--meshes', type=Path, default=default_meshes)
    args = ap.parse_args()
    if args.tcp is not None:
        tcp = np.array(args.tcp)
        print('Using supplied closed-gripper TCP [m]:', np.round(tcp,6))
    else:
        try:
            tcp, tips = stl_estimated_tcp(args.meshes)
        except (OSError, FileNotFoundError, ImportError, RuntimeError) as e:
            ap.error(f'Cannot estimate TCP from mesh: {e}\n'
                     'Use --tcp X Y Z once the fingertip offset is measured.')
        print('ESTIMATED TCP midpoint in link_5 frame [m]:', np.round(tcp,6))
        print('Estimated left/right extrema [m]:', [np.round(p,6).tolist() for p in tips])
        print('WARNING: These extrema assume URDF finger angle = 0 means closed.')
        print('Verify this assumption and actual fingertip contact point.')
    ref = forward(np.zeros(5), tcp)
    if args.reference:
        print('Reference servo J0..J5 [deg]:', [*REF_SERVO, GRIPPER_SERVO])
        print('Reference pulses [us]:', [servo_pulse(i,a) for i,a in enumerate([*REF_SERVO,GRIPPER_SERVO])])
        print('Reference fingertip midpoint [m]:', np.round(ref[:3,3],6))
        print('Reference link_5 orientation [matrix]:\n', np.round(ref[:3,:3],4))
        return
    if args.x is None or args.y is None or args.z is None:
        ap.error('Set --x --y --z (metres), or use --reference')
    target = np.array([args.x,args.y,args.z])
    try:
        q,T,err = solve(target,tcp)
        degrees = servo_angles(q)
        pulses = [servo_pulse(i, angle) for i,angle in enumerate(degrees)]
    except ValueError as e:
        ap.error(str(e))
    print('Target closed fingertip midpoint [m]:', target)
    print('URDF q [deg]:', np.round(np.rad2deg(q),3))
    print('Servo J0-J4 [deg]:', np.round(degrees,3))
    print('Servo J0-J4 [us]:', pulses)
    print(f'Gripper J5 unchanged: {GRIPPER_SERVO} deg / {servo_pulse(5,GRIPPER_SERVO)} us')
    print('Achieved fingertip midpoint [m]:', np.round(T[:3,3],6))
    print(f'Position error: {err*1000:.3f} mm')
    print('CALIBRATION UNVERIFIED — DO NOT DRIVE HARDWARE DIRECTLY.')

if __name__ == '__main__':
    main()
