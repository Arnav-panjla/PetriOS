#!/usr/bin/env python3
"""PetriOS physical arm IK v2: exact URDF joint origins, two calibration poses.

Outputs commanded servo angles and pulse widths; NEVER moves hardware.

Coordinates: metres, relative to the URDF base_link, NOT the table.
TCP: estimated closed-fingertip midpoint, from previous mesh analysis.

Examples:
  python arm_ik_v2.py --reference
  python arm_ik_v2.py --dz 0.02 --hold-direction
  python arm_ik_v2.py --dx 0 --dy 0.02 --dz 0 --hold-direction
  python arm_ik_v2.py --x 0.02 --y -0.20 --z 0.15
  python arm_ik_v2.py --reference --tcp 0.010 -0.120 0.0015

--dz is a change in world Z relative to your NEW folded pose;
--x/--y/--z is an absolute target position in URDF base_link.
Calibrate signs, base frame and TCP before using output on hardware.
"""
import argparse
import math
import numpy as np
from scipy.optimize import least_squares

# Observed physical servo angles (DEGREES) in the earlier horizontal pose.
# We tentatively associate this pose with URDF zero joint angles.
URDF_ZERO_SERVO = np.array([90., 39., 58., 66.5, 116.25])
# NEW folded pose supplied by user; starting point of numerical IK.
START_SERVO = np.array([90.5, 35.5, 58., 151.5, 116.])
GRIPPER_SERVO = 162.  # separate J5 motor; closed state not yet verified

# Provisional physical servo rotation signs relative to URDF joint axes.
# J0 + (CCW top); J1 + (down); J2 - (up); J3 + (down), J4 - (CCW from gripper).
# SIGN ASSUMPTIONS MUST STILL BE VERIFIED with measured TCP displacements.
SERVO_SIGN = np.array([1., 1., -1., 1., -1.])

# Full configured electrical PWM interval at nominal 0..180 deg.
PULSE_LIMITS_US = [(500., 2900.), (500., 2700.), (500., 2900.),
                   (500., 2900.), (500., 2900.), (1100., 1600.)]
# Temporary working limits in degrees to keep computations off hard endpoints.
# Replace with measured SAFE mechanical ranges; these are NOT guarantees.
SOFT_LIMIT_DEG = np.array([
    [5., 175.], [5., 165.], [5., 175.],
    [5., 175.], [5., 175.]
])

# URDF joint origins, in parent frames, metres; axes in joint-local frames.
JOINT_ORIGINS = np.array([
    [0., -.029745, .071600],
    [.008600, -.010000, .019250],
    [0., 0., .110490],
    [0., -.130491, 0.],
    [.010001, -.067246, -.000001],
])
JOINT_AXES = np.array([[0,0,1], [1,0,0], [1,0,0], [1,0,0], [0,1,0]], dtype=float)

# Previous estimate from the finger meshes at URDF closed-gripper configuration.
# This is in link_5 coordinates, not in the world frame. Adjustable --tcp.
TCP_ESTIMATE = np.array([.010023, -.125002, .0015])
# Desired tool axis (points outward from the wrist toward the fingertips).
TOOL_AXIS_LOCAL = np.array([0., -1., 0.])

def rotation(axis, radians):
    x, y, z = axis / np.linalg.norm(axis)
    c, s = np.cos(radians), np.sin(radians)
    return np.array([
        [c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
        [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)]
    ])

def to_q(servo_deg):
    return np.deg2rad((np.asarray(servo_deg)-URDF_ZERO_SERVO)*SERVO_SIGN)

def to_servo(q):
    return URDF_ZERO_SERVO + SERVO_SIGN*np.rad2deg(q)

def pwm(i, servo_deg):
    if not (0 <= servo_deg <= 180):
        raise ValueError(f'Joint {i}: invalid servo angle {servo_deg:.3f}')
    lo, hi = PULSE_LIMITS_US[i]
    return int(round(lo+(hi-lo)*servo_deg/180.))

def fk(q, tcp):
    T = np.eye(4)
    frames = [T.copy()]
    for xyz, axis, a in zip(JOINT_ORIGINS, JOINT_AXES, q):
        H = np.eye(4)
        H[:3,3] = xyz
        H[:3,:3] = rotation(axis, a)
        T = T @ H
        frames.append(T.copy())
    H = np.eye(4)
    H[:3,3] = tcp
    T = T @ H
    return T, frames

def limits_q():
    v1 = (SOFT_LIMIT_DEG[:,0]-URDF_ZERO_SERVO)*SERVO_SIGN
    v2 = (SOFT_LIMIT_DEG[:,1]-URDF_ZERO_SERVO)*SERVO_SIGN
    return np.deg2rad(np.minimum(v1, v2)), np.deg2rad(np.maximum(v1, v2))

def solve(target, tcp, start_servo, hold_direction=False):
    target = np.asarray(target,dtype=float)
    q0 = to_q(start_servo)
    T0,_ = fk(q0,tcp)
    axis_target = T0[:3,:3] @ TOOL_AXIS_LOCAL if hold_direction else None
    lo, hi = limits_q()
    q0 = np.clip(q0,lo+1e-7,hi-1e-7)

    def residual(q):
        T,_ = fk(q,tcp)
        out = [(T[:3,3]-target)/0.10]   # 10cm scaling
        if axis_target is not None:
            # Preserve tool approach direction (2 orientation DOF), not full roll.
            out.append(0.8*(T[:3,:3] @ TOOL_AXIS_LOCAL-axis_target))
        # Prefer minimal change from the NEW resting pose.
        out.append(0.003*(q-q0))
        return np.concatenate(out)

    starts = [q0]
    for j in (1,2,3):
        for d in (-0.4,0.4):
            q=q0.copy();q[j]=np.clip(q[j]+d,lo[j]+1e-7,hi[j]-1e-7)
            starts.append(q)
    sols=[]
    for seed in starts:
        res=least_squares(residual,seed,bounds=(lo,hi),max_nfev=1000,
                          ftol=1e-10,xtol=1e-10,gtol=1e-10)
        T,_=fk(res.x,tcp)
        poserr=float(np.linalg.norm(T[:3,3]-target))
        axiserr=0.
        if axis_target is not None:
            cosine=np.clip((T[:3,:3]@TOOL_AXIS_LOCAL)@axis_target,-1,1)
            axiserr=float(np.rad2deg(np.arccos(cosine)))
        sols.append((poserr,axiserr,np.linalg.norm(res.x-q0),res.x,T))
    # Prefer solutions satisfying both tolerances, closest to starting joints.
    valid=[s for s in sols if s[0]<.003 and (not hold_direction or s[1]<3.)]
    if not valid:
        best=min(sols,key=lambda s:s[0]+(s[1]/180 if hold_direction else 0))
        raise ValueError(f'No sufficiently accurate solution: closest '
                         f'position error {best[0]*1000:.1f} mm, '
                         f'direction error {best[1]:.2f} deg. '
                         'Try another reachable target or omit --hold-direction.')
    best=min(valid,key=lambda s:(s[2],s[0]))
    return best[3],best[4],best[0],best[1]

def print_pose(name,servo,tcp):
    q=to_q(servo)
    T,_=fk(q,tcp)
    print(f'{name} J0-J4 servo [deg]:',np.round(servo,3).tolist())
    print(f'{name} J0-J4 pulses [us]:',[pwm(i,x) for i,x in enumerate(servo)])
    print(f'{name} URDF q [deg]:',np.round(np.rad2deg(q),3).tolist())
    print(f'{name} estimated TCP position [m]:',np.round(T[:3,3],6).tolist())
    print(f'{name} tool direction:',np.round(T[:3,:3]@TOOL_AXIS_LOCAL,5).tolist())
    return T

def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reference',action='store_true')
    for v in ('x','y','z','dx','dy','dz'):
        ap.add_argument('--'+v,type=float)
    ap.add_argument('--hold-direction',action='store_true',help='Keep gripper pointing in original direction')
    ap.add_argument('--tcp',type=float,nargs=3,metavar=('X','Y','Z'))
    ap.add_argument('--start-servo',type=float,nargs=5,metavar=('J0','J1','J2','J3','J4'),help='Actual CURRENT servo angle commands (default new folded pose)')
    args=ap.parse_args()
    tcp=np.array(args.tcp if args.tcp is not None else TCP_ESTIMATE)
    start=np.array(args.start_servo if args.start_servo is not None else START_SERVO)
    if np.any(start<SOFT_LIMIT_DEG[:,0]) or np.any(start>SOFT_LIMIT_DEG[:,1]):
        ap.error('Starting servo pose is outside provisional soft limits')
    print('WARNING: Geometry, physical servo signs, TCP and table/base alignment not fully validated.')
    print('TCP offset [m] (link_5):',np.round(tcp,6).tolist())
    print_pose('OLD HORIZONTAL',URDF_ZERO_SERVO,tcp)
    T0=print_pose('NEW FOLDED',start,tcp)
    print('J5 gripper stays:',GRIPPER_SERVO,'deg =',pwm(5,GRIPPER_SERVO),'us')
    if args.reference:
        return
    abspos=[args.x,args.y,args.z]
    rel=[args.dx,args.dy,args.dz]
    if any(v is not None for v in abspos) and any(v is not None for v in rel):
        ap.error('Use EITHER --x/--y/--z OR --dx/--dy/--dz, not both')
    if all(v is not None for v in abspos):
        target=np.array(abspos)
    elif all(v is None for v in abspos) and any(v is not None for v in rel):
        target=T0[:3,3]+np.array([v if v is not None else 0. for v in rel])
    else:
        ap.error('Specify all --x --y --z, or a relative --dx/--dy/--dz, or --reference')
    try:
        q,T,err,axiserr=solve(target,tcp,start,args.hold_direction)
        servo=to_servo(q)
        pulses=[pwm(i,x) for i,x in enumerate(servo)]
    except ValueError as e:
        ap.error(str(e))
    print('\nTARGET TCP [m]:',np.round(target,6).tolist())
    print('Solved URDF q [deg]:',np.round(np.rad2deg(q),3).tolist())
    print('Solved J0-J4 servo [deg]:',np.round(servo,3).tolist())
    print('Change from new pose [deg]:',np.round(servo-start,3).tolist())
    print('Solved J0-J4 pulses [us]:',pulses)
    print('FK result TCP [m]:',np.round(T[:3,3],6).tolist())
    print(f'MODEL position error = {err*1000:.3f} mm')
    if args.hold_direction:
        print(f'MODEL tool direction error = {axiserr:.3f} deg')
    print('DRY RUN ONLY. Do NOT send commands to hardware before calibration / collision checks.')

if __name__=='__main__':
    main()
