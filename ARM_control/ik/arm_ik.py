"""PetriOS URDF-based IK; prints safe suggestions, NEVER drives servos.
Requires: pip install numpy scipy
Example: python arm_ik.py --reference
         python arm_ik.py --x 0.01 --y -0.20 --z 0.20
Coordinates in metres, relative to URDF base_link.
"""
import argparse
import numpy as np
from scipy.optimize import least_squares

# Photo-reference angles supplied by user, J0..J4, in existing CLI degrees.
REFERENCE_DEG = np.array([90., 39., 58., 66.5, 116.25])
GRIPPER_DEG = 162.  # J5 is NOT in arm IK.
# UNVERIFIED: change signs after individual joint motion tests.
SIGNS = np.array([1., 1., 1., 1., 1.])
# Exact URDF origins, joint_1 through joint_5 (m), in parent coordinates.
ORIGINS = np.array([
    [0., -.029745, .071600],
    [.008600, -.010000, .019250],
    [0., 0., .110490],
    [0., -.130491, 0.],
    [.010001, -.067246, -.000001]
])
AXES = np.array([[0,0,1], [1,0,0], [1,0,0], [1,0,0], [0,1,0]])
PULSE_LIMITS = [(500,2900), (500,2700), (500,2900),
                (500,2900), (500,2900), (1100,1600)]
# TCP defaults to link_5 origin, NOT gripper tip. Measure tip offset to update.
TCP_OFFSET = np.array([0., 0., 0.])
TOOL_AXIS = np.array([0., -1., 0.])

def rotation(axis, q):
    axis = np.asarray(axis, dtype=float)
    x,y,z = axis
    c,s = np.cos(q),np.sin(q)
    return np.array([
        [c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
        [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)]
    ])

def fk(q):
    """T_base_to_tcp = product(translation(origin_i) * rotation(axis_i,q_i))."""
    T=np.eye(4)
    for origin,axis,angle in zip(ORIGINS,AXES,q):
        H=np.eye(4)
        H[:3,3]=origin
        H[:3,:3]=rotation(axis,angle)
        T=T@H
    H=np.eye(4)
    H[:3,3]=TCP_OFFSET
    return T@H

def servo_to_q(servo):
    return np.deg2rad((np.asarray(servo)-REFERENCE_DEG)*SIGNS)

def q_to_servo(q):
    return REFERENCE_DEG+SIGNS*np.rad2deg(q)

def pulse(joint,deg):
    if not 0 <= deg <= 180:
        raise ValueError(f'Joint {joint} outside servo 0..180 degrees: {deg}')
    lo,hi=PULSE_LIMITS[joint]
    return round(lo+(hi-lo)*deg/180)

def ik(target_xyz, direction=None, seed=None):
    """3-D position IK, optional 2-DOF pointing direction, using bounded LS.
    The arm has 5 DOF, so arbitrary 6D poses are not always achievable.
    """
    target=np.asarray(target_xyz,dtype=float)
    if target.shape!=(3,):
        raise ValueError('Target must be [x,y,z] in metres')
    if direction is not None:
        direction=np.asarray(direction,dtype=float)
        if direction.shape!=(3,) or np.linalg.norm(direction)<1e-9:
            raise ValueError('Direction must be a nonzero 3-vector')
        direction=direction/np.linalg.norm(direction)
    seed=REFERENCE_DEG if seed is None else np.asarray(seed,dtype=float)
    qseed=servo_to_q(seed)
    low=np.deg2rad((0.-REFERENCE_DEG)*SIGNS)
    high=np.deg2rad((180.-REFERENCE_DEG)*SIGNS)
    lb,ub=np.minimum(low,high),np.maximum(low,high)
    qseed=np.clip(qseed,lb+1e-8,ub-1e-8)
    def residual(q):
        T=fk(q)
        parts=[(T[:3,3]-target)/.1]
        if direction is not None:
            parts.append(.75*(T[:3,:3]@TOOL_AXIS-direction))
        parts.append(.002*(q-qseed))
        return np.concatenate(parts)
    starts=[qseed,(lb+ub)/2]
    for j in [1,2,3]:
        for delta in [-.6,.6]:
            s=qseed.copy()
            s[j]=np.clip(s[j]+delta,lb[j]+1e-8,ub[j]-1e-8)
            starts.append(s)
    results=[least_squares(residual,s,bounds=(lb,ub),max_nfev=1500) for s in starts]
    result=min(results,key=lambda r:np.linalg.norm(residual(r.x)))
    T=fk(result.x)
    error=np.linalg.norm(T[:3,3]-target)
    dir_error=None
    if direction is not None:
        dir_error=np.rad2deg(np.arccos(np.clip((T[:3,:3]@TOOL_AXIS)@direction,-1,1)))
    if error>.003 or (dir_error is not None and dir_error>3):
        raise ValueError(f'No sufficiently accurate solution: position error={error*1000:.1f} mm; direction error={dir_error}')
    servos=q_to_servo(result.x)
    pulses=[pulse(i,a) for i,a in enumerate(servos)]
    return result.x,servos,pulses,T,error,dir_error

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--reference',action='store_true')
    for dim in 'xyz':ap.add_argument('--'+dim,type=float)
    ap.add_argument('--direction',nargs=3,type=float)
    args=ap.parse_args()
    if args.reference:
        print('Reference pose ASSUMED to be URDF q=[0,0,0,0,0]')
        print('Photo servo degrees:',REFERENCE_DEG)
        print('Photo arm pulses (us):',[pulse(i,a) for i,a in enumerate(REFERENCE_DEG)])
        print('Gripper pulse (us):',pulse(5,GRIPPER_DEG))
        print('URDF link_5 origin [m]:',fk(np.zeros(5))[:3,3])
        return
    if any(getattr(args,d) is None for d in 'xyz'):
        ap.error('provide x,y,z or --reference')
    try:q,servo,pulses,T,err,derr=ik([args.x,args.y,args.z],args.direction)
    except ValueError as exc:ap.error(str(exc))
    print('URDF q (degrees):',np.round(np.rad2deg(q),3))
    print('Arm servo degrees J0-J4:',np.round(servo,3))
    print('Arm pulses (us) J0-J4:',pulses)
    print('Gripper J5 remains:',GRIPPER_DEG,'deg /',pulse(5,GRIPPER_DEG),'us')
    print('Achieved TCP [m]:',np.round(T[:3,3],5),'error mm:',round(err*1000,3))
    if derr is not None:print('Direction error degrees:',round(derr,3))
    print('CALIBRATION UNVERIFIED: do not use these pulses to drive hardware yet.')

if __name__=='__main__':main()
