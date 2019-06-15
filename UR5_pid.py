import logging
import time
from RTIF.API import API

# this script demonstrate how to control the robot arm to a specified location
# by writing the angle of each joint.
#
# Notice: the default initial position is a singular position which we can only
# control the joint angle but cannot set tool location (the Invert Kines solver
# will report error)

def main(ROBOT_HOST = "192.168.1.104"):

    logging.getLogger().setLevel(logging.INFO)
    api = API(ROBOT_HOST)

    target=[]#the target position of end effector x,y,z,rx,ry,rz
    qnear=[]
    target_j=api.get_inv_kin(target,qnear)
    current_j=api.GetCurrentJointRad
    kp=3
    kv=0.5
    lamb=kp/kv
    error=current_j-target_j
    sat=vmax/(lamb*np.abs(error))
    scale=np.ones(6)
    if np.any(sat<1):
        index=np.argmin(sat)
        unclipped=kp*error[index]
        clipped=kv*vmax*np.sign(error[index])
        scale=np.ones(6*clipped/unclipped)
        scale[index]=1
    qv=-kv*(dx+np.clip(sat/scale,0,1)*scale[index]*lamb*error)
    api.SpeedJointRad(qv,a,t)

if __name__ == "__main__":
    main()
