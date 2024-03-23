"""
kin_sim.py
Kinematics simulation is litt

read MR. 132. and 13.4 and do MR 13.33 
"""

import os
import csv
import numpy as np
import numpy.linalg as nla


os.system("rm milestone2.csv") # enable re-testing

def NextState(state, vels, dt, max_vel):
    """
    like gym's step() function?

    no, like MR 13.4: Odometry

    :params:
    state: shape(,12) vector
    vels: shape(,9) vector

    :return:
    next_state: shape(,12) vector of next state
    """
    r = 0.0475
    l = 0.235
    w = 0.15
    chassis_state = state[0:3]
    arm_state = state[3:8]
    wheel_state = state[8:12]

    
    joint_vels = vels[0:5]
    wheel_vels = vels[5:9]
    joint_vels[(np.absolute(joint_vels) > max_vel)] = max_vel
    arm_new = arm_state + joint_vels*dt

    wheel_vels[(np.absolute(wheel_vels) > max_vel)] = max_vel
    d_theta = wheel_vels*dt # wheel vels
    wheel_new = wheel_state + d_theta


    ######################
    # BODY CHASSIS calc ##
    ######################
    # eqn 13.3
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                  [1,1,1, 1],
                  [-1,1,-1, 1]])
    

    # Chassis body twist
    V_b = F@d_theta.T 

    wbz = V_b[0]
    vbx = V_b[1]
    vby = V_b[2]

    phi = chassis_state[0]
    x = chassis_state[1]
    y = chassis_state[2]

    if wbz < 1e-6: # zero case but dealing wit floating points (michael heath)
        dqb = np.array([0,vbx,vby])
    else:
        dqb = np.array([wbz,
                       (vbx*np.sin(wbz)+vby*(np.cos(wbz)-1))/wbz,
                       (vby*np.sin(wbz)+vbx*(1-np.cos(wbz)))/wbz])

    # eqn 13.36
    dqs = np.array([[1,0,0],
                    [0,np.cos(phi),-1*np.sin(phi)],
                    [0,np.sin(phi),np.cos(phi)]])
    
    dq = dqs@dqb.T

    chassis_new = chassis_state + dq

    data_row = np.concatenate((chassis_new, arm_new, wheel_new), axis=None)
    return data_row


if __name__ == "__main__":
    ### our unit test
    filepath = os.path.dirname(os.path.abspath(__file__))
    filename = filepath + '//' + 'milestone2.csv'

    i = 0
    dt = 0.01
    length = 0.3
    max_vel = 12.5

    # initial state
    chassis_config = np.array((0.0,0.0,0.0))
    theta_arm = np.array((0.0,0.0,0.0,0.0,0.0))
    theta_wheels = np.array((0.0,0.0,0.0,0.0))
    theta_dot_arm = np.array((0.025,0.025,0.025,0.025,0.025))
    theta_dot_wheels = np.array((0.0125,0.0125,0.0125,0.0125))

    state = np.concatenate((chassis_config, theta_arm, theta_wheels), axis=None)
    vels = np.concatenate((theta_dot_arm,theta_dot_wheels), axis=None)

    t = 0.0
    with open(filename, 'a', newline='') as f:
        csvwriter = csv.writer(f,)
        csvwriter.writerow(state)
        while (t < 1.0):
            next_data = NextState(state, vels, dt, max_vel)
            csvwriter.writerow(next_data)
            state = next_data
            t+=dt
            i+=1

    print("here's i: ", i)





