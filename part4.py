import numpy as np
import numpy.linalg as nla

import matplotlib.pyplot as plt

from utils import trajectory_sequence

"""
we shall combine trajectory generation , kin simulation, and feedforward
"""
# {b}: car body frame
# {c}: cube frame

def full_function(Tsc_init, Tsc_final, Tsb_initial):
    # Tse Modern Robotics Wiki Page. initial EE config
    Tse_init = np.array([[0,0,1,0],
                      [0,1,0,0],
                      [-1,0,0,0.5],
                      [0,0,0,1]])
    
    # Chasis frame config Tsc
    Tsc_init = np.array([[1,0,0,1.25],
                         [0,1,0,0],
                           [0,0,1,0.025],
                           [0,0,0,1]])

    Tsc_final = np.array([[0,1,0,0],
                      [-1,0,0,-1.5],
                      [0,0,1,0.025],
                      [0,0,0,1]])


    # control theory stuff
    errors = np.array([0,0,0,0,0,0])

    # Set Tse from Joint Angles
    # chassis_config = np.array((0.0,0.0,0.0))
    # theta_arm = np.array((0.0,-0.5,-0.5,-0.5,0.0))
    # Tse_init = fc.calc_Tse(chassis_config,theta_arm)

    # Define Time for each trajectory segment
    max_velocity = 0.25 # m/s
    joint_speed_limit = 15
    k = 1 # number of trajectory reference configurations per 0.01 seconds


    traj_list_complete, gripper_state_complete = trajectory_sequence(Tse_init, Tsc_init, Tsc_final, max_velocity,  k)

