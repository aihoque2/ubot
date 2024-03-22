import numpy as np
import numpy.linalg as nla

"""
we shall combine trajectory generation , kin simulation, and feedforward
"""

def full_function():
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



    errors = np.array([0,0,0,0,0,0])

    # Set Tse from Joint Angles
    # chassis_config = np.array((0.0,0.0,0.0))
    # theta_arm = np.array((0.0,-0.5,-0.5,-0.5,0.0))
    # Tse_init = fc.calc_Tse(chassis_config,theta_arm)

    # Define Time for each trajectory segment
    max_velocity = 0.25 # m/s
    joint_speed_limit = 15
    k = 1 # number of trajectory reference configurations per 0.01 seconds


    traj_list_complete, gripper_state_complete = tg.gen_full_trajectory(Tsc_init, Tsc_final, Tse_init, k, max_velocity, write_test_traj = False)