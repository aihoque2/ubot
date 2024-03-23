"""
utils.py
"""
import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as Rot

import traj_gen as tg
import modern_robotics as mr

""""
generate the whole sequence
"""

def trajectory_sequence(Tse_init, Tsc_init, Tsc_final, max_vel, k, csv_name):
    # Setup initial, final, standoff, and grasping configurations for end effector
    pce_standoff = [-0.025,0,0.15]
    r = Rot.from_euler('zyx', [0, 145, 0], degrees = True)
    Rce = r.as_matrix()
    Tce_standoff = mr.RpToTrans(Rce,pce_standoff)

    pce_grasp = [0.0,0,0.001]
    Tce_grasp = mr.RpToTrans(Rce,pce_grasp)

    # Change all configurations to Space Frame
    Tse_init_standoff = np.dot(Tsc_init,Tce_standoff)
    Tse_init_grasp = np.dot(Tsc_init,Tce_grasp)
    Tse_final_standoff = np.dot(Tsc_final,Tce_standoff)
    Tse_final_grasp = np.dot(Tsc_final,Tce_grasp)


    gripper_states = [] # sequence of gripper styates
    traj_seq = [] # append each traj_list


    pce_standoff = np.array([-0.025,0,0.15])
    r = Rot.from_euler('zyx', [0, 145, 0], degrees = True)
    Rce = r.as_matrix()
    Tce_standoff = mr.RpToTrans(Rce,pce_standoff)

    pce_grasp = [0.0,0,0.025]
    Tce_grasp = mr.RpToTrans(Rce,pce_grasp)

    # Define Time for each trajectory segment
    k = 1 # number of trajectory reference configurations per 0.01 seconds

    # Change all configurations to Space Frame
    Tse_init_standoff = Tsc_init@Tce_standoff
    Tse_init_grasp = Tsc_init@Tce_grasp
    Tse_final_standoff = Tsc_final@Tce_standoff
    Tse_final_grasp = Tsc_final@Tce_grasp

    # Move from initial configuration to Tce_standoff
    T_start = Tse_init
    T_end = Tse_init_standoff
    gripper_state = 0
    traj = tg.TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)
    gripper_states.append(gripper_state)
    traj_seq.append(traj)

    # Move to grasp configuration
    T_start = T_end
    T_end = Tse_init_grasp
    gripper_state = 0
    traj = tg.TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)
    gripper_states.append(gripper_state)
    traj_seq.append(traj)

    # Close gripper
    traj_vec = tg.gripper_open_close_trajectory(T_end, k)

    gripper_state = 1
    tg.write_trajectory_to_csv(traj_vec,gripper_state, csv_name)
    gripper_states.append(gripper_state)
    traj_seq.append(traj_vec)

    # Move from current configuration to final standoff configuration
    T_start = T_end
    T_end = Tse_final_standoff
    gripper_state = 1
    traj = tg.TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)
    gripper_states.append(gripper_state)
    traj_seq.append(traj)

    # Move from current configuration to final standoff configuration
    T_start = T_end
    T_end = Tse_final_grasp
    gripper_state = 1
    traj = tg.TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)
    gripper_states.append(gripper_state)
    traj_seq.append(traj)

    # Open gripper
    traj_vec = tg.gripper_open_close_trajectory(T_end, k)
    gripper_state = 0
    tg.write_trajectory_to_csv(traj_vec, gripper_state, csv_name)
    gripper_states.append(gripper_state)
    traj_seq.append(traj_vec)

    return traj_seq, gripper_states


#unitTest
    
csv_name = 'utilstest.csv'
# Define initial, final, standoff, and grasping configurations
Tsc_init = np.array([[1,0,0,1],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]])

Tsc_final = np.array([[0,1,0,0],
                 [-1,0,0,-1],
                 [0,0,1,0],
                 [0,0,0,1]])

Tse_init = np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,1],
                 [0,0,0,1]])

k = 1 # number of trajectory reference configurations per 0.01 seconds
max_vel = .10

traj_list_complete, gripper_state_complete = trajectory_sequence(Tse_init, Tsc_init, Tsc_final, max_vel, k, csv_name)
