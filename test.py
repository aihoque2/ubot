import os
import csv
import pandas as pd
import numpy as np
import numpy.linalg as nla

from scipy.spatial.transform import Rotation as Rot


import modern_robotics as mr

dataframe = pd.DataFrame()


def create_data_row(T,gripper_state):
    """
    params:
    T: transformation matrix of shape (4,4)
    gripper_state: int 0 closed, 1 open

    return:
    list in format
    [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state]

    """
    R = T[:-1,0:-1]
    p = T[:-1,-1:]

    # unpack position state to one list
    data_row = np.concatenate((R,p,gripper_state), axis=None) # axis=None for flattening
    data_row = list(data_row)

    return data_row

def add_df_row(config_matrix_T,gripper_state):

    # unpack the data
    data_row = create_data_row(config_matrix_T,gripper_state)

    # create the file and write the row
    filepath = os.path.dirname(os.path.abspath(__file__))
    filename = filepath + '//' + 'results.csv'

    with open(filename, 'a', newline='') as f:
        csvwriter = csv.writer(f,)

        csvwriter.writerow(data_row)

def write_trajectory_to_csv(traj_list,gripper_state):
    for matrix in traj_list:
        add_df_row(matrix,gripper_state)

def gripper_open_close_trajectory(config_matrix, k):

    Tf = 1 # 1 second for gripper to close

    N = int(Tf*k/0.01)

    traj_list = [config_matrix]*N

    return traj_list

def trajectory_speed(frame_start,frame_end, max_velocity, k):
    p1 = frame_start[:-1,-1:].flatten()
    p2 = frame_end[:-1,-1:].flatten()

    dist = nla.norm(p1-p2)

    Tf = dist/max_velocity
    N = Tf*k/0.01

    return Tf, N

def TrajectoryGenerator(T_start, T_end, k, gripper_state):
    """
    meat and otatoes of our code
    """
    max_velocity = 0.10 # m/s
    Tf, N = trajectory_speed(T_start, T_end, max_velocity, k)
    traj_list = mr.ScrewTrajectory(T_start, T_end, Tf, N, 3)
    write_trajectory_to_csv(traj_list,gripper_state)

    
"""
Scene 8 Test

Move from initial configuration to cube
Move to grasp configuration
Close gripper
Move from current configuration to final cube configuration
Move to final grasp configuration
Open gripper

"""
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
TrajectoryGenerator(T_start, T_end, k, gripper_state)

# Move to grasp configuration
T_start = T_end
T_end = Tse_init_grasp
gripper_state = 0
TrajectoryGenerator(T_start, T_end, k, gripper_state)

# Close gripper
traj_list = gripper_open_close_trajectory(T_end, k)
gripper_state = 1
write_trajectory_to_csv(traj_list,gripper_state)

# Move from current configuration to final standoff configuration
T_start = T_end
T_end = Tse_final_standoff
gripper_state = 1
TrajectoryGenerator(T_start, T_end, k, gripper_state)

# Move from current configuration to final standoff configuration
T_start = T_end
T_end = Tse_final_grasp
gripper_state = 1
TrajectoryGenerator(T_start, T_end, k, gripper_state)

# Open gripper
traj_list = gripper_open_close_trajectory(T_end, k)
gripper_state = 0
write_trajectory_to_csv(traj_list,gripper_state)
