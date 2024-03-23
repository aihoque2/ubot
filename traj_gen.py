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

def add_df_row(config_matrix_T,gripper_state, csv_name):

    # unpack the data
    data_row = create_data_row(config_matrix_T,gripper_state)

    # create the file and write the row
    filepath = os.path.dirname(os.path.abspath(__file__))
    filename = filepath + '//' + csv_name

    with open(filename, 'a', newline='') as f:
        csvwriter = csv.writer(f,)

        csvwriter.writerow(data_row)

def write_trajectory_to_csv(trajs,gripper_state, csv_name):
    for matrix in trajs:
        add_df_row(matrix,gripper_state, csv_name)

def gripper_open_close_trajectory(config_matrix, k):

    tf = 1 # 1 second for gripper to close

    N = int(tf*k/0.01)

    traj_vec = [config_matrix]*N

    return traj_vec

def trajectory_speed(frame_start,frame_end, max_velocity, k):
    p1 = frame_start[:-1,-1:].flatten()
    p2 = frame_end[:-1,-1:].flatten()

    dist = nla.norm(p1-p2)
    dt = 0.01
    tf = dist/max_velocity
    N = tf*k/dt # dt=0.01

    return tf, N

def TrajectoryGenerator(T_start, T_end, k, gripper_state, max_velocity, csv_name):
    """
    meat and potatoes of our code
    """
    tf, N = trajectory_speed(T_start, T_end, max_velocity, k)
    trajs = mr.ScrewTrajectory(T_start, T_end, tf, N, 3)
    write_trajectory_to_csv(trajs,gripper_state, csv_name)
    return trajs

    
"""
Scene 8 Test

Move from initial configuration to cube
Move to grasp configuration
Close gripper
Move from current configuration to final cube configuration
Move to final grasp configuration
Open gripper

"""
if __name__ == "__main__":
    csv_name = 'milestone1.csv'
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
    max_vel = .10

    # Change all configurations to Space Frame
    Tse_init_standoff = Tsc_init@Tce_standoff
    Tse_init_grasp = Tsc_init@Tce_grasp
    Tse_final_standoff = Tsc_final@Tce_standoff
    Tse_final_grasp = Tsc_final@Tce_grasp

    # Move from initial configuration to Tce_standoff
    T_start = Tse_init
    T_end = Tse_init_standoff
    gripper_state = 0
    TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)

    # Move to grasp configuration
    T_start = T_end
    T_end = Tse_init_grasp
    gripper_state = 0
    TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)

    # Close gripper
    traj_vec = gripper_open_close_trajectory(T_end, k)
    gripper_state = 1
    write_trajectory_to_csv(traj_vec, gripper_state, csv_name)

    # Move from current configuration to final standoff configuration
    T_start = T_end
    T_end = Tse_final_standoff
    gripper_state = 1
    TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)

    # Move from current configuration to final standoff configuration
    T_start = T_end
    T_end = Tse_final_grasp
    gripper_state = 1
    TrajectoryGenerator(T_start, T_end, k, gripper_state, max_vel, csv_name)

    # Open gripper
    traj_vec = gripper_open_close_trajectory(T_end, k)
    gripper_state = 0
    write_trajectory_to_csv(traj_vec, gripper_state, csv_name)
