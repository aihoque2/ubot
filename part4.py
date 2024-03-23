import numpy as np
import numpy.linalg as nla

import matplotlib.pyplot as plt
import modern_robotics as mr

import os
import csv

from utils import trajectory_sequence
from ff_ctrl import FeedbackControl, joint_limits, get_Tse
from kin_sim import NextState

np.set_printoptions(suppress=True)
np.set_printoptions(precision=3)

"""
we shall combine trajectory generation , kin simulation, and feedforward
"""

"""
globals
"""
# T matrix from chassis frame to b to arm frame O
Tb0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])

# M matrix of end-effector in arm frame coords
M = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])

# screw axes
B1 = np.array([0, 0, 1, 0, 0.033, 0])
B2 = np.array([0, -1, 0, -0.5076, 0, 0])
B3 = np.array([0,-1, 0, -0.3526, 0, 0]) 
B4 = np.array([0, -1, 0, -0.2176, 0, 0])
B5 = np.array([0, 0, 1, 0, 0, 0])

B = np.array([B1, B2, B3, B4, B5])

"""
helper fns
"""
# {b}: car body frame
# {c}: cube frame
# {e}: end effector (not mistaken with err)
def create_data_row(position_state, gripper_state):
    data_row = np.concatenate((position_state[0],position_state[1],position_state[2],np.array(gripper_state)),axis = None)
    data_row = list(data_row)

    return data_row

def data_rows_to_csv(data_rows, csv_name):
    # create the file and write the row
    filepath = os.path.dirname(os.path.abspath(__file__))
    filename = filepath + '//' + csv_name
    with open(filename, 'a', newline='') as f:
        csvwriter = csv.writer(f,)
        
        for data_row in data_rows:
            csvwriter.writerow(data_row)

def plot_errors(errors, Kp, Ki):
    """
    Plot the errors over time for a PI-controlled system.

    Parameters:
    - errors: A list of 6-element arrays representing the error at each time step.
    - Kp: A 6x6 numpy array representing the proportional gains.
    - Ki: A 6x6 numpy array representing the integral gains.
    """
    # Convert errors list of arrays into a 2D NumPy array
    errors_array = np.array(errors)
    
    # Assuming dt (time step) is known, modify as necessary
    dt = 0.01
    time_steps = np.arange(errors_array.shape[0]) * dt  # Create a time array

    # Plot each component of the error
    plt.figure(figsize=(12, 8))

    # Plot position errors
    plt.subplot(2, 1, 1)
    plt.plot(time_steps, errors_array[:, 0], label='ex')
    plt.plot(time_steps, errors_array[:, 1], label='ey')
    plt.plot(time_steps, errors_array[:, 2], label='ez')
    plt.title(f'Position Errors Over Time | Kp: {Kp[0][0]} | Ki: {Ki[0][0]}')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error')
    plt.legend()

    # Plot orientation errors
    plt.subplot(2, 1, 2)
    plt.plot(time_steps, errors_array[:, 3], label='ewx')
    plt.plot(time_steps, errors_array[:, 4], label='ewy')
    plt.plot(time_steps, errors_array[:, 5], label='ewz')
    plt.title('Orientation Errors Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation Error')
    plt.legend()

    plt.tight_layout()
    plt.show()

"""
entree
"""

def full_function(Tsc_init, Tsc_final, Tsb_initial, Tsb_desired, position_init, ki, kp, csv_name):

    dt = 0.01
    position_state = position_init
    print("position state: ", position_state)

    # control theory stuff
    errors = np.array([[0,0,0,0,0,0]])
    Ki_error = 0

    Ki = ki*np.eye(6)
    Kp = kp*np.eye(6)

    # Define Time for each trajectory segment
    max_velocity = 0.25 # m/s
    joint_speed_limit = 15
    k = 1 # number of trajectory reference configurations per 0.01 seconds

    trajs, gripper_states = trajectory_sequence(Tse_init, Tsc_init, Tsc_final, max_velocity, k, test1_name)
    for i in range(len(trajs)):
        ref_traj = trajs[i]
        gripper_state = gripper_states[i]
        data_rows = []
        for i in range(len(ref_traj)-2):

            test_joints = joint_limits(position_state[1])
            Xd = ref_traj[i]
            Xd_next = ref_traj[i+1]
            T0e = mr.FKinBody(M, B.T, arm_thetas.T)
            Tse = get_Tse(position_state[0], T0e, Tb0)
            u_theta, Xerr, Ki_error, J_e, V_t, Vd = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, position_state, test_joints, Ki_error)

            print("part4.py Xerr shape: ", Xerr.shape)
            arm_vels = np.array(u_theta[4:])
            wheel_vels = np.array(u_theta[0:4])
            errors = np.concatenate((errors,[Xerr]), axis = 0)

            vels = (arm_vels, wheel_vels)
            
            data_row = create_data_row(position_state, gripper_state)
            print("pt 4 data rows:", data_row)
            data_rows.append(data_row)
            #update position
            position_vec = NextState(np.array(data_row[0:12]), u_theta, dt, max_velocity) 
            position_state = [position_vec[0:3], position_vec[3:8], position_vec[8:]]
            print("new position state: ", position_state)
        # writing the csv file
        data_rows_to_csv(data_rows, csv_name)

    return errors

#  pass

test1_name = "best.csv"

chassis_config = np.array([0.0,0.0,0.09632])
arm_thetas = np.array([0.0,0.0,0.0,0.0,0.0])
wheel_thetas = np.array([0.0,0.0,np.deg2rad(90),np.deg2rad(-180)])
positions = [chassis_config,arm_thetas,wheel_thetas]

# Tse Modern Robotics Wiki Page. initial EE config
Tse_init = np.array([[0,0,1,0],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])

# 
Tsc_init = np.array([[1, 0, 0,     1],
					[0, 1, 0,     0.0],
					[0, 0, 1, 0.000],
					[0, 0, 0,     1]])

Tsc_final = np.array([[ 0, 1, 0,     0.047],
					[-1, 0, 0,    -1.047],
					[ 0, 0, 1, 0.027],
					[ 0, 0, 0,     1]])
phi = chassis_config[0]


x = chassis_config[1]
y = chassis_config[2]
z = 0.0963 # chassis height


Tsb_init = np.array([[np.cos(phi),-np.sin(phi),0,x],
                    [np.sin(phi),np.cos(phi),0,y],
                    [0,0,1,z],
                    [0,0,0,1]])
    
Tsb_desired =  np.eye(4)

max_velocity = 0.25 # m/s
joint_speed_limit = 15
k = 1 # number of trajectory reference configurations per 0.01 seconds

kp = 0.75
ki = 0.00
#kp = 0.045
#ki = 0.35

errors = full_function(Tsc_init, Tsc_final, Tsb_init, Tsb_desired, positions, ki, kp, test1_name)
plot_errors(errors, kp*np.eye(6), ki*np.eye(6))
