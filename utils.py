"""
utils.py
"""

import traj_gen as tg

""""
generate the whole sequence
"""

def trajectory_sequence(Tse_init, Tsc_init, Tsc_final, max_vel, k)
    gripper_states = [] # sequence of gripper styates
    trajectory_sequence = [] # append each traj_list