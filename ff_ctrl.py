import numpy as np
import modern_robotics as mr


def FeedbackControl(Tse:np.ndarray, Tse_d:np.ndarray, Tse_dnext:np.ndarray, Kp:np.ndarray, Ki: np.ndarray,  dt:float):
    """
    feedforward+feedback control law as discussed in class.
    
    Tse: current ee config
    Tse_d : current "reference" ee config
    Tse_d_next: ee config at next timestep

    """

    Xe = Tse
    Xd = Tse_d
    Xd_next = Tse_dnext

    Ve = Xe @ Xe
    Vd = Xd @ Xd


    pass