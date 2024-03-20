import numpy as np
import modern_robotics as mr


def FeedbackControl(Tse:np.ndarray, Tse_d_current:np.ndarray, Tse_dnext:np.ndarray, Kp:np.ndarray, Ki: np.ndarray,  dt:float):
    """
    feedforward+feedback control law as discussed in class.
    
    Tse: current ee config
    Tse_d_current : current reference ee config
    Tse_d_next: ee config at next timestep

    """

    X_e = Tse
    X_d = Tse_d_current
    X_dnext = Tse_dnext


    pass