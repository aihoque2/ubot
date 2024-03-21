import numpy as np
import numpy.linalg as nla
import modern_robotics as mr


def FeedbackControl(Tse:np.ndarray, Tse_d:np.ndarray, Tse_dnext:np.ndarray, k: int):
    """
    feedforward+feedback control law as discussed in class.
    
    Tse: current ee config
    Tse_d : current "reference" ee config
    Tse_d_next: ee config at next timestep

    k_i: scalar of integral gain
    k_p: scalar of proportional gain

    """
    dt = 0.01

    Xe = Tse
    Xd = Tse_d
    Xd_next = Tse_dnext

    kp = 0.045
    ki = 0.35
    Kp = kp * np.eye(4)
    Ki = ki* np.eye(4)
 
    Xd_inv = nla.inv(Xd)
    Xe_inv = nla.inv(Xe)

    Vd_mat = mr.MatrixLog6((Xd_inv @ Xd_next)/dt)
    Vd = mr.se3ToVec(Vd_mat)
    Verr_mat = mr.MatrixLog6(Xe_inv @ Xd)
    Adj = mr.Adjoint(Xe_inv @ Xd)
    V_new =  np.dot(Adj, Vd) + Kp@Xe + Ki


def calc_jacobian()
