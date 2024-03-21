import numpy as np
import numpy.linalg as nla
import modern_robotics as mr


def FeedbackControl(Tse:np.ndarray, Tse_d:np.ndarray, Tse_dnext:np.ndarray, kp: float, ki: float):
    """
    feedforward+feedback control law as discussed in class.
    
    Tse: current ee config
    Tse_d : current "reference" ee config
    Tse_d_next: ee config at next timestep

    ki: scalar of integral gain
    kp: scalar of proportional gain

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


    # screw axes
    B1 = np.array([0, 0, 1, 0, 0.033, 0])
    B2 = np.array([0, -1, 0, -0.5076, 0, 0])
    B3 = np.array([0,-1,0,-0.3526, 0, 0]) 
    B4 = np.array([0, -1, 0, -0.2176, 0, 0])
    B5 = np.array([0, 0, 1, 0, 0, 0])

    Jb_0 = np.array([B1, B2, B3, B4, B5])
    