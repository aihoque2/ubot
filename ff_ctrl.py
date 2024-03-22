import numpy as np
import numpy.linalg as nla
import modern_robotics as mr
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

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

def joint_limits(positions):
    arm_thetas = positions[1]

    joint_free = np.ones(arm_thetas.shape, dtype=bool)

    if arm_thetas[2] > -0.01:
        joint_free[2] = False
    
    if arm_thetas[3] > -0.01:
        joint_free[3] = False

    return joint_free        

    

def FeedbackControl(Tse:np.ndarray, Tse_d:np.ndarray, Tse_dnext:np.ndarray, kp: float, ki: float, positions, test_joints, errors):
    """
    feedforward+feedback control law as discussed in class.
    
    Tse: current ee config
    Tse_d : current "reference" ee config
    Tse_d_next: ee config at next timestep

    ki: scalar of integral gain
    kp: scalar of proportional gain

    """
    r = 0.0475
    l = 0.235
    w = 0.15
    dt = 0.01

    Xe = Tse
    Xd = Tse_d
    Xd[np.abs(Xd) < 1e-7] = 0.0
    Xd_next = Tse_dnext
    Xd_next[np.abs(Xd_next) < 1e-7] = 0.0


    Kp = kp * np.eye(6)
    Ki = ki* np.eye(6)

    Xd_inv = mr.TransInv(Xd)
    Xe_inv = mr.TransInv(Xe)

    Vd_mat = mr.MatrixLog6((Xd_inv @ Xd_next)/dt)
    Vd = mr.se3ToVec(Vd_mat)
    
    Xerr = mr.se3ToVec(mr.MatrixLog6(Xe_inv @ Xd))

    """
    TODO: calculate error sum
    """

    Adj = mr.Adjoint(Xe_inv @ Xd)
    print("adjoint shape: ", Vd.shape)
    errors += Xerr*dt

    # Control Law
    V_new =  Adj@Vd + Kp@Xerr + Ki@errors
    print("result of adj@Vd: ", Adj@Vd)

    """
    CALC'ing JACOBIAN
    """

    H_0 = (1/r)*np.array([[-l-w,1,-1],
                  [l+w,1,1],
                  [l+w,1,-1],
                  [-l-w,1,1]])

    F6 = np.zeros([6,4])
    F = np.linalg.pinv(H_0)
    F6[2:-1,:] = F

    theta_arm = positions[1]
     # EE config in {0} coords
    T0e = mr.FKinBody(M, B.T, theta_arm)
    T0e[np.abs(T0e) < 1e-7] = 0
    Teb = mr.TransInv(T0e) @ mr.TransInv(Tb0)

    J_base = mr.Adjoint(Teb) @ F6
    J_arm = mr.JacobianBody(B.T, theta_arm)

    # end effector jacobian
    Je = np.concatenate((J_base, J_arm), axis=1)
    Je[np.abs(Je) < 1e-7] = 0

    #Je[:, np.where(test_joints == False)] = 0

    u_thetas = nla.pinv(Je)@V_new
    u_thetas[np.abs(u_thetas) < 1e-7] = 0.0
    return u_thetas, Xerr, errors, Je, V_new, Vd    


def get_Tse(positions, T0e, Tb0):
    
    phi = positions[0]
    x = positions[1]
    y = positions[2]
    z = 0.0963 # chassis height

    Tsb = np.array([[np.cos(phi),-np.sin(phi),0,x],
                    [np.sin(phi),np.cos(phi),0,y],
                    [0,0,1,z],
                    [0,0,0,1]])
    

    Teb = mr.TransInv(T0e) @ mr.TransInv(Tb0)

    Tse = Tsb @ mr.TransInv(Teb)
    return Tse



""""
testing
"""
chassis_config = np.array([0.0,0.0,0.0])
arm_thetas = np.array([0.0,0.0,0.2,-1.6,0.0])
wheel_thetas = np.array([0.0,0.0,0.0,0.0])
position_state = [chassis_config,arm_thetas,wheel_thetas]

test_joints = joint_limits(position_state)



# Fixed Offset between the end effector and the base of the arm frameb calculated from trajectory generation

# EE config in {0} coords
T0e = mr.FKinBody(M, B.T, arm_thetas.T)

# Testing the functions
Tse = get_Tse(position_state[0], T0e, Tb0)


# Feedback control law
Xd = np.array([[0,0,1,0.5],
                [0,1,0,0],
                [-1,0,0,0.5],
                [0,0,0,1]])

Xd_next = np.array([[0,0,1,0.6],
                    [0,1,0,0],
                    [-1,0,0,0.3],
                    [0,0,0,1]])
    
#kp = 0.045
#ki = 0.35
kp = 1.0
ki = 0.0

Ki_error = 0

dt = 0.01

u_theta, Xerr, Ki_error, J_e, V_t, Vd = FeedbackControl(Tse, Xd, Xd_next, kp, ki, position_state, test_joints, Ki_error)

print("++++++++++++++++++++++++++++++")
print("Vd")
print(Vd)
print("++++++++++++++++++++++++++++++")
print("Xerr")
print(Xerr)
print("++++++++++++++++++++++++++++++")
print("Je")
print(J_e)
print("++++++++++++++++++++++++++++++")
print("V_t")
print(V_t)
print("++++++++++++++++++++++++++++++")
print("U_theta")
print(u_theta)