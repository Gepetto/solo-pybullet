# coding: utf8

# Other modules
import numpy as np
# Pinocchio modules
import pinocchio as pin  # Pinocchio library

from .PD import PD

################
#  CONTROLLER ##
################


def c(q, qdot, dt):
    # unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
    # qu = q[:7]
    # actuated, [q1, q2, ..., q8] angular position of the 8 motors
    qa = q[7:]
    # [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes
    # of the world
    # qu_dot = qdot[:6]
    qa_dot = qdot[6:]  # angular velocity of the 8 motors

    qa_ref = np.zeros((8, 1))  # target angular positions for the motors
    qa_dot_ref = np.zeros((8, 1))  # target angular velocities for the motors

    ###############################################

    # Insert code here to set qa_ref and qa_dot_ref

    ###############################################

    # Parameters for the PD controller
    Kp = 8.
    Kd = 0.06
    torque_sat = 3  # torque saturation in N.m
    torques_ref = np.zeros((8, 1))  # feedforward torques

    # Call the PD controller
    torques = PD(qa_ref, qa_dot_ref, qa, qa_dot, dt, Kp, Kd, torque_sat, torques_ref)

    # torques must be a numpy array of shape (8, 1) containing the torques applied to the 8 motors
    return torques


########################
## WALKING_CONTROLLER ##
########################

# Method : Inverse Kinematics

# Initialization of the controller's parameters
q_ref = np.zeros((15, 1))
flag_q_ref = True


def c_walking_IK(q, qdot, dt, solo, t_simu):
    # unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
    # qu = q[:7]
    qa = q[7:]  # actuated, [q1, q2, ..., q8] angular position of the 8 motors
    # [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes
    # of the world
    # qu_dot = qdot[:6]
    qa_dot = qdot[6:]  # angular velocity of the 8 motors

    qa_ref = np.zeros((8, 1))  # target angular positions for the motors
    qa_dot_ref = np.zeros((8, 1))  # target angular velocities for the motors

    ###############################################
    # Insert code here to set qa_ref and qa_dot_ref

    from numpy.linalg import pinv

    global q_ref, flag_q_ref, T, dx, dz

    if flag_q_ref:
        q_ref = solo.q0.copy()
        flag_q_ref = False

    # Initialization of the variables
    K = 100.  # Convergence gain
    T = 0.2  # period of the foot trajectory
    xF0 = 0.19  # initial position of the front feet
    xH0 = -0.19  # initial position of the hind feet
    z0 = 0.0  # initial altitude of each foot
    dx = 0.03  # displacement amplitude by x
    dz = 0.06  # displacement amplitude by z

    # Get the frame index of each foot
    ID_FL = solo.model.getFrameId("FL_FOOT")
    ID_FR = solo.model.getFrameId("FR_FOOT")
    ID_HL = solo.model.getFrameId("HL_FOOT")
    ID_HR = solo.model.getFrameId("HR_FOOT")

    # function defining the feet's trajectory
    def ftraj(t, x0, z0):  # arguments : time, initial position x and z
        global T, dx, dz
        x = []
        z = []
        if t >= T:
            t %= T
        x.append(x0 - dx * np.cos(2 * np.pi * t / T))
        if t <= T / 2.:
            z.append(z0 + dz * np.sin(2 * np.pi * t / T))
        else:
            z.append(0)
        return np.matrix([x, z])

    # Compute/update all the joints and frames
    pin.forwardKinematics(solo.model, solo.data, q_ref)
    pin.updateFramePlacements(solo.model, solo.data)

    # Get the current height (on axis z) and the x-coordinate of the front left foot
    xz_FL = solo.data.oMf[ID_FL].translation[0::2]
    xz_FR = solo.data.oMf[ID_FR].translation[0::2]
    xz_HL = solo.data.oMf[ID_HL].translation[0::2]
    xz_HR = solo.data.oMf[ID_HR].translation[0::2]

    # Desired foot trajectory
    xzdes_FL = ftraj(t_simu, xF0, z0)
    xzdes_HR = ftraj(t_simu, xH0, z0)
    xzdes_FR = ftraj(t_simu + T / 2, xF0, z0)
    xzdes_HL = ftraj(t_simu + T / 2, xH0, z0)

    # Calculating the error
    err_FL = xz_FL - xzdes_FL
    err_FR = xz_FR - xzdes_FR
    err_HL = xz_HL - xzdes_HL
    err_HR = xz_HR - xzdes_HR

    # Computing the local Jacobian into the global frame
    oR_FL = solo.data.oMf[ID_FL].rotation
    oR_FR = solo.data.oMf[ID_FR].rotation
    oR_HL = solo.data.oMf[ID_HL].rotation
    oR_HR = solo.data.oMf[ID_HR].rotation

    # Getting the different Jacobians
    fJ_FL3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_FL)[:3, -8:]  # Take only the translation terms
    oJ_FL3 = oR_FL * fJ_FL3  # Transformation from local frame to world frame
    oJ_FLxz = oJ_FL3[0::2, -8:]  # Take the x and z components

    fJ_FR3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_FR)[:3, -8:]
    oJ_FR3 = oR_FR * fJ_FR3
    oJ_FRxz = oJ_FR3[0::2, -8:]

    fJ_HL3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_HL)[:3, -8:]
    oJ_HL3 = oR_HL * fJ_HL3
    oJ_HLxz = oJ_HL3[0::2, -8:]

    fJ_HR3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_HR)[:3, -8:]
    oJ_HR3 = oR_HR * fJ_HR3
    oJ_HRxz = oJ_HR3[0::2, -8:]

    # Displacement error
    nu = np.vstack([err_FL, err_FR, err_HL, err_HR])

    # Making a single x&z-rows Jacobian vector
    J = np.vstack([oJ_FLxz, oJ_FRxz, oJ_HLxz, oJ_HRxz])

    # Computing the velocity
    qa_dot_ref = -K * pinv(J) * nu
    q_dot_ref = np.concatenate((np.zeros([6, 1]), qa_dot_ref))

    # Computing the updated configuration
    q_ref = pin.integrate(solo.model, q_ref, q_dot_ref * dt)
    qa_ref = q_ref[7:]

    # DONT FORGET TO RUN GEPETTO-GUI BEFORE RUNNING THIS PROGRAMM #
    solo.display(q)  # display the robot in the viewer Gepetto-GUI given its configuration q

    # End of the control code
    ###############################################

    # Parameters for the PD controller
    Kp = 8.
    Kd = 0.2
    torque_sat = 3  # torque saturation in N.m
    torques_ref = np.zeros((8, 1))  # feedforward torques

    # Call the PD controller
    torques = PD(qa_ref, qa_dot_ref, qa, qa_dot, dt, Kp, Kd, torque_sat, torques_ref)

    # torques must be a numpy array of shape (8, 1) containing the torques applied to the 8 motors
    return torques


# Method : Inverse Dynamics

# Initialization of the controller's parameters
q_dot_ref = np.zeros((14, 1))
q_dot2_ref = np.zeros((14, 1))
qa_dot2_ref = np.zeros((8, 1))


def c_walking_ID(q, qdot, dt, solo, t_simu):

    qu = q[:
           7]  # unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
    qa = q[7:]  # actuated, [q1, q2, ..., q8] angular position of the 8 motors
    qu_dot = qdot[:
                  6]  # [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes of the world
    qa_dot = qdot[6:]  # angular velocity of the 8 motors

    qa_ref = np.zeros((8, 1))  # target angular positions for the motors
    qa_dot_ref = np.zeros((8, 1))  # target angular velocities for the motors

    ###############################################
    # Insert code here to set qa_ref and qa_dot_ref

    from numpy.linalg import pinv, inv

    global qa_dot2_ref, q_dot2_ref, q_dot_ref, q_ref, flag_q_ref, T, dx, dz

    if flag_q_ref:
        q_ref = solo.q0.copy()
        q_dot_ref = qdot
        flag_q_ref = False

    #Initialization of the variables
    K = 100.  #Convergence gain
    T = 0.2  #period of the foot trajectory
    xF0 = 0.19  #initial position of the front feet
    xH0 = -0.19  #initial position of the hind feet
    z0 = 0.0  #initial altitude of each foot
    dx = 0.03  #displacement amplitude by x
    dz = 0.06  #displacement amplitude by z

    # Get the frame index of each foot
    ID_FL = solo.model.getFrameId("FL_FOOT")
    ID_FR = solo.model.getFrameId("FR_FOOT")
    ID_HL = solo.model.getFrameId("HL_FOOT")
    ID_HR = solo.model.getFrameId("HR_FOOT")

    # function defining the feet's trajectory
    def ftraj(t, x0, z0):  #arguments : time, initial position x and z
        global T, dx, dz
        x = []
        z = []
        if t >= T:
            t %= T
        x.append(x0 - dx * np.cos(2 * np.pi * t / T))
        if t <= T / 2.:
            z.append(z0 + dz * np.sin(2 * np.pi * t / T))
        else:
            z.append(0)
        return np.matrix([x, z])

    # Compute/update all the joints and frames
    pin.forwardKinematics(solo.model, solo.data, q_ref)
    pin.updateFramePlacements(solo.model, solo.data)

    # Get the current height (on axis z) and the x-coordinate of the front left foot
    xz_FL = solo.data.oMf[ID_FL].translation[0::2]
    xz_FR = solo.data.oMf[ID_FR].translation[0::2]
    xz_HL = solo.data.oMf[ID_HL].translation[0::2]
    xz_HR = solo.data.oMf[ID_HR].translation[0::2]

    # Desired foot trajectory
    xzdes_FL = ftraj(t_simu, xF0, z0)
    xzdes_HR = ftraj(t_simu, xH0, z0)
    xzdes_FR = ftraj(t_simu + T / 2, xF0, z0)
    xzdes_HL = ftraj(t_simu + T / 2, xH0, z0)

    # Calculating the error
    err_FL = xz_FL - xzdes_FL
    err_FR = xz_FR - xzdes_FR
    err_HL = xz_HL - xzdes_HL
    err_HR = xz_HR - xzdes_HR

    # Computing the local Jacobian into the global frame
    oR_FL = solo.data.oMf[ID_FL].rotation
    oR_FR = solo.data.oMf[ID_FR].rotation
    oR_HL = solo.data.oMf[ID_HL].rotation
    oR_HR = solo.data.oMf[ID_HR].rotation

    # Getting the different Jacobians
    fJ_FL3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_FL)[:3, -8:]  #Take only the translation terms
    oJ_FL3 = oR_FL * fJ_FL3  #Transformation from local frame to world frame
    oJ_FLxz = oJ_FL3[0::2, -8:]  #Take the x and z components

    fJ_FR3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_FR)[:3, -8:]
    oJ_FR3 = oR_FR * fJ_FR3
    oJ_FRxz = oJ_FR3[0::2, -8:]

    fJ_HL3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_HL)[:3, -8:]
    oJ_HL3 = oR_HL * fJ_HL3
    oJ_HLxz = oJ_HL3[0::2, -8:]

    fJ_HR3 = pin.frameJacobian(solo.model, solo.data, q_ref, ID_HR)[:3, -8:]
    oJ_HR3 = oR_HR * fJ_HR3
    oJ_HRxz = oJ_HR3[0::2, -8:]

    # Computing the mass matrix M and the dynamic drift b
    M = pin.crba(solo.model, solo.data, q_ref)  #compute the mass matrix
    b = pin.rnea(solo.model, solo.data, q_ref, q_dot_ref, q_dot2_ref)  #compute the dynamic drift

    # Calculating reference torques
    Kp_tau = 8.
    Kd_tau = 0.2
    torques_ref = -Kp_tau * (q_ref[7:] - q[7:]) - Kd_tau * qa_dot_ref

    # Calculating joint accelerations
    qa_dot2_ref = inv(M[6:, 6:]) * (torques_ref - b[6:])
    q_dot2_ref = np.concatenate((np.zeros([6, 1]), qa_dot2_ref))

    # Computing the velocity
    qa_dot_ref += qa_dot2_ref * dt
    q_dot_ref = np.concatenate((np.zeros([6, 1]), qa_dot_ref))

    # Computing the updated configuration
    q_ref = pin.integrate(solo.model, q_ref, q_dot_ref * dt)
    qa_ref = q_ref[7:]

    # DONT FORGET TO RUN GEPETTO-GUI BEFORE RUNNING THIS PROGRAMM #
    solo.display(q)  # display the robot in the viewer Gepetto-GUI given its configuration q

    # End of the control code
    ###############################################

    # Parameters for the PD controller
    Kp = 8.
    Kd = 0.2
    torque_sat = 3  # torque saturation in N.m
    torques_ref = np.zeros((8, 1))  # feedforward torques

    # Call the PD controller
    torques = PD(qa_ref, qa_dot_ref, qa, qa_dot, dt, Kp, Kd, torque_sat, torques_ref)

    # torques must be a numpy array of shape (8, 1) containing the torques applied to the 8 motors
    return torques
