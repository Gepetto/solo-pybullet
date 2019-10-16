# coding: utf8

# Pinocchio modules
import pinocchio as pin # Pinocchio library
from pinocchio.utils import * # Utilitary functions from Pinocchio
from pinocchio.robot_wrapper import RobotWrapper # Robot Wrapper to load an URDF in Pinocchio

# Other modules
import numpy as np
from PD import *

################
## CONTROLLER ##
################

def c(q, qdot, dt):

    qu = q[:7] # unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
    qa = q[7:] # actuated, [q1, q2, ..., q8] angular position of the 8 motors
    qu_dot = qdot[:6] # [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes of the world
    qa_dot = qdot[6:] # angular velocity of the 8 motors
    
    qa_ref = np.zeros((8,1)) # target angular positions for the motors
    qa_dot_ref = np.zeros((8,1)) # target angular velocities for the motors

    #
    # Insert code here to set qa_ref and qa_dot_ref
    #
    
    # Parameters for the PD controller
    Kp = 0
    Kd = 0
    torque_sat = 3 # torque saturation in N.m
    torques_ref = np.zeros((8,1)) # feedforward torques

    # Call the PD controller
    torques = PD(qa_ref, qa_dot_ref, qa, qa_dot, dt, Kp, Kd, torque_sat, torques_ref)
   
    # torques must be a numpy array of shape (8, 1) containing the torques applied to the 8 motors
    return torques