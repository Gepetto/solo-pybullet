# coding: utf8

# Load modules
import numpy as np

def PD(qa_ref, qa_dot_ref, qa, qa_dot, dt, Kp=1, Kd=1, torque_sat=100, torques_ref=np.zeros((8,1))):

    # Output torques
    torques = Kp * (qa_ref - qa) + Kd * (qa_dot_ref - qa_dot) + torques_ref
    
    # Saturation to limit the maximal value that torques can have
    torques = np.maximum(np.minimum(torques, torque_sat * np.ones((8,1))), -torque_sat * np.ones((8,1))) 

    return torques
