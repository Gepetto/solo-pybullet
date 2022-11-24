# coding: utf8

import numpy as np  # Numpy library
import pybullet_data
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

import pybullet as p  # PyBullet simulator


def configure_simulation(dt, enableGUI):
    global jointTorques
    # Load the robot for Pinocchio
    solo = loadSolo(True)
    solo.initDisplay(loadModel=True)

    # Start the client for PyBullet
    if enableGUI:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)  # noqa
    # p.GUI for graphical version
    # p.DIRECT for non-graphical version

    # Set gravity (disabled by default)
    p.setGravity(0, 0, -9.81)

    # Load horizontal plane for PyBullet
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Load the robot for PyBullet
    robotStartPos = [0, 0, 0.35]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath(
        "/opt/openrobots/share/example-robot-data/solo_description/robots"
    )
    robotId = p.loadURDF("solo.urdf", robotStartPos, robotStartOrientation)

    # Set time step of the simulation
    # dt = 0.001
    p.setTimeStep(dt)
    # If True then we will sleep in the main loop to have a frequency of 1/dt
    # realTimeSimulation = True

    # Disable default motor control for revolute joints
    revoluteJointIndices = [0, 1, 3, 4, 6, 7, 9, 10]
    p.setJointMotorControlArray(
        robotId,
        jointIndices=revoluteJointIndices,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[0.0 for m in revoluteJointIndices],
        forces=[0.0 for m in revoluteJointIndices],
    )

    # Enable torque control for revolute joints
    jointTorques = [0.0 for m in revoluteJointIndices]
    p.setJointMotorControlArray(
        robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques
    )

    # Compute one step of simulation for initialization
    p.stepSimulation()

    return robotId, solo, revoluteJointIndices


def getPosVelJoints(robotId, revoluteJointIndices):
    """get the position/velocity of the base and the angular position/velocity of all
    joints."""

    jointStates = p.getJointStates(robotId, revoluteJointIndices)  # State of all joints
    baseState = p.getBasePositionAndOrientation(
        robotId
    )  # Position of the free flying base
    baseVel = p.getBaseVelocity(robotId)  # Velocity of the free flying base

    # Reshaping data into q and qdot
    q = np.vstack(
        (
            np.array([baseState[0]]).transpose(),
            np.array([baseState[1]]).transpose(),
            np.array(
                [[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]
            ).transpose(),
        )
    )
    qdot = np.vstack(
        (
            np.array([baseVel[0]]).transpose(),
            np.array([baseVel[1]]).transpose(),
            np.array(
                [[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]
            ).transpose(),
        )
    )

    return q, qdot
