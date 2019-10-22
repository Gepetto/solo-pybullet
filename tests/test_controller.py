# coding: utf8

import unittest

import numpy as np
from example_robot_data import loadSolo

from solo_pybullet.controller import c_walking_IK
from solo_pybullet.PD import PD


class TestController(unittest.TestCase):
    def test_controller(self):

        solo = loadSolo(True)
        solo.initViewer(loadModel=True)
        q = solo.q0.copy()
        qdot = np.zeros((solo.model.nv, 1))
        dt = 1.0
        t_simu = 0.0

        qa_ref = solo.q0[7:]
        qa_dot_ref = np.zeros((8, 1))
        qa = q[7:]
        qa_dot = qdot[6:]

        torques = np.matrix([[3.], [3.], [-3.], [3.], [-3.], [-3.], [3.], [-3.]])

        for i in range(8):
            self.assertEqual(c_walking_IK(q, qdot, dt, solo, t_simu)[i, 0], torques[i, 0])


if __name__ == '__main__':
    unittest.main()
