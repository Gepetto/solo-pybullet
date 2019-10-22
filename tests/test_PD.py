# coding: utf8

import unittest
import numpy as np
from PD import PD

class TestPD(unittest.TestCase):
	def test_PD(self):
		
		qa_ref = np.zeros((8,1))
		qa_ref[0] = 1.
		qa_dot_ref = np.zeros((8,1))
		qa = np.zeros((8,1))
		qa_dot = np.zeros((8,1))
		dt = 0
		
		torques = np.zeros((8,1))
		torques[0] = 1.
		
		for i in range(8):
			self.assertEqual(PD(qa_ref, qa_dot_ref, qa, qa_dot, dt)[i], torques[i])
		
		
if __name__ == '__main__':
	unittest.main()
