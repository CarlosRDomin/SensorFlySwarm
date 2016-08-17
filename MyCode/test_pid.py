"""
	Simple script to test/debug our PID class implemented in PID.py
"""

import PID
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

def test_pid(P = 0.2,  I = 0.0, D= 0.0, L=100):
	"""
	Core function to test/debug our custom PID class
	"""
	pid = PID.PID(P, I, D, is_interval_const=False)
	pid.setOutMax(0.5)

	pid.SetPoint=0.0
	pid.setSampleTime(0.01)

	END = L
	feedback_list = []
	time_list = []
	setpoint_list = np.hstack((630*np.ones(10), 640*np.ones(L-10)))
	feedback = setpoint_list[0]

	for i in range(1, END+1):
		pid.update(feedback)
		output = pid.output
		# if pid.SetPoint > 0:
			# feedback += (output - (1/i))
		feedback += output
		if i==50: feedback = 200
		pid.SetPoint = setpoint_list[i-1]
		time.sleep(0.02)

		feedback_list.append(feedback)
		# setpoint_list.append(pid.SetPoint)
		time_list.append(i)

	time_sm = np.array(time_list)
	time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
	feedback_smooth = spline(time_list, feedback_list, time_smooth)

	# plt.interactive(True)
	plt.plot(time_smooth, feedback_smooth)
	plt.plot(time_list, setpoint_list)
	plt.xlim((0, L))
	plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
	plt.xlabel('time (s)')
	plt.ylabel('PID (PV)')
	plt.title('TEST PID')

	# plt.ylim((1-0.5, 1+0.5))

	plt.grid(True)
	plt.interactive(True)
	plt.show()
	if plt.isinteractive(): plt.waitforbuttonpress()

if __name__ == "__main__":
	# test_pid(1.2, 3, 0.001, L=150)
	test_pid(4, 1, 0.001, L=150)
#    test_pid(0.8, L=50)