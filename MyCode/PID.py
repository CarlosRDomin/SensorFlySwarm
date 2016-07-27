#!/usr/bin/python
#
# Original version downloaded from: https://github.com/ivmech/ivPID/blob/master/PID.py
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :PID.py
# description     :python pid controller
# author          :Caner Durmusoglu
# date            :20151218
# version         :0.1
# notes           :
# python_version  :2.7
# ==============================================================================

"""
	Modification of Ivmech's PID Controller (https://github.com/ivmech/ivPID) to fit my custom needs.
	File contains two classes:
	 - PID: Implements a basic PID loop, with added features like error bounding, output bounding, I_max,
		option for error sign inversion, option to treat the error in degrees (-180, 180]...
	 - PIDposAndVel: Implements a basic PIV loop (Proportional position loop + Integral and proportional Velocity loop).
		The idea is: a position PID reads current position and determines desired velocity to reach its set-point pos.
		Then, the output of that PID controller (desired velocity) is used as the set-point for the second PID loop,
		which looks at current velocity and acts upon it by providing a reaction acceleration.
"""
from datetime import datetime, timedelta

class PID:
	"""
		Simple PID Controller, with added features like error bounding, output bounding, I_max,
		option for error sign inversion, option to treat the error in degrees (-180, 180]...
	"""

	def __init__(self, P=0, I=0, D=0, set_point=0, offs=0, out_max=float('Inf'), I_max=float('Inf'), error_max=float('Inf'), invert_error=False, error_in_degrees=False, is_interval_const=False):
		self.clear()

		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.SetPoint = set_point
		self.out_offs = self.output = offs
		self.out_max = out_max
		self.I_max = I_max
		self.error_max = error_max
		self.invert_error = invert_error
		self.error_in_degrees = error_in_degrees
		self.curr_input = self.last_input = 0

		self.sample_time = 0.0
		self.curr_time = self.last_time = datetime.now()
		self.is_interval_const = is_interval_const

		self.PTerm = self.ITerm = self.DTerm = 0.0
		self.curr_error = self.last_error = 0.0
		self.clearing = True

	def clear(self):
		"""Clears PID computations and coefficients"""
		self.PTerm = 0.0
		self.ITerm = 0.0
		self.DTerm = 0.0
		self.clearing = True

		self.output = 0.0

	def update(self, feedback_value, t=None):
		"""Calculates PID value for given reference feedback

		.. math::
			u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

		.. figure:: images/pid_1.png
		   :align:   center

		   Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

		"""
		if t is None: t = datetime.now()

		self.last_time = self.curr_time
		self.last_input = self.curr_input
		self.last_error = self.curr_error
		self.curr_time = t
		self.curr_input = feedback_value
		self.curr_error = self.SetPoint - feedback_value

		if self.invert_error: self.curr_error = -self.curr_error
		if self.error_in_degrees:
			self.curr_error %= 360
			while self.curr_error < -180: self.curr_error += 360
			while self.curr_error > 180: self.curr_error -= 360  # Back to (-180, 180] range
		self.curr_error = min(self.error_max, max(-self.error_max, self.curr_error))

		if self.clearing:  # If the method clear() was just called, force delta_error=0 and delta_time=0 -> ITerm = DTerm = 0
			delta_time = 0
			delta_error = 0
			self.clearing = False
		else:
			if self.is_interval_const:
				self.curr_time = self.last_time + self.sample_time + timedelta(milliseconds=25)  # Add 25ms so delta_time is never 0 (not even if sample_time=0), so that D component is not always 0
			delta_time = (self.curr_time - self.last_time).total_seconds()
			delta_error = self.curr_error - self.last_error
		if delta_time < self.sample_time: return self.output

		self.PTerm = self.Kp * self.curr_error
		self.ITerm = min(self.I_max, max(-self.I_max, self.ITerm + self.curr_error*delta_time))
		self.DTerm = 0.0 if delta_time<=0 else delta_error/delta_time

		self.output = self.out_offs + min(self.out_max, max(-self.out_max, self.PTerm + (self.Ki*self.ITerm) + (self.Kd*self.DTerm)))
		return self.output

	def setKp(self, proportional_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
		self.Kp = proportional_gain

	def setKi(self, integral_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
		self.Ki = integral_gain

	def setKd(self, derivative_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
		self.Kd = derivative_gain

	def setSetPoint(self, set_point, clearPID=False):
		"""Sets the desired value we want the process value (PV) to hold"""
		self.SetPoint = set_point
		if (clearPID): self.clear()

	def setOffset(self, offset):
		"""Determines the desired output value that holds the process value (PV) stable at the SetPoint (ie, error=0)"""
		self.out_offs = offset

	def setOutMax(self, out_max):
		"""Sets the maximum (and minimum) value of the Manipulated Variable (MV or output).
		If lower_bound is bool, the minimum value is set to -upper_bound.
		Otherwise, the value of lower_bound is taken as in (eg, float)"""
		self.out_max = out_max

	def setImax(self, I_max):
		"""Integral windup, also known as integrator windup or reset windup,
		refers to the situation in a PID feedback controller where
		a large change in setpoint occurs (say a positive change)
		and the integral terms accumulates a significant error
		during the rise (windup), thus overshooting and continuing
		to increase as this accumulated error is unwound
		(offset by errors in the other direction).
		The specific problem is the excess overshooting.
		"""
		self.I_max = I_max

	def setErrorMax(self, error_max):
		"""Determines the maximum (absolute value) error considered (ie, any error>error_max will be clipped to error=error_max)"""
		self.error_max = error_max

	def setSampleTime(self, sample_time):
		"""PID that should be updated at a regular interval.
		Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
		"""
		self.sample_time = sample_time

class PIDposAndVel:
	"""
		Structure of 2 PID loops (PIV controller), where the output of one (position)
		is used as the set-point for the other (velocity)
	"""
	def __init__(self, posP=0, posI=0, posD=0, pos_set_point=0, pos_offs=0, pos_out_max=float('Inf'), pos_I_max=float('Inf'), pos_error_max=float('Inf'), pos_invert_error=False,
				 velP=0, velI=0, velD=0, vel_set_point=0, vel_offs=0, vel_out_max=float('Inf'), vel_I_max=float('Inf'), vel_error_max=float('Inf'), vel_invert_error=False, vel_invert_input=False, error_in_degrees=False, is_interval_const=False, max_vel=300):
		self.PIDpos = PID(posP, posI, posD, pos_set_point, pos_offs, pos_out_max, pos_I_max, pos_error_max, pos_invert_error, error_in_degrees, is_interval_const)
		self.PIDvel = PID(velP, velI, velD, vel_set_point, vel_offs, vel_out_max, vel_I_max, vel_error_max, vel_invert_error, error_in_degrees, is_interval_const)
		self.vel = 0
		self.invert_vel = vel_invert_input
		self.max_vel = max_vel

	def clear(self):
		self.PIDpos.clear()
		self.PIDvel.clear()

	def update(self, feedback_value, t=None):
		self.PIDpos.update(feedback_value, t)
		self.PIDvel.SetPoint = self.PIDpos.output
		self.PIDvel.update(self.getCurrVel(), t)

		return self.PIDvel.output

	def setSetPoint(self, set_point):
		self.PIDpos.SetPoint = set_point

	def getSetPoint(self):
		return self.PIDpos.SetPoint

	def getOutput(self):
		return self.PIDvel.output

	def getCurrPos(self):
		return self.PIDpos.curr_input

	def getLastPos(self):
		return self.PIDpos.last_input

	def getCurrVel(self):
		inv_factor = -1 if self.invert_vel else 1
		delta_time = (self.PIDpos.curr_time - self.PIDpos.last_time).total_seconds()

		if delta_time <= 0:
			self.vel = 0
		else:
			alpha = 0.5
			self.vel = (1-alpha)*self.vel + alpha*min(self.max_vel, max(-self.max_vel, inv_factor*(self.PIDpos.curr_input-self.PIDpos.last_input)/delta_time))

		return self.vel
