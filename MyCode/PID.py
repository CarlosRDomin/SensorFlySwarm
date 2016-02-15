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

"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller at Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """

    def __init__(self, P=1, I=1, D=0.001, offs=0, out_upper_bound=float('Inf'), out_lower_bound=False, invert_error=False, error_in_degrees=False, error_max=float('Inf'), I_max=20.0, is_interval_const=False):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.SetPoint = 0.0
        self.out_offs = offs
        self.setMaxMV(out_upper_bound, out_lower_bound)
        self.invert_error = invert_error
        self.error_in_degrees = error_in_degrees
        self.error_max = error_max
        self.I_max = I_max

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.is_interval_const = is_interval_const

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.clearing = True

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value
        if self.invert_error: error = -error
        if self.error_in_degrees:
            error %= 360
            if error > 180: error -= 360  # Back to (-180, 180] range
        if error < -self.error_max:
            error = -self.error_max
        elif error > self.error_max:
            error = self.error_max
        if self.clearing:  # If the method clear() was just called, force delta_error=0 and delta_time=0
            self.last_error = error
            self.last_time = time.time()
            self.clearing = False

        self.current_time = time.time()
        if self.is_interval_const:
            self.current_time = self.last_time + self.sample_time + 0.01  # Add 0.01 so delta_time is never 0 (not even if sample_time=0), so that D component is not always 0
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.I_max):
                self.ITerm = -self.I_max
            elif (self.ITerm > self.I_max):
                self.ITerm = self.I_max

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if self.output < self.out_min:
                self.output = self.out_min
            elif self.output > self.out_max:
                self.output = self.out_max
            self.output += self.out_offs

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setOffset(self, offset):
        """Determines the desired output value that holds the process value (PV) stable at the SetPoint (ie, error=0)"""
        self.out_offs = offset

    def setWindup(self, I_max):
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

    def setMaxMV(self, upper_bound, lower_bound=False):
        """Sets the maximum and minimum values of the Manipulated Variable (MV or output).
        If lower_bound is bool, the minimum value is set to -upper_bound.
        Otherwise, the value of lower_bound is taken as in (eg, float)"""
        self.out_max = upper_bound
        if isinstance(lower_bound, bool):
            self.out_min = -upper_bound
        else:
            self.out_min = lower_bound

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
