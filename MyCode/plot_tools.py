"""
	Tools designed to facilitate logging (and plotting afterwards) all info related to a PID (or PIV) loop:
	setpoint, measured value, P/I/D terms, output value... as a function of time.
	Also allows to save the figure to a file.
"""

from datetime import datetime, timedelta
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

class MagnitudeControlLog:
	def __init__(self, str_magnitude="", pid_offset=0):
		self.lst_timestamp = []
		self.lst_setpoint = []
		self.lst_measured = []
		self.lst_out_P = []
		self.lst_out_I = []
		self.lst_out_D = []
		self.pid_offset = pid_offset
		self.str_magnitude = str_magnitude

	def clear(self):
		self.lst_timestamp = []
		self.lst_setpoint = []
		self.lst_measured = []
		self.lst_out_P = []
		self.lst_out_I = []
		self.lst_out_D = []

	def get_timestamp(self):
		return np.array(self.lst_timestamp)

	def get_setpoint(self):
		return np.array(self.lst_setpoint)

	def get_measured(self):
		return np.array(self.lst_measured)

	def get_out_P(self):
		return np.array(self.lst_out_P)

	def get_out_I(self):
		return np.array(self.lst_out_I)

	def get_out_D(self):
		return np.array(self.lst_out_D)

	def update(self, pid, timestamp=None):
		if timestamp is None: timestamp = pid.curr_time
		self.pid_offset = pid.out_offs
		if self.str_magnitude.lower().startswith("thrust"):
			self.update_long(-pid.SetPoint, -pid.curr_input, pid.PTerm, pid.Ki * pid.ITerm, pid.Kd * pid.DTerm, timestamp)
		elif pid.invert_error:
			self.update_long(pid.SetPoint, pid.curr_input, -pid.PTerm, -pid.Ki * pid.ITerm, -pid.Kd * pid.DTerm, timestamp)
		else:
			self.update_long(pid.SetPoint, pid.curr_input, pid.PTerm, pid.Ki * pid.ITerm, pid.Kd * pid.DTerm, timestamp)

	def update_long(self, setpoint, measured, out_P, out_I, out_D, timestamp=None):
		self.lst_setpoint.append(setpoint)
		self.lst_measured.append(measured)
		self.lst_out_P.append(out_P)
		self.lst_out_I.append(out_I)
		self.lst_out_D.append(out_D)
		if timestamp is None:
			self.lst_timestamp.append(datetime.now())
		elif isinstance(timestamp, timedelta):  # Can't plot a timedelta axis, so add today's date to convert them to datetime (so we can plot them)
			self.lst_timestamp.append(timestamp + datetime.now().replace(hour=0, minute=0, second=0, microsecond=0))
		else:
			self.lst_timestamp.append(timestamp)

	def plot(self):
		if len(self.lst_measured) < 1:
			return None
		if len(self.lst_timestamp) < 1:
			self.lst_timestamp = [timedelta(seconds=i) for i in range(0, len(self.lst_measured))]

		fig, ax1 = plt.subplots()
		ax1.plot(self.get_timestamp(), self.get_setpoint(), 'r--', label=self.str_magnitude + ' SetPoint', linewidth=1.5)
		ax1.plot(self.get_timestamp(), self.get_measured(), 'b', label=self.str_magnitude + ' measured', linewidth=2)
		ax1.set_xlim((min(self.get_timestamp()), max(self.get_timestamp())))

		ax2 = ax1.twinx()
		ax2.plot(self.get_timestamp(), self.pid_offset + self.get_out_P()+self.get_out_I()+self.get_out_D(), 'g', label=self.str_magnitude + ' PID output', linewidth=2)
		ax2.plot(self.get_timestamp(), self.pid_offset + self.get_out_P(), 'm', label=self.str_magnitude + ' P component', linewidth=1)
		ax2.plot(self.get_timestamp(), self.pid_offset + self.get_out_I(), 'c', label=self.str_magnitude + ' I component', linewidth=1)
		ax2.plot(self.get_timestamp(), self.pid_offset + self.get_out_D(), 'k', label=self.str_magnitude + ' D component', linewidth=1)
		ax2.set_xlim(ax1.get_xlim())

		if not self.str_magnitude.lower().startswith("thrust"):
			ax1_ylim = ax1.get_ylim()
			ax2_ylim = ax2.get_ylim()
			ax1.set_ylim((min(ax1_ylim[0], ax2_ylim[0]), max(ax1_ylim[1], ax2_ylim[1])))
			ax2.set_ylim(ax1.get_ylim())

		ax1.grid(True)
		ax1.set_title(self.str_magnitude, fontsize='x-large', fontweight='bold')
		ax1.set_xlabel('Time (s)', fontsize='large', fontweight='bold')
		ax1.set_ylabel(self.str_magnitude + ' values', color='b', fontsize='large', fontweight='bold')
		for tl in ax1.get_yticklabels(): tl.set_color('b')
		ax2.set_ylabel(self.str_magnitude + ' PID output', color='g', fontsize='large', fontweight='bold')
		for tl in ax2.get_yticklabels(): tl.set_color('g')
		ax1.legend(fontsize='medium', loc='lower left')
		ax2.legend(fontsize='medium', loc='lower right')

		plt.ion()
		ax1.xaxis.set_major_formatter(mdates.DateFormatter('%M:%S'))
		fig.autofmt_xdate()  # Tilt x-ticks to be more readable
		figManager = plt.get_current_fig_manager()
		figManager.window.showMaximized()
		fig.set_tight_layout(True)
		fig.show()
		fig.savefig('log/log_' + self.str_magnitude + '_' + datetime.now().strftime("%m-%d_%H-%M-%S") + '.pdf')

		return fig


class OverallControlLog:
	def __init__(self):
		self.roll_pos = MagnitudeControlLog("Roll_pos")
		self.roll_vel = MagnitudeControlLog("Roll_vel")
		self.pitch_pos = MagnitudeControlLog("Pitch_pos")
		self.pitch_vel = MagnitudeControlLog("Pitch_vel")
		self.yaw = MagnitudeControlLog("Yaw")
		self.thrust_pos = MagnitudeControlLog("Thrust_pos")
		self.thrust_vel = MagnitudeControlLog("Thrust_vel")

	def clear(self):
		self.roll_pos.clear()
		self.roll_vel.clear()
		self.pitch_pos.clear()
		self.pitch_vel.clear()
		self.yaw.clear()
		self.thrust_pos.clear()
		self.thrust_vel.clear()

	def update(self, roll_PID, pitch_PID, yaw_PID, thrust_PID, timestamp=None):
		if roll_PID is not None:
			self.roll_pos.update(roll_PID.PIDpos, timestamp)
			self.roll_vel.update(roll_PID.PIDvel, timestamp)
		if pitch_PID is not None:
			self.pitch_pos.update(pitch_PID.PIDpos, timestamp)
			self.pitch_vel.update(pitch_PID.PIDvel, timestamp)
		if yaw_PID is not None:
			self.yaw.update(yaw_PID, timestamp)
		if thrust_PID is not None:
			self.thrust_pos.update(thrust_PID.PIDpos, timestamp)
			self.thrust_vel.update(thrust_PID.PIDvel, timestamp)

	def update_long(self, roll_pos_setpoint, roll_pos_measured, roll_pos_out_P, roll_pos_out_I, roll_pos_out_D,
					roll_vel_setpoint, roll_vel_measured, roll_vel_out_P, roll_vel_out_I, roll_vel_out_D,
					pitch_pos_setpoint, pitch_pos_measured, pitch_pos_out_P, pitch_pos_out_I, pitch_pos_out_D,
					pitch_vel_setpoint, pitch_vel_measured, pitch_vel_out_P, pitch_vel_out_I, pitch_vel_out_D,
					yaw_setpoint, yaw_measured, yaw_out_P, yaw_out_I, yaw_out_D,
					thrust_pos_setpoint, thrust_pos_measured, thrust_pos_out_P, thrust_pos_out_I, thrust_pos_out_D,
					thrust_vel_setpoint, thrust_vel_measured, thrust_vel_out_P, thrust_vel_out_I, thrust_vel_out_D, timestamp=None):
		self.roll_pos.update_long(roll_pos_setpoint, roll_pos_measured, roll_pos_out_P, roll_pos_out_I, roll_pos_out_D, timestamp)
		self.roll_vel.update_long(roll_vel_setpoint, roll_vel_measured, roll_vel_out_P, roll_vel_out_I, roll_vel_out_D, timestamp)
		self.pitch_pos.update_long(pitch_pos_setpoint, pitch_pos_measured, pitch_pos_out_P, pitch_pos_out_I, pitch_pos_out_D, timestamp)
		self.pitch_vel.update_long(pitch_vel_setpoint, pitch_vel_measured, pitch_vel_out_P, pitch_vel_out_I, pitch_vel_out_D, timestamp)
		self.yaw.update_long(yaw_setpoint, yaw_measured, yaw_out_P, yaw_out_I, yaw_out_D, timestamp)
		self.thrust_pos.update_long(thrust_pos_setpoint, thrust_pos_measured, thrust_pos_out_P, thrust_pos_out_I, thrust_pos_out_D, timestamp)
		self.thrust_vel.update_long(thrust_vel_setpoint, thrust_vel_measured, thrust_vel_out_P, thrust_vel_out_I, thrust_vel_out_D, timestamp)

	def plot(self, wait=True):
		plt.ion()
		self.roll_pos.plot()
		self.roll_vel.plot()
		self.pitch_pos.plot()
		self.pitch_vel.plot()
		self.yaw.plot()
		self.thrust_pos.plot()
		self.thrust_vel.plot()
		if wait:
			while not plt.waitforbuttonpress(timeout=5): pass  # Wait until a key is pressed (waitforbuttonpress will return True if a key was pressed, False if the mouse was clicked)


if __name__ == '__main__':
	from random import random
	log = MagnitudeControlLog("Roll")
	for x in range(0, 100):
		log.update_long(5, 5-random(), 10*random()-2, 3*random()-1.5, random())
	log.plot()
	plt.waitforbuttonpress()
