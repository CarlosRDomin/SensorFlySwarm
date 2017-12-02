"""
	Tools designed to facilitate logging (and plotting afterwards) all info related to a flight.
	It includes classes to log single values as well as PID (or PIV) loops (which includes setpoint, measured value,
	P/I/D terms, output value...) as a function of time. Also allows to save the logs and figures to a file.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import PID
from datetime import datetime, timedelta


class MagnitudeLog (object):
	"""
	This class logs a magnitude (single value) over time. It can then save the log to a file as well as plot the results.
	"""

	def __init__(self, experiment_start_datetime, str_magnitude=""):
		self.lst_timestamp = []  # List that keeps track of timestamp values at which data was collected
		self.lst_time_float = []  # List that keeps track of timestamp values at which data was collected, but in float
		self.lst_measured = []  # List that keeps track of the actual magnitude values over time
		self.str_magnitude = str_magnitude.replace('_', ' ')  # For plot titles, legends... replace "_" by " " for readability
		self.EXPERIMENT_START_DATETIME = experiment_start_datetime  # Datetime at which experiment/flight started
		self.LOG_FILENAME = self.update_log_filename()  # Filename that will be used when saving the log to a file
		self.LOG_FOLDER = "log/{}".format(experiment_start_datetime)  # Folder where the log will be saved to
		self.SYM_FOLDER = "log/{}".format(str_magnitude)  # Folder where a symlink to the log file will be created
		self.custom_plot = None  # If provided, it indicates a custom function responsible for plotting the data

	def clear(self):
		"""
		Reset (clear) logged magnitude.
		"""
		self.lst_timestamp = []
		self.lst_time_float = []
		self.lst_measured = []

	def create_log_dirs(self):
		if not os.path.exists(self.LOG_FOLDER):
			os.makedirs(self.LOG_FOLDER)
		if not os.path.exists(self.SYM_FOLDER):
			os.makedirs(self.SYM_FOLDER)

	def update_log_filename(self):
		self.LOG_FILENAME = "log_{}_{}".format(self.str_magnitude.replace(' ', '_'), self.EXPERIMENT_START_DATETIME.replace(':', '-').replace(' ', '_').replace('/', '_'))
		return self.LOG_FILENAME

	def get_log_filename(self, is_sym=False):
		return os.path.join(self.SYM_FOLDER if is_sym else self.LOG_FOLDER, self.LOG_FILENAME)

	def get_timestamp(self):
		return np.array(self.lst_timestamp)

	def get_time_float(self):
		return np.array(self.lst_time_float)

	def get_measured(self):
		return np.array(self.lst_measured)

	def update(self, measured, timestamp=None):
		"""
		Adds a new value to the log.
		:param measured: New value of the magnitude to log.
		:param timestamp: Time (None=now) at which "measured" was obtained.
		"""
		self.lst_measured.append(measured)
		if timestamp is None:
			self.lst_timestamp.append(datetime.now())
		elif isinstance(timestamp, timedelta):  # Can't plot a timedelta axis, so add today's date to convert them to datetime (so we can plot them)
			self.lst_timestamp.append(timestamp + datetime.now().replace(hour=0, minute=0, second=0, microsecond=0))
		else:
			self.lst_timestamp.append(timestamp)
		self.lst_time_float.append((self.lst_timestamp[-1] - self.lst_timestamp[0]).total_seconds())

	def plot(self):
		"""
		Plots the data logged by this magnitude over time.
		"""
		if len(self.lst_measured) < 1:
			return None
		if len(self.lst_timestamp) < 1:
			self.lst_timestamp = [datetime.now().replace(hour=0, minute=0, second=0, microsecond=0) + timedelta(seconds=i) for i in range(0, len(self.lst_measured))]
		plot_timestamp = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0) + np.array([(t-self.lst_timestamp[0]) for t in self.lst_timestamp])

		if self.custom_plot is not None:
			fig = self.custom_plot()
		else:
			fig, ax1 = plt.subplots()
			ax1.plot(plot_timestamp, self.get_measured(), 'b', label=self.str_magnitude, linewidth=2)
			ax1.set_xlim((min(plot_timestamp), max(plot_timestamp)))

			ax1.grid(True)
			ax1.set_title(self.str_magnitude, fontsize='x-large', fontweight='bold')
			ax1.set_xlabel('Time (s)', fontsize='large', fontweight='bold')
			ax1.set_ylabel(self.str_magnitude, fontsize='large', fontweight='bold')

			plt.ion()
			ax1.xaxis.set_major_formatter(mdates.DateFormatter('%M:%S'))
			fig.autofmt_xdate()  # Tilt x-ticks to be more readable
		figManager = plt.get_current_fig_manager()
		# figManager.window.showMaximized()
		fig.set_tight_layout(True)
		fig.show()

		self.create_log_dirs()  # Make sure LOG_FOLDER and SYM_FOLDER exist so saving the figure doesn't raise an exception
		fig.savefig("{}.pdf".format(self.get_log_filename()))
		if not os.path.exists("{}.pdf".format(self.get_log_filename(is_sym=True))):  # Create symlink (if not already created, would raise an exception in that case)
			os.symlink("{}.pdf".format(os.path.abspath(self.get_log_filename())), "{}.pdf".format(self.get_log_filename(is_sym=True)))  # Create a symlink to keep logs organized by date and also by magnitude (make sure symlink source path is absolute!!)

		return fig

	def shift_time_float(self, t0=None):
		if t0 is None or len(self.lst_timestamp) == 0:
			return  # Nothing to do
		if isinstance(t0, datetime):
			t0 = t0 - self.lst_timestamp[0]
		if isinstance(t0, timedelta):
			t0 = t0.total_seconds()

		for i in range(len(self.lst_time_float)):
			self.lst_time_float[i] = self.lst_time_float[i] - t0

	def save(self, t0=None):
		"""
		Saves log data to its corresponding *.npz file (determined by get_log_filename)
		"""
		self.create_log_dirs()  # Make sure LOG_FOLDER and SYM_FOLDER exist so saving the arrays doesn't raise an exception
		self.shift_time_float(t0)  # If necessary, shift lst_time_float (maybe other magnitudes started logging earlier than self.lst_timestamp[0])
		np.savez_compressed("{}.npz".format(self.get_log_filename()), t=self.get_timestamp(), tFloat=self.get_time_float(), measured=self.get_measured())
		if not os.path.exists("{}.npz".format(self.get_log_filename(is_sym=True))):  # Create symlink (if not already created, would raise an exception in that case)
			os.symlink("{}.npz".format(os.path.abspath(self.get_log_filename())), "{}.npz".format(self.get_log_filename(is_sym=True)))  # Create a symlink to keep logs organized by date and also by magnitude (make sure symlink source path is absolute!!)

	def load(self):
		"""
		Loads log data from its corresponding *.npz file (determined by get_log_filename)
		"""
		with np.load("{}.npz".format(self.get_log_filename())) as data:
			self.lst_timestamp = list(data["t"])
			self.load_time_float(data)
			self.lst_measured = list(data["measured"])

	def load_time_float(self, data):
		try:
			self.lst_time_float = list(data["tFloat"])
		except:
			self.lst_time_float = [(t - self.lst_timestamp[0]).total_seconds() for t in self.lst_timestamp] if len(self.lst_timestamp) > 0 else []


class PidLog (MagnitudeLog):
	"""
	This class logs a PID loop (setpoint, measured value, P/I/D terms, output value...) over time.
	It can then save the log to a file as well as plot the results.
	"""

	def __init__(self, experiment_start_datetime, str_magnitude="", pid_offset=0):
		super(PidLog, self).__init__(experiment_start_datetime, str_magnitude)
		self.lst_setpoint = []  # On top of the variables from MagnitudeLog, init the ones that are PID specific
		self.lst_out_P = []
		self.lst_out_I = []
		self.lst_out_D = []
		self.pid_offset = pid_offset

	def clear(self):
		"""
		Reset (clear) logged magnitude.
		"""
		super(PidLog, self).clear()
		self.lst_setpoint = []
		self.lst_out_P = []
		self.lst_out_I = []
		self.lst_out_D = []

	def get_setpoint(self):
		return np.array(self.lst_setpoint)

	def get_out_P(self):
		return np.array(self.lst_out_P)

	def get_out_I(self):
		return np.array(self.lst_out_I)

	def get_out_D(self):
		return np.array(self.lst_out_D)

	def update(self, pid, timestamp=None):
		"""
		Adds a new value to the log.
		:param pid: New value of the PID to log.
		:param timestamp: Time (None=pid.curr_time) at which "measured" was obtained.
		"""
		if timestamp is None: timestamp = pid.curr_time
		self.pid_offset = pid.out_offs
		if self.str_magnitude.lower().startswith("thrust"):
			self.update_long(-pid.SetPoint, -pid.curr_input, pid.PTerm, pid.Ki * pid.ITerm, pid.Kd * pid.DTerm, timestamp)
		elif pid.invert_error:
			self.update_long(pid.SetPoint, pid.curr_input, -pid.PTerm, -pid.Ki * pid.ITerm, -pid.Kd * pid.DTerm, timestamp)
		else:
			self.update_long(pid.SetPoint, pid.curr_input, pid.PTerm, pid.Ki * pid.ITerm, pid.Kd * pid.DTerm, timestamp)

	def update_long(self, setpoint, measured, out_P, out_I, out_D, timestamp=None):
		"""
		Adds a new value to the log.
		:param setpoint: PID's setpoint value to log.
		:param measured: PID's measured value to log.
		:param out_P: PID's output P term to log.
		:param out_I: PID's output I term to log.
		:param out_D: PID's output D term to log.
		:param timestamp: Time (None=now) at which the PID iteration was computed.
		"""
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

	def plot(self, plot_PID_terms=True):
		"""
		Plots the data logged by this PID over time.
		:param plot_PID_terms: If True, output P, I and D terms are plotted besides the measured and setpoint values.
		"""
		if len(self.lst_measured) < 1:
			return None
		if len(self.lst_timestamp) < 1:
			self.lst_timestamp = [datetime.now().replace(hour=0, minute=0, second=0, microsecond=0) + timedelta(seconds=i) for i in range(0, len(self.lst_measured))]
		plot_timestamp = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0) + np.array([(t-self.lst_timestamp[0]) for t in self.lst_timestamp])

		fig, ax1 = plt.subplots()
		ax1.plot(plot_timestamp, self.get_setpoint(), 'r--', label=self.str_magnitude + ' SetPoint', linewidth=1.5)
		ax1.plot(plot_timestamp, self.get_measured(), 'b', label=self.str_magnitude + ' measured', linewidth=2)
		ax1.set_xlim((min(plot_timestamp), max(plot_timestamp)))

		if plot_PID_terms:
			ax2 = ax1.twinx()
			ax2.plot(plot_timestamp, self.pid_offset + self.get_out_P()+self.get_out_I()+self.get_out_D(), 'g', label=self.str_magnitude + ' PID output', linewidth=2)
			ax2.plot(plot_timestamp, self.pid_offset + self.get_out_P(), 'm', label=self.str_magnitude + ' P component', linewidth=1)
			ax2.plot(plot_timestamp, self.pid_offset + self.get_out_I(), 'c', label=self.str_magnitude + ' I component', linewidth=1)
			ax2.plot(plot_timestamp, self.pid_offset + self.get_out_D(), 'k', label=self.str_magnitude + ' D component', linewidth=1)
			ax2.set_xlim(ax1.get_xlim())

			if not self.str_magnitude.lower().startswith("pZ"):
				ax1_ylim = ax1.get_ylim()
				ax2_ylim = ax2.get_ylim()
				ax1.set_ylim((min(ax1_ylim[0], ax2_ylim[0]), max(ax1_ylim[1], ax2_ylim[1])))
				ax2.set_ylim(ax1.get_ylim())

		ax1.grid(True)
		ax1.set_title(self.str_magnitude, fontsize='x-large', fontweight='bold')
		ax1.set_xlabel('Time (s)', fontsize='large', fontweight='bold')
		ax1.set_ylabel(self.str_magnitude + ' values', color='b', fontsize='large', fontweight='bold')
		ax1.legend(fontsize='medium', loc='lower left')
		for tl in ax1.get_yticklabels(): tl.set_color('b')
		if plot_PID_terms:
			ax2.set_ylabel(self.str_magnitude + ' PID output', color='g', fontsize='large', fontweight='bold')
			for tl in ax2.get_yticklabels(): tl.set_color('g')
			ax2.legend(fontsize='medium', loc='lower right')

		plt.ion()
		ax1.xaxis.set_major_formatter(mdates.DateFormatter('%M:%S'))
		fig.autofmt_xdate()  # Tilt x-ticks to be more readable
		figManager = plt.get_current_fig_manager()
		# figManager.window.showMaximized()
		fig.set_tight_layout(True)
		fig.show()

		self.create_log_dirs()  # Make sure LOG_FOLDER and SYM_FOLDER exist so saving the figure doesn't raise an exception
		fig.savefig("{}.pdf".format(self.get_log_filename()))
		if not os.path.exists("{}.pdf".format(self.get_log_filename(True))):  # Create symlink (if not already created, would raise an exception in that case)
			os.symlink("{}.pdf".format(os.path.abspath(self.get_log_filename())), "{}.pdf".format(self.get_log_filename(True)))  # Create a symlink to keep logs organized by date and also by magnitude (make sure symlink source path is absolute!!)

		return fig

	def save(self, t0=None):
		"""
		Saves log data to its corresponding *.npz file (determined by get_log_filename)
		"""
		self.create_log_dirs()  # Make sure LOG_FOLDER and SYM_FOLDER exist so saving the arrays doesn't raise an exception
		self.shift_time_float(t0)  # If necessary, shift lst_time_float (maybe other magnitudes started logging earlier than self.lst_timestamp[0])
		np.savez_compressed("{}.npz".format(self.get_log_filename()),
							t=self.get_timestamp(), tFloat=self.get_time_float(), setpoint=self.get_setpoint(), measured=self.get_measured(),
							out_P=self.get_out_P(), out_I=self.get_out_I(), out_D=self.get_out_D(), offset=self.pid_offset)
		if not os.path.exists("{}.npz".format(self.get_log_filename(True))):  # Create symlink (if not already created, would raise an exception in that case)
			os.symlink("{}.npz".format(os.path.abspath(self.get_log_filename())), "{}.npz".format(self.get_log_filename(True)))  # Create a symlink to keep logs organized by date and also by magnitude (make sure symlink source path is absolute!!)

	def load(self):
		"""
		Loads log data from its corresponding *.npz file (determined by get_log_filename)
		"""
		with np.load("{}.npz".format(self.get_log_filename())) as data:
			self.lst_timestamp = list(data["t"])
			self.load_time_float(data)
			self.lst_setpoint = list(data["setpoint"])
			self.lst_measured = list(data["measured"])
			self.lst_out_P = list(data["out_P"])
			self.lst_out_I = list(data["out_I"])
			self.lst_out_D = list(data["out_D"])
			self.pid_offset = data["offset"][()]  # offset was a number, so it becomes a 0-d array in numpy -> [()] is the only way to index it without getting an error =)


class ExperimentLog:
	"""
	This class aggregates all logs we want to keep track of for the current flight/experiment.
	It basically is an array of MagnitudeLog's and it has batch functions to call MagnitudeLog's methods on each element.
	"""

	def __init__(self, experiment_start_datetime, log_desc):
		self.EXPERIMENT_START_DATETIME = experiment_start_datetime
		self.magnitudes = {}  # Dictionary containing all magnitudes associated with the current experiment/flight
		for desc in log_desc.iteritems():  # Create and initialize MagnitudeLog's based on the log_desc
			self.add_magnitude(desc)

	def clear(self):
		"""
		Resets (clears) all magnitudes.
		"""
		for m in self.magnitudes.itervalues():
			m.clear()

	def update(self, **kwargs):
		"""
		Adds new values to the logs.
		:param kwargs: Set of key-value pairs indicating which magnitude and which corresponding value to update to.
		"""
		timestamp = kwargs.pop("timestamp", None)  # Use same timestamp for all updates (None by default)
		for key, value in kwargs.iteritems():  # For every item they want to update. Don't surround by try/except, we don't want experiments to accidentally not be recording some of the data
			key = key.lower()  # Case insensitive comparison to avoid problems
			if isinstance(value, PID.PIDposAndVel):  # If it's a PIV, update its 2 PidLog's separately
				self.magnitudes["{}_pos".format(key)].update(value.PIDpos, timestamp)
				self.magnitudes["{}_vel".format(key)].update(value.PIDvel, timestamp)
			else:
				self.magnitudes[key].update(value, timestamp)

	def plot(self, wait=True):
		"""
		Plots all data logged so far.
		:param wait: If True, don't return until a key is pressed (so user can look at the plots).
		"""
		plt.ion()
		for m in self.magnitudes.itervalues():
			m.plot()
		if wait:
			while not plt.waitforbuttonpress(timeout=5): pass  # Wait until a key is pressed (waitforbuttonpress will return True if a key was pressed, False if the mouse was clicked)

	def save(self):
		"""
		Saves all data logged so far to their corresponding *.npz files (determined by get_log_filename)
		"""
		t_min = None  # Find the earliest time a magnitude was logged (they may have started at different times, but we want them all to have the same time_float)
		for m in self.magnitudes.itervalues():
			if len(m.lst_timestamp) > 0 and (t_min is None or t_min > m.lst_timestamp[0]):  # Make sure m.lst_timestamp has at least one data point, otherwise it'll crash and not save
				t_min = m.lst_timestamp[0]  # New "minimum" t found, save it

		for m in self.magnitudes.itervalues():
			m.save(t_min)  # Pass the datetime of the earliest log, so time_float can shift the values if necessary

	def load(self):
		"""
		Loads data for all magnitudes from their corresponding *.npz files (determined by get_log_filename)
		"""
		for m in self.magnitudes.itervalues():
			m.load()

	def add_magnitude(self, log_desc, **kwargs):
		"""
		Adds a magnitude to the set of magnitudes we want to keep track of for this experiment/flight.
		:param log_desc: Description of the magnitude to be logged: key-value pair where the key represents the desired str_magnitude and
		the value is a string with either "piv" for double-PID (PIV) logging, "pid" for PID, or otherwise for a single-value magnitude.
		:param kwargs:
		:return:
		"""
		experiment_start_datetime = kwargs.pop("experiment_start_datetime", self.EXPERIMENT_START_DATETIME)  # Use default start datetime if not provided

		str_magnitude, log_type = log_desc
		log_type = log_type.lower()  # Case insensitive comparison to avoid problems
		if log_type == "piv":
			self.magnitudes["{}_pos".format(str_magnitude.lower())] = PidLog(experiment_start_datetime, "{}_pos".format(str_magnitude))
			self.magnitudes["{}_vel".format(str_magnitude.lower())] = PidLog(experiment_start_datetime, "{}_vel".format(str_magnitude))
		elif log_type == "pid":
			self.magnitudes[str_magnitude.lower()] = PidLog(experiment_start_datetime, str_magnitude)
		else:
			self.magnitudes[str_magnitude.lower()] = MagnitudeLog(experiment_start_datetime, str_magnitude)

		for key, value in kwargs.iteritems():
			setattr(self.magnitudes[str_magnitude.lower()], key, value)


class DerivativeHelper:
	"""
	This class contains helper functions to calculate a filtered derivative (e.g., velocity and acceleration from position).
	Based on this answer http://dsp.stackexchange.com/questions/9498/have-position-want-to-calculate-velocity-and-acceleration
	"""

	@staticmethod
	def sg_filter(x, m, k=0):
		"""
		Savitzky-Golay Filter (http://dsp.stackexchange.com/questions/1676/savitzky-golay-smoothing-filter-for-not-equally-spaced-data/9494#9494)
		:param x: Vector of sample times
		:param m: Order of the smoothing polynomial
		:param k: Which derivative
		:return: Vector (np.array) containing the corresponding Savitzky-Golay Filter
		"""
		mid = len(x)/2
		if isinstance(x[0], datetime):  # Errors will be raised when working with timedeltas, so in case of datetime input, convert timedelta to float
			a = np.array([(t - x[mid]).total_seconds() for t in x])
		else:
			a = np.array(x) - x[mid]
		expa = lambda x: map(lambda i: i ** x, a)
		A = np.r_[map(expa, range(0, m + 1))].transpose()
		Ai = np.linalg.pinv(A)

		return Ai[k]

	@staticmethod
	def differentiate(signal, t, win_half_size=5, poly_order=2, deriv=0):
		"""
		Calculates a filtered derivative (e.g., velocity and acceleration from position). Crops input signal if necessary (no need to pass a segment of length=win_half_size).
		:param signal: Signal to differentiate
		:param t: Times at which signal was sampled
		:param win_half_size: Number of points for the smoothing window (window will have 2*win_half_size + 1 points)
		:param poly_order: Order of the smoothing polynomial
		:param deriv: Order of the derivative to compute (e.g., from pos: deriv=1 is vel, deriv=2 is accel...)
		:return: Estimation of d(signal)/dt at the center of the smoothing window (so at t[-win_half_size])
		"""

		if deriv > poly_order: raise Exception("deriv must be <= poly_order!")

		start_ind = max(0, len(signal) - (2 * win_half_size + 1))  # Get last 2*win_half_size+1 points of the singal (or the whole signal if len(signal) is shorter)
		f = DerivativeHelper.sg_filter(t[start_ind:], poly_order, deriv)  # Compute the Savitzky-Golay Filter
		result = np.dot(f, signal[start_ind:])  # And apply the filter to the (windowed) signal

		if deriv > 1:  # Finally, multiply by the corresponding coefficient
			import math
			result *= math.factorial(deriv)

		return result

	@staticmethod
	def differentiate_whole(signal, t, win_half_size=5, poly_order=2, deriv=0):
		"""
		Calculates a filtered derivative (e.g., velocity and acceleration from position). Takes a moving window of the signal and calls differentiate() accordingly.
		:param signal: Signal to differentiate
		:param t: Times at which signal was sampled
		:param win_half_size: Number of points for the smoothing window (window will have 2*win_half_size + 1 points)
		:param poly_order: Order of the smoothing polynomial
		:param deriv: Order of the derivative to compute (e.g., from pos: deriv=1 is vel, deriv=2 is accel...)
		:return: Estimation of d(signal)/dt at every point/instant of the signal
		"""
		n = len(t)
		result = np.zeros(n)  # Initialize the output
		for i in xrange(win_half_size, n-win_half_size):  # Take a moving window
			start, end = i - win_half_size, i + win_half_size + 1  # Find the indices for the window interval
			result[i] = DerivativeHelper.differentiate(signal[start:end], t[start:end], win_half_size, poly_order, deriv)  # Compute the derivative at that point

		return result


if __name__ == '__main__':
	radio_ch = 80
	experiment_start_datetime = "2017-02-05 11-37-15"
	experiment_log = ExperimentLog("{}/{}".format(radio_ch, experiment_start_datetime), {"aX_raw": "mag", "aY_raw": "mag", "aZ_raw": "mag", "aX_world": "mag", "aY_world": "mag", "aZ_world": "mag", "pX_cam": "mag", "pY_cam": "mag", "pZ_cam": "mag", "vX_cam": "mag", "vY_cam": "mag", "vZ_cam": "mag", "aX_cam": "mag", "aY_cam": "mag", "aZ_cam": "mag"})
	# experiment_log = ExperimentLog("{}/{}".format(radio_ch, experiment_start_datetime), {"vX_cam": "mag", "vY_cam": "mag", "vZ_cam": "mag", "aX_cam": "mag", "aY_cam": "mag", "aZ_cam": "mag"})
	experiment_log.load()

	# t = experiment_log.magnitudes["px_cam"].lst_timestamp
	# win_half_size = 11; poly_order = 2;
	# for axis in ("X", "Y", "Z"):
	# 	experiment_log.add_magnitude(("v{}_cam".format(axis), "mag"), lst_timestamp=t, lst_measured=DerivativeHelper.differentiate_whole(experiment_log.magnitudes["p{}_cam".format(axis.lower())].lst_measured, t, poly_order=poly_order, win_half_size=win_half_size, deriv=1))
	# 	experiment_log.add_magnitude(("a{}_cam".format(axis), "mag"), lst_timestamp=t, lst_measured=DerivativeHelper.differentiate_whole(experiment_log.magnitudes["p{}_cam".format(axis.lower())].lst_measured, t, poly_order=poly_order, win_half_size=win_half_size, deriv=2))
	# 	plt.figure()
	# 	plt.subplot(3,1,1); plt.plot(t, experiment_log.magnitudes["p{}_cam".format(axis.lower())].lst_measured)
	# 	plt.subplot(3,1,2); plt.plot(t, experiment_log.magnitudes["v{}_cam".format(axis.lower())].lst_measured)
	# 	plt.subplot(3,1,3); plt.plot(t, experiment_log.magnitudes["a{}_cam".format(axis.lower())].lst_measured)
	# experiment_log.save()
	# exit()

	# t_min = None
	# for win_size in experiment_log.magnitudes.itervalues():
	# 	if t_min is None or t_min > win_size.lst_timestamp[0]:
	# 		t_min = win_size.lst_timestamp[0]
	# for win_size in experiment_log.magnitudes.itervalues():
	# 	win_size.lst_time_float = [(t - t_min).total_seconds() for t in win_size.lst_timestamp]
	# for m in experiment_log.magnitudes.itervalues():
	# 	m.lst_time_float = m.lst_timestamp
	experiment_log.save()
	exit()


	from random import random

	log = MagnitudeLog(str(datetime.now())[:-7], "Roll")
	for x in range(0, 100):
		log.update(2*random()-1)
	log.plot()
	log.save()
	log2 = MagnitudeLog(log.EXPERIMENT_START_DATETIME, log.str_magnitude)
	log2.load()
	log2.plot()
	log2.save()
	print "Save & Load procedures work for MagnitudeLog! :)" if (log.lst_timestamp == log2.lst_timestamp) and (log.lst_measured == log2.lst_measured) else "Ooops, Save & Load for MagnitudeLog still needs some more work... :("

	log = PidLog(str(datetime.now())[:-7], "Roll")
	for x in range(0, 100):
		log.update_long(5, 5-random(), 10*random()-2, 3*random()-1.5, random())
	log.plot()
	log.save()
	log2 = PidLog(log.EXPERIMENT_START_DATETIME, log.str_magnitude)
	log2.load()
	log2.plot()
	log2.save()
	print "Save & Load procedures work for PidLog! :)" if (log.lst_timestamp == log2.lst_timestamp) and (log.lst_measured == log2.lst_measured) and (log.lst_out_P == log2.lst_out_P) and (log.pid_offset == log2.pid_offset) else "Ooops, Save & Load for PidLog still needs some more work... :("
	plt.waitforbuttonpress()
