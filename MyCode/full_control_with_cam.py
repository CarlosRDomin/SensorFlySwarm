"""
	Main code that implements full control (X, Y, Z , yaw) of a Crazyflie using a camera through OpenCV.
	The algorithm basically does the following:
	 - Fly at a set constant thrust to take off, until a key ("e") is pressed.
		When that happens, current position is used as target position to hold
	 - Using the camera, we find the x,y coordinates of the drone, and estimate z based on size of a known-size object
	 - Independent PIV loops (position + velocity PIDs) control movement in each direction (x, y, z)
	 - ASDWUH keys can modify the target position to hold, any other key will kill the algorithm and stop the drone.
		When that happens, all the sensor and control data that's been logged will be plotted and stored in files
	 - NOTE: PID constants could still be further tuned to reduce overshoot
"""

import logging
import time
from datetime import datetime, timedelta
import shutil
import os
import cv2
import numpy as np
import sdl2
import sdl2.ext
import struct

from uvc_capture import UvcCapture
from operator import attrgetter
import PID
import plot_tools

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.param import ParamTocElement, WRITE_CHANNEL
from cflib import crtp
from cflib.crtp.crtpstack import CRTPPort, CRTPPacket


class DroneController:
	COLOR_BALL_TRACKED = (255, 0, 0)
	COLOR_BALL_UNTRACKED = (0, 0, 255)
	COLOR_LINE_TRACKED = (255, 0, 0)
	COLOR_LINE_UNTRACKED = (0, 0, 255)
	COLOR_TARGET_TRACKED = (0, 255, 0)
	COLOR_TARGET_UNTRACKED = (0, 0, 255)
	FIGURE_NAME = "Output"
	SETTINGS_ENVIRONMENT = "Very bright bedroom"
	CAMERA_SETTINGS_FILE = "config/cam_settings/Camera settings - USB 2.0 Camera - {}.txt".format(SETTINGS_ENVIRONMENT)
	COLOR_THRESH_SETTINGS_FILE = "config/color_thresh/Color threshold settings - {}.txt".format(SETTINGS_ENVIRONMENT)
	BLOB_DETECTOR_SETTINGS_FILE = "config/blob_detector/Blob detector settings.txt"
	SETTINGS_SEPARATOR = UvcCapture.SETTINGS_SEPARATOR  # We save files in a csv type of way
	ASK_FOR_TARGET_YAW = False
	TAKEOFF_THRUST = 44000

	def __init__(self):
		self.t_start = self.t_frame = self.t_last_frame = datetime.now()
		self.t_events = []
		self.EXPERIMENT_START_DATETIME = str(self.t_start)[:-7].replace(':', '-')
		self.VIDEO_FOLDER = "img-ns/{}".format(self.EXPERIMENT_START_DATETIME)
		self.experiment_log = plot_tools.ExperimentLog(self.EXPERIMENT_START_DATETIME, {"Roll": "piv", "Pitch": "piv", "Yaw": "pid", "Thrust": "piv", "Estimated_Z": "log", "Velocity_Z": "log"})
		self.window_for_kb_input = None
		self.video_capture = None
		self.cv_HSV_thresh_min = np.array([  0,   0,   0], dtype=np.uint8)
		self.cv_HSV_thresh_max = np.array([255, 255, 255], dtype=np.uint8)
		self.cv_blob_detect_params = None
		self.cv_cam_frame = None
		self.cv_filtered_HSV_mask = None
		self.cv_frame_out = None
		self.crazyflie = None
		self.cf_log = None
		self.cf_radio_connecting = True
		self.cf_radio_connected = False
		self.cf_ignore_camera = False
		self.cf_pos_tracked = False
		self.cf_taking_off = True
		self.cf_str_status = "TAKING OFF"
		self.cf_roll = self.cf_pitch = self.cf_yaw = self.cf_estimated_z = self.cf_vel_z = 0
		self.cf_curr_pos = np.array([0, 0, 0])
		self.cf_PID_roll = PID.PIDposAndVel(posP=0.5, velP=0.05, velI=0.01, vel_offs=0, pos_out_max=300, vel_out_max=30, vel_invert_error=True)
		self.cf_PID_pitch = PID.PIDposAndVel(posP=0.7, velP=0.3, velI=0.002, vel_offs=0, pos_out_max=30, vel_out_max=30)
		self.cf_PID_yaw = PID.PID(P=0.5, I=0.3, D=0, offs=0, out_max=20, invert_error=True, error_in_degrees=True)
		self.cf_PID_thrust = PID.PIDposAndVel(posP=1, velP=35, velI=25, vel_offs=43000, pos_out_max=300, vel_out_max=7000, pos_invert_error=True, vel_invert_input=True)

	def init_video_cam_and_cv_algorithm(self, create_video_folder=True):
		"""
		Initializes camera: connects to it, loads settings from config file (if available),
		loads color threshold and blob detector settings (if available), creates a folder to save cv output images...
		:param create_video_folder: True to create folder specified by VIDEO_FOLDER (where output frames will be saved)
		"""
		self.video_capture = UvcCapture.new_from_settings(self.CAMERA_SETTINGS_FILE)  # Connect to device specified by settings, and load its desired param values
		if self.video_capture is None:  # If unable to connect to device specified by settings, open first available camera
			self.video_capture = UvcCapture(0)
			if self.video_capture is None:  # If still unable to connect, raise an exception
				raise Exception("Couldn't open camera! :(")

			# If we're here, we couldn't connect to device specified by settings but were able to open 1st available cam
			if not self.video_capture.load_settings(self.CAMERA_SETTINGS_FILE):  # Try to load frame size & rate from settings
				self.video_capture.select_best_frame_mode(60)  # If loading settings failed, choose best frame size with fps >= 60
		logging.info("Camera opened! :)")

		# Initialize cv algorithm too: load settings for color thresholding and blob detector
		self.load_color_thresh_settings()
		self.load_blob_detector_settings()

		# Sometimes, first couple frames take a long time to be obtained, do it before quad goes in flying mode
		self.video_capture.get_frame_robust()
		self.video_capture.get_frame_robust()

		# Initialize PID setpoints and initial input value to the center of the frame
		self.cf_PID_roll.setSetPoint(self.video_capture.frame_size[0] / 2)
		self.cf_PID_thrust.setSetPoint(self.video_capture.frame_size[1] / 2)
		self.cf_PID_pitch.setSetPoint(40)
		self.cf_PID_roll.PIDpos.curr_input = self.cf_PID_roll.getSetPoint()
		self.cf_PID_thrust.PIDpos.curr_input = self.cf_PID_thrust.getSetPoint()
		self.cf_PID_pitch.PIDpos.curr_input = self.cf_PID_pitch.getSetPoint()

		# Prepare the folder self.VIDEO_FOLDER so we can store each frame we processed (for debugging)
		if create_video_folder:
			shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=True)  # Delete the folder and its contents, if it exists (ignore errors if it doesn't)
			os.makedirs(self.VIDEO_FOLDER)  # Now create the folder, which won't throw any exceptions as we made sure it didn't already exist

	def init_UI_window(self):
		"""
		Initializes the appropriate user interface depending on experiment's purpose (see is_input_for_drone_commands)
		:param is_input_for_drone_commands: True if we have communication with the CF and need input to send it commands
		False if we're just debugging the computer vision side (camera settings, color threshold, etc.) and need input
		to debug pixel information
		"""
		# Open an SDL2 window to track keyboard keydown events and use it to send commands to the CF
		sdl2.ext.init()
		self.window_for_kb_input = sdl2.ext.Window("Window to receive keyboard input", size=(400, 300))
		self.window_for_kb_input.show()

	def save_color_thresh_settings(self, HSV_min, HSV_max, file_name=COLOR_THRESH_SETTINGS_FILE, sep=SETTINGS_SEPARATOR):
		"""
		Saves specified color threshold settings to a file.
		:param HSV_min: np.array (1x3) containing minimum H, S and V values for the color thresholding CF detection
		:param HSV_max: np.array (1x3) containing maximum H, S and V values for the color thresholding CF detection
		:param file_name: Name of the file to use when saving the settings
		:param sep: String that will be used to separate parameters (in a csv fashion). Eg: '\t', ',', etc.
		:return: True if everything went well; False if settings couldn't be saved
		"""
		try:
			logging.debug("Saving current color threshold settings to file '{}'".format(file_name))
			with open(file_name, 'w') as f:  # Open file for output
				f.write("{}\n".format(sep.join(HSV_min.astype(str))))  # Store HSV_min
				f.write("{}\n".format(sep.join(HSV_max.astype(str))))  # Store HSV_max
		except:
			logging.exception("Something went wrong while saving current color threshold settings to '{}'.".format(file_name))
			return False
		return True

	def load_color_thresh_settings(self, file_name=COLOR_THRESH_SETTINGS_FILE, sep=SETTINGS_SEPARATOR):
		"""
		Loads color threshold settings from a file.
		:param file_name: Path to the file to load the settings from
		:param sep: String that was used to separate parameters (in a csv fashion). Eg: '\t', ',', etc.
		:return: True if everything went well; False if settings couldn't be loaded
		"""
		try:
			logging.debug("Loading color threshold settings from file '{}'".format(file_name))
			with open(file_name, 'r') as f:  # Open file for input
				self.cv_HSV_thresh_min = np.array(f.readline().rstrip('\r\n').split(sep), dtype=np.uint8)
				self.cv_HSV_thresh_max = np.array(f.readline().rstrip('\r\n').split(sep), dtype=np.uint8)
				logging.debug("\tLoaded color threshold settings: HSV_min={}; HSV_max={}".format(self.cv_HSV_thresh_min, self.cv_HSV_thresh_max))
		except:
			logging.exception("Something went wrong while loading color threshold settings from '{}'.".format(file_name))
			return False
		return True

	def save_blob_detector_settings(self, detector_params, file_name=BLOB_DETECTOR_SETTINGS_FILE, sep=SETTINGS_SEPARATOR):
		"""
		Saves specified blob detector settings to a file.
		:param detector_params: cv2.SimpleBlobDetector_Params object containing the params which want to be saved
		:param file_name: Name of the file to use when saving the settings
		:param sep: String that will be used to separate parameters (in a csv fashion). Eg: '\t', ',', etc.
		:return: True if everything went well; False if settings couldn't be saved
		"""
		try:
			logging.debug("Saving current blob detector settings to file '{}'".format(file_name))
			with open(file_name, 'w') as f:  # Open file for output
				for m in dir(detector_params):  # Traverse all methods and properties of detector_params
					if m[0] != '_':  # For every property that's not "built-in" (ie: doesn't start by '_')
						f.write("{}{}{}\n".format(m, sep, getattr(detector_params, m)))  # Store the property name and value
		except:
			logging.exception("Something went wrong while saving current blob detector settings to '{}'.".format(file_name))
			return False
		return True

	def load_blob_detector_settings(self, file_name=BLOB_DETECTOR_SETTINGS_FILE, sep=SETTINGS_SEPARATOR):
		"""
		Loads blob detector settings from a file. Leave file_name empty to only load default params.
		cv2 will use these params to detect the drone from a binary image mask (the output of the color thresholding step).
		:param file_name: Path to the file to load the settings from
		:param sep: String that was used to separate parameters (in a csv fashion). Eg: '\t', ',', etc.
		:return: True if everything went well; False if settings couldn't be loaded
		"""
		detector_params = cv2.SimpleBlobDetector_Params()
		self.cv_blob_detect_params = detector_params

		# Filter by color
		detector_params.filterByColor = False
		detector_params.blobColor = 255

		# Change thresholds
		detector_params.minThreshold = 254
		detector_params.maxThreshold = 255

		# Filter by Area.
		detector_params.filterByArea = True
		detector_params.minArea = 30
		detector_params.maxArea = 40000

		# Filter by Circularity
		detector_params.filterByCircularity = False
		detector_params.minCircularity = 0.7
		detector_params.maxCircularity = 1.0

		# Filter by Convexity
		detector_params.filterByConvexity = False
		detector_params.minConvexity = 0.7
		detector_params.maxConvexity = 1.0

		# Filter by Inertia
		detector_params.filterByInertia = False
		detector_params.minInertiaRatio = 0.5
		detector_params.maxInertiaRatio = 1.0

		detector_params.minRepeatability = 1
		detector_params.minDistBetweenBlobs = 3000

		try:
			logging.debug("Loading blob detector settings from file '{}'".format(file_name))
			with open(file_name, 'r') as f:  # Open file for input
				for line in f:  # Every line contains one property: name + sep + value
					name, value = line.split(sep)
					setattr(detector_params, name, eval(value))  # Use eval to cast to right type (eg: False instead of "False")
					logging.debug("\tLoaded blob detector setting: '{}' = {}".format(name, value))
		except:
			logging.exception("Something went wrong while loading blob detector settings from '{}'.".format(file_name))
			return False
		return True

	def connect_to_cf(self, retry_after=10, max_timeout=20):
		"""
		Initializes radio drivers, looks for available CrazyRadios, and attempts to connect to the first available one.
		Doesn't return anything, but raises exceptions if anything goes wrong.
		:param retry_after: Time in seconds after which we should abort current connection and restart the attempt.
		:param max_timeout: Time in seconds after which we should give up if we haven't been able to establish comm. yet
		"""
		logging.info("Initializing drivers.")
		crtp.init_drivers(enable_debug_driver=False)

		logging.info("Setting up the communication link. Looking for available CrazyRadios in range.")
		available_links = crtp.scan_interfaces()
		if len(available_links) == 0:
			raise Exception("Error, no Crazyflies found. Exiting.")
		else:
			logging.info("CrazyFlies found:")  # For debugging purposes, show info about available links
			for i in available_links:
				logging.info("\t" + i[0])
			link_uri = available_links[0][0]  # Choose first available link

		logging.info("Initializing CrazyFlie (connecting to first available interface: '{}').".format(link_uri))
		self.crazyflie = Crazyflie(ro_cache="cachero", rw_cache="cacherw")  # Create an instance of Crazyflie
		self.crazyflie.connected.add_callback(self.on_cf_radio_connected)  # Set up callback functions for communication feedback
		self.crazyflie.disconnected.add_callback(self.on_cf_radio_disconnected)
		self.crazyflie.connection_failed.add_callback(self.on_cf_radio_conn_failed)
		self.crazyflie.connection_lost.add_callback(self.on_cf_radio_conn_lost)

		cnt = 0  # Initialize a time counter
		while self.cf_radio_connecting and cnt < max_timeout:
			if cnt % retry_after == 0:
				if cnt > 0:  # Only show warning after first failed attempt
					logging.warning("Unable to establish communication with Crazyflie ({}) after {}s. Retrying...".format(link_uri, retry_after))
					self.crazyflie.close_link()  # Closing the link will call on_disconnect, which will set cf_radio_connecting to False
					self.cf_radio_connecting = True  # Reset cf_radio_connecting back to True
				self.crazyflie.open_link(link_uri)  # Connect/Reconnect to the CrazyRadio through the selected interface
			time.sleep(1)  # Sleep for a second (give time for callback functions to detect whether we are connected)
			cnt += 1  # Increase the "waiting seconds" counter

		if cnt >= max_timeout:
			self.crazyflie.close_link()
			raise Exception("Unable to establish communication with CrazyFlie after {}s. Given up :(".format(max_timeout))
		elif not self.cf_radio_connected:
			raise Exception("Something failed while attempting to connect to the CrazyFlie, exiting.")
		# self.crazyflie.commander.send_setpoint(0, 0, 0, 0)  # If we successfully connected to the CF, send thrust=0 (new firmware initializes thrustLock=True, only way to unlock it so it executes commands is by setting thrust=0)

	def setup_cf(self):
		"""
		Sets up the CrazyFlie before running the experiment. This includes configuring some params to default values
		and requesting the CrazyFlie to log certain variables back at constant intervals.
		Doesn't return anything, but raises exceptions if anything goes wrong.
		"""
		try:  # Send some default values for CF params
			self.crazyflie.param.set_value('controller.tiltComp', '{:d}'.format(True))
			self.crazyflie.param.set_value('flightmode.poshold', '{:d}'.format(False))  # Disable poshold and althold by default
			self.crazyflie.param.set_value('flightmode.althold', '{:d}'.format(False))
			self.crazyflie.param.set_value('flightmode.posSet', '{:d}'.format(False))
			self.crazyflie.param.set_value('flightmode.yawMode', '0')
			self.crazyflie.param.set_value('flightmode.timeoutStab', '{:d}'.format(1000*60*10))  # Stabilize (rpy=0) CF if doesn't receive a radio command in 10min
			self.crazyflie.param.set_value('flightmode.timeoutShut', '{:d}'.format(1000*60*20))  # Shutdown CF if doesn't receive a radio command in 20min
			self.crazyflie.param.set_value('posCtlPid.thrustBase', '{}'.format(self.TAKEOFF_THRUST))
			self.crazyflie.param.set_value("ring.effect", "1")  # Turn off LED ring
			self.crazyflie.param.set_value("ring.headlightEnable", "0")  # Turn off LED headlight
		except Exception as e:
			raise Exception("Couldn't initialize CrazyFlie params to their desired values. Details: {}".format(e.message))

		# Create a log configuration and include all variables that want to be logged
		self.cf_log = LogConfig(name="cf_log", period_in_ms=10)
		self.cf_log.add_variable("stabilizer.roll", "float")
		self.cf_log.add_variable("stabilizer.pitch", "float")
		self.cf_log.add_variable("stabilizer.yaw", "float")
		self.cf_log.add_variable("posEstimatorAlt.estimatedZ", "float")
		self.cf_log.add_variable("posEstimatorAlt.velocityZ", "float")
		try:
			self.crazyflie.log.add_config(self.cf_log)  # Validate the log configuration and attach it to our CF
		except Exception as e:
			raise Exception("Couldn't attach the log config to the CrazyFlie, bad configuration. Details: {}".format(e.message))
		self.cf_log.data_received_cb.add_callback(self.on_cf_log_new_data)  # Register appropriate callbacks
		self.cf_log.error_cb.add_callback(self.on_cf_log_error)
		self.cf_log.start()  # Start logging!

		# Get the CF's initial yaw (should be facing the camera) so we can have PID_yaw maintain that orientation
		if self.ASK_FOR_TARGET_YAW:  # Either ask the user to press Enter to indicate the CF's orientation is ready
			raw_input("\nRotate the drone so it faces the camera, press Enter when you're ready...\n")
		else:  # Or automatically detect the first yaw log packet and set the current orientation as the desired yaw
			while abs(self.cf_yaw) < 0.01:  # Wait until first cf_yaw value is received (cf_yaw=0 by default)
				time.sleep(0.1)
		self.cf_PID_yaw.SetPoint = self.cf_yaw
		print "Target yaw set at {:.2f}.".format(self.cf_yaw)

		self.crazyflie.add_port_callback(CRTPPort.CONSOLE, self.print_cf_console)
		self.crazyflie.commander.send_setpoint(0, 0, 0, 0)  # New firmware version requires to send thrust=0 at least once to "unlock thrust"

	def fly_cf(self):
		"""
		Provides the structure to make the drone fly (actual flight control is done in hover() though).
		hover() is called until user stops the experiment, then hover is called for a few more secs to record more data
		(usually includes information about the CF crashing...)
		:return: Whether or not to keep (store) the logs, based on whether the CF ever actually took off and flew
		"""
		print "Giving you 5s to prepare for take-off..."
		time.sleep(5)

		# t = Timer(20, self.m_CrazyFlie.close_link)  # Start a timer to automatically disconnect in 20s
		# t.start()

		# Prepare for take off: clear PIDs, log start time...
		self.cf_str_status = "TAKING OFF"
		self.t_start = datetime.now()
		self.cf_PID_roll.clear()
		self.cf_PID_pitch.clear()
		self.cf_PID_yaw.clear()
		self.cf_PID_thrust.clear()

		# Alright, let's fly!
		tStop = None
		while tStop is None:  # tStop will remain None while everything's fine
			tStop = self.hover()  # hover returns None while everything's fine; the time to end the experiment otherwise

		# If we get here, either the user stopped the experiment or the code detected something went wrong
		print "AT t={}, A KEY WAS PRESSED -> STOPPING!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])
		save_logs = (self.cf_str_status != "TAKING OFF")  # Only store the logs if the drone ever started flying (not just took off)
		self.cf_str_status = "STOPPED"  # Updated new drone status

		while datetime.now() < tStop:  # Keep calling hover until tStop, so data is still logged
			self.hover()

		return save_logs  # Return whether or not to keep the logs

	def cleanup_experiment(self, save_logs=True):
		"""
		"Cleans up" the experiment: closes any open windows, saves logs, disconnects camera and drone, etc.
		"""
		# If we get here, either the user ended the experiment, or an exception occurred. Same action regardless:
		if self.cf_log is not None:  # If we were logging data from the CF, stop it (will reconnect faster next time)
			self.cf_log.stop()
			self.cf_log.delete()
		if self.crazyflie is not None:  # If the CF was ever initialized, make sure the communication link is closed
			self.crazyflie.close_link()

		self.video_capture = None  # Destroy the video capture object (this takes care of closing the camera etc.)
		cv2.destroyAllWindows()  # Close all UI windows that could be open
		if self.window_for_kb_input is not None:
			self.window_for_kb_input.hide()
			sdl2.ext.quit()

		if save_logs:  # If the experiment didn't crash before starting (the CF ever took off), plot and save the logs
			self.experiment_log.plot(False)
			self.experiment_log.save()
		else:  # Otherwise just delete the folder (and its contents) where cam frames would have been/were saved
			shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=False)

	def run_experiment(self):
		"""
		DroneController's main method: initializes all components (vision, communication, drone params, etc.) then
		runs the experiment.
		"""
		logging.disable(logging.DEBUG)  # Seems to work better than .basicConfig(INFO), especially if logging has already been initialized -> Only show messages of level INFO or higher

		save_logs = True  # Use this auxiliary variable to prevent saving logs if the drone never took off
		try:
			self.connect_to_cf()  # Connect to the CrazyFlie
			self.setup_cf()  # Can't initialize LogConfig until we're connected, because it needs to check that the variables we'd like to add are in the TOC. So this function must be called after connect_to_cf()
			self.init_video_cam_and_cv_algorithm()  # Connect to the first available camera, load default settings, etc.
			self.init_UI_window()  # Open a window to receive user input to control the CF
			save_logs = self.fly_cf()  # And finally fly the CF
		except:
			logging.exception("Shutting down due to an exception =( See details below:")

		# If we got here, either the user ended the experiment, or an exception occurred. Same action regardless:
		self.cleanup_experiment(save_logs)  # "Cleanup" the experiment: close windows, save logs, etc.

	def get_cf_target_pos(self):
		"""
		:return: np.array containing the current (estimated) 3D position of the CrazyFlie
		"""
		return np.array([int(round(x)) for x in [self.cf_PID_roll.getSetPoint(), self.cf_PID_thrust.getSetPoint(), self.cf_PID_pitch.getSetPoint()]])

	def get_cf_curr_pos(self):
		"""
		:return: np.array containing the current (estimated) 3D position of the CrazyFlie
		"""
		return np.array([self.cf_PID_roll.getCurrPos(), self.cf_PID_thrust.getCurrPos(), self.cf_PID_pitch.getCurrPos()])

	def get_cf_curr_vel(self):
		"""
		:return: np.array containing the current (estimated) 3D velocity of the CrazyFlie
		"""
		return np.array([self.cf_PID_roll.getCurrVel(), self.cf_PID_thrust.getCurrVel(), self.cf_PID_pitch.getCurrVel()])

	def on_cf_radio_connected(self, link_uri):
		logging.info("Successfully connected to Crazyflie at '{}'!".format(link_uri))
		self.cf_radio_connecting = False
		self.cf_radio_connected = True

	def on_cf_radio_conn_failed(self, link_uri, msg):
		logging.error("Connection to '{}' failed: {}.".format(link_uri, msg))  # Initial connection fails (i.e no Crazyflie at the speficied address)
		self.cf_radio_connecting = False
		self.cf_radio_connected = False

	def on_cf_radio_conn_lost(self, link_uri, msg):
		logging.warning("Connection to '{}' lost: {}.".format(link_uri, msg))  # Disconnected after a connection has been made (i.e Crazyflie moves out of range)

	def on_cf_radio_disconnected(self, link_uri):
		logging.error("Disconnected from '{}'.".format(link_uri))  # Crazyflie is disconnected (called in all cases)
		self.cf_radio_connecting = False
		self.cf_radio_connected = False

	def on_cf_log_error(self, logconf, msg):
		logging.error("Error when logging %s: %s." % (logconf.name, msg))

	def on_cf_log_new_data(self, timestamp, data, logconf):
		logging.debug("[%d][%s]: %s" % (timestamp, logconf.name, data))
		self.cf_roll = data['stabilizer.roll']
		self.cf_pitch = data['stabilizer.pitch']
		self.cf_yaw = data['stabilizer.yaw']
		self.cf_estimated_z = data['posEstimatorAlt.estimatedZ']
		self.cf_vel_z = data['posEstimatorAlt.velocityZ']
		print "\rCurrent yaw: {:.2f}deg".format(self.cf_yaw),

	def print_cf_console(self, packet):
		console_text = packet.data.decode('UTF-8')
		print("Console: {}".format(console_text))

	def send_cf_param(self, complete_name, value):
		"""
		Modified version of crazyflie.param.set_value that sends the packet immediately (instead of using a Thread+Queue
		"""
		element = self.crazyflie.param.toc.get_element_by_complete_name(complete_name)

		if not element:
			raise KeyError("Couldn't set {}={}, param is not in the TOC!".format(complete_name, value))
		elif element.access == ParamTocElement.RO_ACCESS:
			raise AttributeError("Couldn't set {}={}, param is read-only!".format(complete_name, value))
		else:
			varid = element.ident
			pk = CRTPPacket()
			pk.set_header(CRTPPort.PARAM, WRITE_CHANNEL)
			pk.data = struct.pack('<B', varid)
			pk.data += struct.pack(element.pytype, eval(value))
			self.crazyflie.send_packet(pk, expected_reply=(tuple(pk.data[0:2])))

	def process_kb_input(self):
		"""
		Processes all keydown events, and takes the corresponding action depending on which key was pressed.
		:return: Boolean indicating whether the experiment shall go on: True while everything's fine, False to stop it
		"""
		events = sdl2.ext.get_events()  # Fetch any new input event
		for event in events:  # Iterate through all of them
			if event.type == sdl2.SDL_KEYDOWN:  # And only process keydown events
				key_orig = event.key.keysym.sym  # Fetch the key that was pressed down
				try:
					key = chr(key_orig)  # Try to convert the key to char
				except:  # Usually, this exeption occurs when a key can't be converted to char (arrows, Esc, etc.)
					if key_orig == sdl2.SDLK_UP:
						key = "Up"
					elif key_orig == sdl2.SDLK_DOWN:
						key = "Down"
					elif key_orig == sdl2.SDLK_ESCAPE:
						key = "Esc"
					else:
						key = hex(key_orig)

				logging.info("Key: '{}'".format(key))
				self.window_for_kb_input.title = "Last key pressed: '{}' at t={}".format(key, str(datetime.now().time())[:-3])
				key = key.lower()  # Convert to lowercase so we don't have to worry about different cases
				if key == 'a':    # Move left
					self.cf_PID_roll.setSetPoint(self.cf_PID_roll.getSetPoint() + 20)
				elif key == 's':  # Move back
					self.cf_PID_pitch.setSetPoint(max(self.cf_PID_pitch.getSetPoint() - 2, 1))  # Size (ball radius) can't be negative! Make sure depth value is at least 1px
				elif key == 'd':  # Move right
					self.cf_PID_roll.setSetPoint(self.cf_PID_roll.getSetPoint() - 20)
				elif key == 'w':  # Move forward
					self.cf_PID_pitch.setSetPoint(self.cf_PID_pitch.getSetPoint() + 2)
				elif key == 'u':  # Move up
					self.cf_PID_thrust.setSetPoint(self.cf_PID_thrust.getSetPoint() - 20)
				elif key == 'h':  # Move down
					self.cf_PID_thrust.setSetPoint(self.cf_PID_thrust.getSetPoint() + 20)
				elif key == 'f':  # Toggle altitude hold mode
					if self.cf_ignore_camera: self.cf_PID_thrust.clear()  # If we were ignoring the camera for Z, thrust PID will have a wrong I component
					self.cf_ignore_camera = not self.cf_ignore_camera
					self.cf_str_status = "NO CAM for Z" if self.cf_ignore_camera else "FULL CAM"
					# self.crazyflie.param.set_value('flightmode.althold', '{:d}'.format(self.cf_ignore_camera))
					# while not self.crazyflie.param.param_updater.request_queue.empty():  # Wait for the packet to be sent
					# 	time.sleep(0.01)
					self.send_cf_param('flightmode.althold', '{:d}'.format(self.cf_ignore_camera))
				elif key == 'e':  # Stop taking off and start flying (hover at current position)
					self.cf_taking_off = False
					self.cf_str_status = "FLYING"
					self.cf_PID_roll.setSetPoint(self.cf_PID_roll.getCurrPos())
					self.cf_PID_pitch.setSetPoint(self.cf_PID_pitch.getCurrPos() + 2)
					self.cf_PID_thrust.setSetPoint(self.cf_PID_thrust.getCurrPos())
					self.cf_PID_roll.clear()
					self.cf_PID_pitch.clear()
					self.cf_PID_thrust.clear()
				else:  # Any other key ends the experiment
					# self.crazyflie.param.set_value('flightmode.althold', '{:d}'.format(False))  # Make sure we're not on althold mode, so sending a thrust 0 will kill the motors and not just descend
					self.send_cf_param('flightmode.althold', '{:d}'.format(False))  # Make sure we're not on althold mode, so sending a thrust 0 will kill the motors and not just descend
					return False
		return True

	def detect_cf_in_camera(self, frame=None, find_blob=True):
		"""
		Runs an iteration of the computer vision algorithm that estimates the CrazyFlie's position in 3D. That is,
		it captures a frame from the camera, converts it to HSV, filters by color, and detects a blob.
		Saves the 3D position in cf_curr_pos, camera frame in cv_cam_frame and color filter mask in cv_filtered_HSV_mask.
		"""
		#########################################################
		#                       CAPTURE FRAME                   #
		#########################################################
		self.t_events = [datetime.now()]
		self.cv_cam_frame = frame  # Allow the function to receive a frame if we already got it from the camera
		if self.cv_cam_frame is None:  # Otherwise, if frame is None (default), grab a frame from the camera
			try:
				uvc_frame = self.video_capture.get_frame_robust()  # read() blocks execution until a new frame arrives! -> Obtain t AFTER grabbing the frame
				self.t_events.append(datetime.now())
				self.t_frame = self.t_events[-1]
				self.cv_cam_frame = uvc_frame.bgr  # .copy()
				self.t_events.append(datetime.now())
			except Exception as e:
				raise Exception("Unexpected error accessing the camera frame :( Details: {}.".format(e.message))

		#########################################################
		#                     COLOR THRESHOLD                   #
		#########################################################
		# self.cv_cam_frame = cv2.resize(self.cv_cam_frame, None, fx=0.5, fy=0.5)
		# self.t_events.append(datetime.now())
		# self.cv_cam_frame = cv2.GaussianBlur(self.cv_cam_frame, (3, 3), 0)  # Not needed, camera is already physically "blurring" (sharpness parameter set to 0)
		# self.t_events.append(datetime.now())
		frame_HSV = cv2.cvtColor(self.cv_cam_frame, cv2.COLOR_BGR2HSV)
		self.t_events.append(datetime.now())
		self.cv_filtered_HSV_mask = cv2.inRange(frame_HSV, self.cv_HSV_thresh_min, self.cv_HSV_thresh_max)
		self.t_events.append(datetime.now())
		self.cv_filtered_HSV_mask = cv2.morphologyEx(self.cv_filtered_HSV_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
		self.t_events.append(datetime.now())

		#########################################################
		#                     DRONE DETECTION                   #
		#########################################################
		if find_blob:
			keypoints = cv2.SimpleBlobDetector_create(self.cv_blob_detect_params).detect(self.cv_filtered_HSV_mask)
			self.t_events.append(datetime.now())
			self.cf_pos_tracked = bool(keypoints)  # For now we determine the CF's position is tracked if we find at least 1 blob
			if keypoints:  # If the cv algorithm detected at least one blob
				keypoint = max(keypoints, key=attrgetter('size'))  # Focus on the biggest blob
				self.cf_curr_pos = np.hstack((keypoint.pt, keypoint.size/2))  # And save the position estimated by the CV algorithm

	def control_cf(self):
		"""
		Sends messages to control the CrazyFlie (roll-pitch-yaw-thrust setpoints) using the current location and PID loops.
		:param drone_curr_pos: 3D np.array containing the current (estimated) position of the drone
		"""
		if not self.cf_pos_tracked:  # If cv algorithm wasn't able to detect the drone, linearly estimate its position based on previous position and speed
			# Since PIDs haven't been updated with current values yet, don't have to multiply velocity by (curr_time-last_time) but rather by (t_frame-curr_time)
			self.cf_curr_pos = \
				np.array([self.cf_PID_roll.getCurrPos(),  self.cf_PID_thrust.getCurrPos(), self.cf_PID_pitch.getCurrPos()]) + \
				np.array([self.cf_PID_roll.getCurrVel(), -self.cf_PID_thrust.getCurrVel(), self.cf_PID_pitch.getCurrVel()]) * \
				(self.t_frame - self.cf_PID_thrust.PIDvel.curr_time).total_seconds()
			self.cf_curr_pos[2] = max(1, self.cf_curr_pos[2])  # Make sure 3D (size) stays positive! (Prevents errors further down the line)

		# Update PID loops with new position at t=self.t_frame
		self.cf_PID_roll.update(self.cf_curr_pos[0], self.t_frame)
		self.cf_PID_pitch.update(self.cf_curr_pos[2], self.t_frame)
		self.cf_PID_yaw.update(self.cf_yaw, self.t_frame)
		self.cf_PID_thrust.update(self.cf_curr_pos[1], self.t_frame)

		# Log all relevant variables after each iteration
		self.experiment_log.update(roll=self.cf_PID_roll, pitch=self.cf_PID_pitch, yaw=self.cf_PID_yaw, thrust=self.cf_PID_thrust, estimated_z=self.cf_estimated_z, velocity_z=self.cf_vel_z)

		# Send the appropriate roll-pitch-yaw-thrust setpoint depending on the scenario (eg: taking off, stopping, etc.)
		if self.cf_radio_connected:  # While the experiment is running
			if self.cf_taking_off:  # If taking off, send a constant RPYT setpoint that will make the CF go up "straight"
				# self.crazyflie.commander.send_setpoint(self.cf_PID_roll.PIDvel.out_offs, self.cf_PID_pitch.PIDvel.out_offs, self.cf_PID_yaw.output, self.TAKEOFF_THRUST)
				self.crazyflie.commander.send_setpoint(self.cf_PID_roll.PIDvel.out_offs, self.cf_PID_pitch.PIDvel.out_offs, 0, self.TAKEOFF_THRUST)
			else:  # This condition holds ever after take off (once the user presses the key to start "flying")
				if self.cf_ignore_camera:  # If user selected it, control the drone in althold mode (altitude is done on-board, rest is still controlled with the cam)
					# self.crazyflie.commander.send_setpoint(self.cf_PID_roll.getOutput(), self.cf_PID_pitch.getOutput(), self.cf_PID_yaw.output, 32767)
					self.crazyflie.commander.send_setpoint(self.cf_PID_roll.getOutput(), self.cf_PID_pitch.getOutput(), 0, 32767)
				elif not self.cf_pos_tracked:  # If we couldn't find the drone, don't send any commands (not to mislead it)
					pass
				else:  # Otherwise, just use the camera to control all 4 independent axes
					# self.crazyflie.commander.send_setpoint(self.cf_PID_roll.getOutput(), self.cf_PID_pitch.getOutput(), self.cf_PID_yaw.output, self.cf_PID_thrust.getOutput())
					self.crazyflie.commander.send_setpoint(self.cf_PID_roll.getOutput(), self.cf_PID_pitch.getOutput(), 0, self.cf_PID_thrust.getOutput())
		else:  # If the user has decided to end the experiment, kill the motors and reset PIDs (this is not really necessary)
			self.crazyflie.commander.send_setpoint(0, 0, 0, 0)
			self.cf_PID_roll.clear()
			self.cf_PID_pitch.clear()
			self.cf_PID_yaw.clear()
			self.cf_PID_thrust.clear()

		self.t_events.append(datetime.now())

	def get_OSD_text(self, t):
		"""
		Generates written debug information (OSD=on-screen display) to be displayed at the bottom of the current frame
		:param t: Datetime at which camera frame was obtained (through datetime.now())
		:return: String containing relevant debug information (PID values, drone estimated position, etc.)
		"""
		formatNum = "{:+6.2f}"
		strPrint = ("ROLLp. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"ROLLv. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"PITCHp={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"PITCHv={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"YAW..  ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"THRUSp={:3.0f};{:6.0f} [{:+6.0f}, {:+6.0f}, {:+6.0f}]\t\t" +
					"THRUSv={:3.0f};{:6.0f} [{:+6.0f}, {:+6.0f}, {:+6.0f}]\t\t" +
					"[x:{:4.0f}, y:{:4.0f}, z:{:4.0f}], [vx:{:4.0f}, vy:{:4.0f}, vz:{:4.0f}], " +
					"rpy: " + formatNum + "," + formatNum + "," + formatNum + "]\t\t" + "@{} (FPS: {:5.2f}) - " + self.cf_str_status).format(
			self.cf_PID_roll.PIDpos.SetPoint, self.cf_PID_roll.PIDpos.output, self.cf_PID_roll.PIDpos.PTerm, self.cf_PID_roll.PIDpos.Ki * self.cf_PID_roll.PIDpos.ITerm, self.cf_PID_roll.PIDpos.Kd * self.cf_PID_roll.PIDpos.DTerm,
			self.cf_PID_roll.PIDvel.SetPoint, self.cf_PID_roll.PIDvel.output, self.cf_PID_roll.PIDvel.PTerm, self.cf_PID_roll.PIDvel.Ki * self.cf_PID_roll.PIDvel.ITerm, self.cf_PID_roll.PIDvel.Kd * self.cf_PID_roll.PIDvel.DTerm,
			self.cf_PID_pitch.PIDpos.SetPoint, self.cf_PID_pitch.PIDpos.output, self.cf_PID_pitch.PIDpos.PTerm, self.cf_PID_pitch.PIDpos.Ki * self.cf_PID_pitch.PIDpos.ITerm, self.cf_PID_pitch.PIDpos.Kd * self.cf_PID_pitch.PIDpos.DTerm,
			self.cf_PID_pitch.PIDvel.SetPoint, self.cf_PID_pitch.PIDvel.output, self.cf_PID_pitch.PIDvel.PTerm, self.cf_PID_pitch.PIDvel.Ki * self.cf_PID_pitch.PIDvel.ITerm, self.cf_PID_pitch.PIDvel.Kd * self.cf_PID_pitch.PIDvel.DTerm,
			self.cf_PID_yaw.SetPoint, self.cf_PID_yaw.output, self.cf_PID_yaw.PTerm, self.cf_PID_yaw.Ki * self.cf_PID_yaw.ITerm, self.cf_PID_yaw.Kd * self.cf_PID_yaw.DTerm,
			self.cf_PID_thrust.PIDpos.SetPoint, self.cf_PID_thrust.PIDpos.output, self.cf_PID_thrust.PIDpos.PTerm, self.cf_PID_thrust.PIDpos.Ki * self.cf_PID_thrust.PIDpos.ITerm, self.cf_PID_thrust.PIDpos.Kd * self.cf_PID_thrust.PIDpos.DTerm,
			self.cf_PID_thrust.PIDvel.SetPoint, self.cf_PID_thrust.PIDvel.output, self.cf_PID_thrust.PIDvel.PTerm, self.cf_PID_thrust.PIDvel.Ki * self.cf_PID_thrust.PIDvel.ITerm, self.cf_PID_thrust.PIDvel.Kd * self.cf_PID_thrust.PIDvel.DTerm,
			self.cf_PID_roll.getCurrPos(), self.cf_PID_thrust.getCurrPos(), self.cf_PID_pitch.getCurrPos(), self.cf_PID_roll.getCurrVel(), self.cf_PID_thrust.getCurrVel(), self.cf_PID_pitch.getCurrVel(), self.cf_roll, self.cf_pitch, self.cf_yaw, str(t-self.t_start)[3:-3], 1./(t-self.t_last_frame).total_seconds())

		# logging.debug(strPrint)
		return "          SP | SENT  [   P   ,   I   ,   D  ]\t\t" + strPrint

	def save_algo_iteration(self, str_OSD="", newline_separator='\t\t', margin_x=25, margin_y=25, text_color=(200, 200, 200), font=cv2.FONT_HERSHEY_DUPLEX, font_scale=0.7, font_thickness=1, line_type=cv2.LINE_AA, mask_color=(255, 0, 255), img_resize_factor=0.5, save_cam_frame_before_resizing=True):
		if str_OSD == "":  # Allow for custom OSD text, but if no text specified, print the default debug info (get_OSD_text)
			str_OSD = self.get_OSD_text(self.t_frame)
			self.t_events.append(datetime.now())

		# Resize camera frame and CrazyFlie's current&target positions according to img_resize_factor
		curr_pos_resized = self.get_cf_curr_pos()*img_resize_factor
		target_pos_resized = self.get_cf_target_pos()*img_resize_factor
		frame_resized = cv2.resize(self.cv_cam_frame, None, fx=img_resize_factor, fy=img_resize_factor)
		mask_resized = cv2.resize(self.cv_filtered_HSV_mask, None, fx=img_resize_factor, fy=img_resize_factor)

		# Save the original camera frame to disk (for post-debugging if necessary)
		###### cv2.imwrite(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("frame_%H-%M-%S-%f.jpg")), self.cv_cam_frame if save_cam_frame_before_resizing else frame_resized)
		self.t_events.append(datetime.now())

		# Plot OSD related to CF's current and target positions (2 circles and a connecting line)
		cv2.circle(frame_resized, tuple(curr_pos_resized[0:2].astype(int)), int(curr_pos_resized[2]), self.COLOR_BALL_TRACKED if self.cf_pos_tracked else self.COLOR_BALL_UNTRACKED, -1)
		cv2.line(frame_resized, tuple(curr_pos_resized[0:2].astype(int)), tuple(target_pos_resized[0:2].astype(int)), self.COLOR_LINE_TRACKED if self.cf_pos_tracked else self.COLOR_LINE_UNTRACKED, int(10*img_resize_factor))
		cv2.circle(frame_resized, tuple(target_pos_resized[0:2].astype(int)), int(target_pos_resized[2]), self.COLOR_TARGET_TRACKED if self.cf_pos_tracked else self.COLOR_TARGET_UNTRACKED, -1)
		self.t_events.append(datetime.now())

		# On top of that, overlay the HSV mask (so we can debug color filtering + blob detection steps)
		np.putmask(frame_resized, cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR).astype(bool), list(mask_color))
		self.t_events.append(datetime.now())

		# Generate the output image: upper part is the cam frame downsized according to img_resize_factor; lower part, str_OSD
		lines = str_OSD.split(newline_separator)
		self.cv_frame_out = np.zeros(((frame_resized.shape[0] + margin_y*(len(lines)+1)), frame_resized.shape[1], frame_resized.shape[2]), dtype=frame_resized.dtype)
		self.cv_frame_out[0:frame_resized.shape[0], :] = frame_resized
		self.t_events.append(datetime.now())

		for cnt, l in enumerate(lines):  # Add every line of text in str_OSD. Note that putText asks for bottom-left corner of text and that cnt=0 for 1st line. Therefore vertical component should be frame_resized height + OSD padding/border (0.5*margin_y) + text height (1*margin_y)
			cv2.putText(self.cv_frame_out, l.replace('\t', '; '), (margin_x, frame_resized.shape[0] + int(margin_y*(cnt+1.4))), font, font_scale, text_color, font_thickness, line_type)

		# Save the output image to disk (for post-debugging if necessary)
		cv2.imwrite(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("out_%H-%M-%S-%f.jpg")), self.cv_frame_out)
		self.t_events.append(datetime.now())

	def hover(self):
		try:  # First, run the cv algorithm to estimate the CF's position
			self.detect_cf_in_camera()
		except:  # Only way detect_cf_in_camera raises an Exception is if a camera frame couldn't be grabbed
			logging.exception("Couldn't grab a frame from the camera. Exiting")
			return datetime.now()  # Need to stop now (if I wanted to stop in now+2sec and camera kept throwing exceptions, it would keep delaying the stop and never actually stop)

		# Now that we know where the drone currently is, send messages to control it (roll-pitch-yaw-thrust setpoints)
		self.control_cf()

		# And save the intermediate and output frames/images to disk for debugging
		self.save_algo_iteration()

		# Last, process kb input to control the experiment
		if not self.process_kb_input():  # Process kb input and take action if necessary (will return False when user wants to stop the experiment)
			self.cf_radio_connected = False  # Set connected to False so next calls to this function just send thrust=0 messages
			return datetime.now() + timedelta(seconds=2)  # For debugging purposes, it's great to have a few additional seconds of video&log after an experiment is stopped (helps see why it crashed)
		self.t_events.append(datetime.now())

		logging.debug("DeltaT = {:5.2f}ms -> Total: {:5.2f}ms{}".format(
			(datetime.now() - self.t_frame).total_seconds()*1000, (self.t_frame - self.t_last_frame).total_seconds()*1000,
			"".join(["\t{}->{}: {}ms".format(i+1, i+2, (self.t_events[i+1]-self.t_events[i]).total_seconds()*1000) for i in range(len(self.t_events) - 1)])))
		self.t_last_frame = self.t_frame  # Remember the time this frame was taken so we can estimate FPS in next iteration

		return None  # Return None to indicate that the experiment shall go on. All good.

	def estimate_cf_circle_depth(self):
		if False:
			t1 = datetime.now()
			_, contours, _ = cv2.findContours(self.cv_filtered_HSV_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			contour = contours[0]
			r = cv2.boundingRect(contour)
			c = cv2.minEnclosingCircle(contour)
			cv2.rectangle(self.cv_frame_out, r[0:2], (r[0] + r[2], r[1] + r[3]), (0, 255, 0), 2, cv2.LINE_AA)
			cv2.circle(self.cv_frame_out, tuple(int(x) for x in c[0]), int(c[1]), (0, 255, 0), 2, cv2.LINE_AA)
			t2 = datetime.now()
			print "\t\t{} ms;\t\t\tc:{}\t\tblob:{}".format((t2-t1).total_seconds()*1000, c[1], self.cf_curr_pos[2])


if __name__ == '__main__':
	sendCommands = True
	h = DroneController()
	h.run_experiment()
