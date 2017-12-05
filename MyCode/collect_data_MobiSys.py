"""
	Main code that implements full control (X, Y, Z , yaw) of a Crazyflie using a camera through OpenCV.
	The algorithm basically does the following:
	 - Fly at a set constant thrust to take off, until a key ("e") is pressed.
		When that happens, current position is used as target position to hold
	 - Using the camera, we find the x,y coordinates of the drone, and estimate z based on size of a known-size object
	 - Independent PIV loops (position + velocity PIDs) control movement in each direction (x, y, z)
	 - ASDWUH keys can modify the target position to hold, any other key will kill the algorithm and stop the drone.
		When that happens, all the sensor and control data that's been logged will be plotted and stored in files.
	 - NOTE: PID constants could still be further tuned to reduce overshoot and improve stability.
"""

import logging
import threading
import time
import shutil
import os
from datetime import datetime, timedelta

import numpy as np
import cv2
import sdl2
import sdl2.ext

import vision_aux_functions as auxV
from uvc_capture import UvcCapture
# from operator import attrgetter
import PID
import plot_tools

import struct
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.param import ParamTocElement, WRITE_CHANNEL
from cflib import crtp
from cflib.crtp.crtpstack import CRTPPort, CRTPPacket


def pre_experiment_find_extrinsics(self, bool_world_coords_pattern):  # self can be either GroundTruthThread or Spotter
	"""
	Finds (or assumes default) extrinsic matrix (world->cam coordinate transformation matrix) before we start an experiment.
	:param self: Object with a video_capture member (which is able to collect camera frames) and a
	world_to_camera_transf member, which wants to be set with the found (or default) extrinsic matrix.
	:param bool_world_coords_pattern: If False, the camera is just assumed to be at the origin of the world frame, so
	default extrinsics are used; If True, we look for the calibration pattern in the camera images.
	"""
	def default_extrinsics():  # Place the world origin at cam location (translation=0), and simply rotate the axes so that: x_cam=-y_world, y_cam=z_world, z_cam=-x_world [assumes camera is horizontal!]
		return np.matrix([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0]])

	if bool_world_coords_pattern:
		win_title = "Are these extrinsic values correct?"
		cv2.namedWindow(win_title)
		cv2.moveWindow(win_title, 0,0)
		while self.world_to_camera_transf is None:
			img = self.video_capture.get_frame_robust().bgr
			self.world_to_camera_transf, _ = auxV.find_world_to_cam_and_F(img, self.video_capture.camera_matrix, self.video_capture.dist_coefs)
			if self.world_to_camera_transf is not None:
				is_chessboard = False
				pattern_grid, _ = auxV.get_calib_pattern_info(is_chessboard)
				found, corners = auxV.find_calib_pattern(img, is_chessboard, pattern_grid)
				cv2.drawChessboardCorners(img, pattern_grid, corners, found)
				cv2.circle(img, tuple([int(x) for x in auxV.cam_to_img_coords(auxV.world_to_cam_coords([0,0,0], self.world_to_camera_transf), self.video_capture.camera_matrix, self.video_capture.dist_coefs)]), 10, (100,100,100), -1)
				cv2.putText(img, "Cam is at [x={p[0]:.2f}cm, y={p[1]:.2f}cm, z={p[2]:.2f}cm]; F={F:.2f}px".format(p=100*auxV.cam_to_world_coords([0, 0, 0], self.world_to_camera_transf), F=self.CAM_FOCAL_LENGTH_IN_PX), (50, img.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 0, 200), 1, cv2.LINE_AA)

			cv2.imshow(win_title, img)
			key = cv2.waitKey(1 if self.world_to_camera_transf is None else 0)
			if key == 27:  # Esc
				raise Exception("User decided to stop the experiment before it started.")
			elif key == ord('b'):  # Bypass
				self.world_to_camera_transf = default_extrinsics()
			elif key != ord('y'):  # In case world_to_camera_transf is not None, the user has to agree to use this frame, so pressing any key other than 'y' will get next frame
				self.world_to_camera_transf = None
		cv2.destroyWindow(win_title)
		print("Camera is at position [x={p[0]:.2f}cm, y={p[1]:.2f}cm, z={p[2]:.2f}cm]; F={F:.2f}px (length of 1m segment, 1m away from the camera)".format(p=100*auxV.cam_to_world_coords([0, 0, 0], self.world_to_camera_transf), F=self.CAM_FOCAL_LENGTH_IN_PX))
	else:  # If no pattern is provided to compute the extrinsic matrix, place the world origin at cam location (translation=0), and simply rotate the axes so that: x_cam=-y_world, y_cam=z_world, z_cam=-x_world [assumes camera is horizontal!]
		self.world_to_camera_transf = default_extrinsics()


class FakeVideoCapture:
	"""
	Class with camera_matrix and dist_coefs members to trick CV algorithms into thinking this is a
	valid UvcCapture object, without actually requiring it to connect to a camera, etc.
	"""

	def __init__(self, camera_matrix=np.eye(3), dist_coefs=np.zeros(5)):
		self.camera_matrix = camera_matrix
		self.dist_coefs = dist_coefs


class FakeWorkerDrone:
	"""
	Class with cf_curr_pos member to trick CV algorithms into thinking this is a
	valid WorkerDrone object, without actually requiring it to connect to a CrazyFlie, etc.
	"""

	POS_Z_ALPHA = 0.8

	def __init__(self, curr_pos=np.array([0, 0, 0], dtype=float)):
		self.cf_curr_pos = curr_pos


class WorkerDrone:
	"""
	Class that implements all the necessary functionality to communicate/interact with a worker drone (CrazyFlie).
	"""

	MAX_INTERVAL_FOR_VEL_ESTIMATION = 0.5  # Max time in seconds between 2 consecutive position updates for us to estimate the velocity (as a difference of pos over time)
	VEL_ALPHA = 0.9
	POS_Z_ALPHA = 0.8  # Make sure to change FakeWorkerDrone as well for calibrate_2d_to_3d to display the same results
	POS_OFFSET = np.array([3.1, 0.8, 0.6])
	DERIV_FILT_WIN_HALF_SIZE = 11
	DERIV_FILT_POLY_ORDER = 2
	PRINT_LOGS = False

	def __init__(self, link_uri, experiment_start_datetime, log_folder=None):
		self.cf_radio_ch = link_uri.split("/")[-2]  # Extract CF radio channel number from uri (eg: "radio://0/80/250K")
		self.experiment_log = None
		self.ini_logs(experiment_start_datetime if log_folder is not None else "{}/{}".format(self.cf_radio_ch, experiment_start_datetime), log_folder if log_folder is not None else "log")
		self.cf_log_attitude = self.cf_log_accel = self.cf_log_gyro = None
		self.cf_logs = []
		self.cf_radio_connecting = True
		self.cf_radio_connected = False
		self.cf_pos_tracked = False
		self.ignore_cam_until = datetime.now()
		self.command_info = None
		self.cf_str_status = "Waiting for Matlab..."
		self.cf_roll = self.cf_pitch = self.cf_yaw = self.aX_world = self.cnt_iteration = 0
		self.cf_curr_pos = np.array([0.0, 0.0, 0.0])
		self.cf_curr_pos_t = datetime.now() - timedelta(seconds=self.MAX_INTERVAL_FOR_VEL_ESTIMATION)
		self.cf_past_pos = []
		self.cf_past_pos_t = []
		self.cf_curr_vel = np.array([0.0, 0.0, 0.0])
		self.cf_curr_acc = np.array([0.0, 0.0, 0.0])
		self.droneId = 0
		self.experiment_log.update(droneId=self.droneId)
		self.pause_writing_logs = False
		self.crazyflie = Crazyflie(ro_cache="cachero", rw_cache="cacherw")  # Create an instance of Crazyflie
		self.crazyflie.connected.add_callback(self.on_cf_radio_connected)  # Set up callback functions for communication feedback
		self.crazyflie.disconnected.add_callback(self.on_cf_radio_disconnected)
		self.crazyflie.connection_failed.add_callback(self.on_cf_radio_conn_failed)
		self.crazyflie.connection_lost.add_callback(self.on_cf_radio_conn_lost)
		self.crazyflie.open_link(link_uri)  # Connect to the CrazyRadio through the selected interface

	def ini_logs(self, experiment_start_datetime, log_folder="log"):
		self.experiment_log = plot_tools.ExperimentLog(experiment_start_datetime,
			{"droneId": "mag", "roll": "mag", "pitch": "mag", "yaw": "mag", "gyroX": "mag", "gyroY": "mag", "gyroZ": "mag",
			 "aX_raw": "mag", "aY_raw": "mag", "aZ_raw": "mag", "aX_world": "mag", "aY_world": "mag", "aZ_world": "mag",
			 "pX_cam": "mag", "pY_cam": "mag", "pZ_cam": "mag", "vX_cam": "mag", "vY_cam": "mag", "vZ_cam": "mag", "aX_cam": "mag", "aY_cam": "mag", "aZ_cam": "mag"},
			log_folder, False)

	def setup_cf(self):
		"""
		Sets up the CrazyFlie before running the experiment. This includes configuring some params to default values
		and requesting the CrazyFlie to log certain variables back at constant intervals.
		Doesn't return anything, but raises exceptions if anything goes wrong.
		"""
		try:  # Send some default values for CF params
			#
			self.crazyflie.param.set_value('timeout.timeoutStab', '{:d}'.format(1000*60*10))  # Stabilize (rpy=0) CF if doesn't receive a radio command in 10min
			self.crazyflie.param.set_value('timeout.timeoutShut', '{:d}'.format(1000*60*20))  # Shutdown CF if doesn't receive a radio command in 20min
		except Exception as e:
			raise Exception("Couldn't initialize CrazyFlie params to their desired values. Details: {}".format(e.message))

		# Create a log configuration and include all variables that want to be logged
		self.cf_log_attitude = LogConfig(name="cf_log_attitude", period_in_ms=10)
		self.cf_log_attitude.add_variable("stabilizer.roll", "float")
		self.cf_log_attitude.add_variable("stabilizer.pitch", "float")
		self.cf_log_attitude.add_variable("stabilizer.yaw", "float")
		self.cf_log_accel = LogConfig(name="cf_log_accel", period_in_ms=10)
		self.cf_log_accel.add_variable("acc.x", "float")
		self.cf_log_accel.add_variable("acc.y", "float")
		self.cf_log_accel.add_variable("acc.z", "float")
		self.cf_log_accel.add_variable("stateWorld.ax", "float")
		self.cf_log_accel.add_variable("stateWorld.ay", "float")
		self.cf_log_accel.add_variable("stateWorld.az", "float")
		self.cf_log_gyro = LogConfig(name="cf_log_gyro", period_in_ms=10)
		self.cf_log_gyro.add_variable("gyro.x", "float")
		self.cf_log_gyro.add_variable("gyro.y", "float")
		self.cf_log_gyro.add_variable("gyro.z", "float")
		self.cf_logs = [self.cf_log_attitude, self. cf_log_accel, self.cf_log_gyro]

		try:
			for log in self.cf_logs:
				self.crazyflie.log.add_config(log)  # Validate the log configuration and attach it to our CF
				log.data_received_cb.add_callback(self.on_cf_log_new_data)  # Register appropriate callbacks
				log.error_cb.add_callback(self.on_cf_log_error)
				log.start()
		except Exception as e:
			raise Exception("Couldn't attach the log config to the CrazyFlie, bad configuration. Details: {}".format(e.message))

		# Automatically detect the first yaw log packet and set the current orientation as the desired yaw
		while abs(self.aX_world) < 0.001:  # Wait until first cf_yaw value is received (cf_yaw=0 by default)
			time.sleep(0.1)
		print("Current yaw: {:.2f}.".format(self.cf_yaw))
		# self.crazyflie.param.set_value('baro.aslOffset', '{}'.format(self.cf_estimated_z+0.2))

		self.crazyflie.add_port_callback(CRTPPort.CONSOLE, self.print_cf_console)

	def control_cf(self, new_pos, t_frame):
		"""
		Sends position and/or velocity updates to the worker drone.
		:param new_pos: 1x3 np.array indicating the current Spotter's position estimation of the Worker
		:param t_frame: Datetime at which new_pos was estimated
		"""
		self.cf_pos_tracked = (new_pos is not None and datetime.now() > self.ignore_cam_until)
		dt = (t_frame - self.cf_curr_pos_t).total_seconds()
		if not self.cf_pos_tracked:  # If cv algorithm wasn't able to detect the drone, linearly estimate its position based on previous position and speed
			# self.cf_curr_pos += self.cf_curr_vel*dt
			pass
		else:
			if self.droneId > 0:  # Only increase iteration number when collecting data
				self.cnt_iteration += 1
			self.cf_past_pos.append(new_pos)  # Update history of position and sampling times
			self.cf_past_pos_t.append(t_frame)
			self.experiment_log.update(pX_cam=new_pos[0], pY_cam=new_pos[1], pZ_cam=new_pos[2], timestamp=t_frame)
			if dt < self.MAX_INTERVAL_FOR_VEL_ESTIMATION:
				# self.cf_curr_vel = self.VEL_ALPHA * self.cf_curr_vel + (1 - self.VEL_ALPHA) * (new_pos - self.cf_curr_pos) / dt
				self.cf_curr_vel = plot_tools.DerivativeHelper.differentiate(self.cf_past_pos, self.cf_past_pos_t, win_half_size=self.DERIV_FILT_WIN_HALF_SIZE, poly_order=self.DERIV_FILT_POLY_ORDER, deriv=1)
				self.cf_curr_acc = plot_tools.DerivativeHelper.differentiate(self.cf_past_pos, self.cf_past_pos_t, win_half_size=self.DERIV_FILT_WIN_HALF_SIZE, poly_order=self.DERIV_FILT_POLY_ORDER, deriv=2)
				self.experiment_log.update(vX_cam=self.cf_curr_vel[0], vY_cam=self.cf_curr_vel[1], vZ_cam=self.cf_curr_vel[2], aX_cam=self.cf_curr_acc[0], aY_cam=self.cf_curr_acc[1], aZ_cam=self.cf_curr_acc[2], timestamp=t_frame)
				# self.send_cf_dr_update(True, self.cf_curr_vel[0], self.cf_curr_vel[1], self.cf_curr_vel[2], self.cnt_iteration)
				str_debug_vel = "vx={v[0]:.2f}m/s, vy={v[1]:.2f}m/s, vz={v[2]:.2f}m/s".format(v=self.cf_curr_vel)
			else:  # Haven't tracked the worker for some time, so I shouldn't use past information to estimate velocity in the (near) future
				self.cf_past_pos = [new_pos]  # "Forget" about past history, last point was too long ago
				self.cf_past_pos_t = [t_frame]
				str_debug_vel = "can't estimate (dt={:.2f}s)".format(dt)
			self.cf_curr_pos = new_pos  # Update the new current position
			self.cf_curr_pos_t = t_frame  # Update the time at which curr_pos was estimated
			if self.PRINT_LOGS:
				print("@{t}, found blob at position: px={p[0]:.2f}m, py={p[1]:.2f}m, pz={p[2]:.2f}m; velocity: {v}; n={n}".format(t=t_frame, p=self.cf_curr_pos, v=str_debug_vel, n=self.cnt_iteration))

	def cleanup_cf(self, save_logs=True):
		"""
		"Cleans up" the worker: stops logging, saves/plots logs if necessary, disconnects drone, etc.
		:param save_logs: Whether or not to plot and save all logs
		"""
		# If we get here, either the user ended the experiment, or an exception occurred. Same action regardless:
		try:
			for l in self.cf_logs:
				if l is not None:  # If we were logging data from the CF, stop it (will reconnect faster next time)
					l.stop()
					l.delete()
			if self.crazyflie is not None:  # If the CF was ever initialized, make sure the communication link is closed
				self.crazyflie.close_link()
		except:
			pass

		if save_logs:  # If the experiment didn't crash before starting (the CF ever took off), plot and save the logs
			self.experiment_log.save()
			self.experiment_log.plot(False)

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
		if self.pause_writing_logs: return  # Ignore log if requested
		if logconf.name == "cf_log_accel":
			self.aX_raw = data['acc.x']
			self.aY_raw = data['acc.y']
			self.aZ_raw = data['acc.z']
			self.aX_world = data['stateWorld.ax']
			self.aY_world = data['stateWorld.ay']
			self.aZ_world = data['stateWorld.az']
			self.experiment_log.update(aX_raw=self.aX_raw, aY_raw=self.aY_raw, aZ_raw=self.aZ_raw, aX_world=self.aX_world, aY_world=self.aY_world, aZ_world=self.aZ_world)
			if self.PRINT_LOGS:
				print("@{t}, accel world: ax={:6.3f}g, ay={:6.3f}g, az={:6.3f}g".format(self.aX_world, self.aY_world, self.aZ_world, t=datetime.now()))
		elif logconf.name == "cf_log_gyro":
			self.gyroX = data['gyro.x']
			self.gyroY = data['gyro.y']
			self.gyroZ = data['gyro.z']
			self.experiment_log.update(gyroX=self.gyroX, gyroY=self.gyroY, gyroZ=self.gyroZ)
		elif logconf.name == "cf_log_attitude":
			self.cf_roll = data['stabilizer.roll']
			self.cf_pitch = data['stabilizer.pitch']
			self.cf_yaw = data['stabilizer.yaw']
			# print "\rCurrent yaw: {:.2f}deg".format(self.cf_yaw),
			self.experiment_log.update(roll=self.cf_roll, pitch=self.cf_pitch, yaw=self.cf_yaw)

	def print_cf_console(self, packet):
		console_text = packet.data.decode('UTF-8')
		print("Console: {}".format(console_text))

	def send_cf_param(self, complete_name, value):
		"""
		Modified version of crazyflie.param.set_value that sends the packet immediately (instead of using a Thread+Queue)
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

	def get_OSD_text(self, t_frame, t_last_frame, t_start):
		"""
		Generates written debug information (OSD=on-screen display) to be displayed at the bottom of the current frame
		:param t_frame: Datetime at which camera frame was obtained (through datetime.now())
		:param t_last_frame: Datetime at which last camera frame was obtained
		:param t_start: Datetime at which the experiment/flight started (so we can display elapsed time)
		:return: String containing relevant debug information (worker estimated position, velocity, roll-pitch-yaw, etc.)
		"""
		return "Pos: px={p[0]:5.2f}m, py={p[1]:5.2f}m, pz={p[2]:5.2f}m\t\tVel: vx={v[0]:5.2f}m/s, vy={v[1]:5.2f}m/s, vz={v[2]:5.2f}m/s\t\tAcc: ax={a[0]:5.2f}m/s2, ay={a[1]:5.2f}m/s2, az={a[2]:5.2f}m/s2\t\tCmd_start: [{c[0]:5.2f}, {c[1]:5.2f}, {c[2]:5.2f}]\t\tCmd_delta: [{c[3]:5.2f}, {c[4]:5.2f}, {c[5]:5.2f}]\t\t@{} n={:4} (FPS: {:5.2f}) - {}".format(
				str(t_frame-t_start)[3:-3], self.cnt_iteration, 1./(t_frame-t_last_frame).total_seconds(), self.cf_str_status, p=self.cf_curr_pos, v=self.cf_curr_vel, a=self.cf_curr_acc, c=self.command_info if self.command_info is not None else np.zeros(6))


class Spotter:
	"""
	Class that implements all the necessary functionality for a spotter drone (collect camera images, process them,
	find/identify workers, report their position&velocity, send commands, etc.)
	"""

	COLOR_BALL_TRACKED = (255, 0, 0)
	COLOR_BALL_UNTRACKED = (0, 0, 255)
	COLOR_LINE_TRACKED = (255, 0, 0)
	COLOR_LINE_UNTRACKED = (0, 0, 255)
	COLOR_TARGET_TRACKED = (0, 255, 0)
	COLOR_TARGET_UNTRACKED = (0, 0, 255)
	COLOR_INI_COMMAND = (0, 255, 255)
	COLOR_END_COMMAND = (255, 255, 0)
	SETTINGS_ENVIRONMENT = "Secret lab"
	CAMERA_SETTINGS_FILE = "config/cam_settings/Camera settings - USB 2.0 Camera - {}.txt".format(SETTINGS_ENVIRONMENT)
	COLOR_THRESH_SETTINGS_FILE = "config/color_thresh/Color threshold settings - {}.txt".format(SETTINGS_ENVIRONMENT)
	BLOB_DETECTOR_SETTINGS_FILE = "config/blob_detector/Blob detector settings.txt"
	SETTINGS_SEPARATOR = UvcCapture.SETTINGS_SEPARATOR  # We save files in a csv type of way
	CAM_FOCAL_LENGTH_IN_PX = 1250.0
	CF_RADIUS_IN_M = 0.02
	LOG_FOLDER = "../../Mobisys 18 - Paper/Code/data/Real/Ours/"
	NUM_WORKERS = 6

	def __init__(self, experiment_number, starting_iter=1, bool_world_coords_pattern=False):
		self.t_start = self.t_frame = self.t_last_frame = datetime.now()
		self.t_events = []
		self.EXPERIMENT_NUMBER = experiment_number
		self.EXPERIMENT_START_DATETIME = str(self.t_start)[:-7].replace(':', '-')
		self.VIDEO_FOLDER = "img-ns/{}".format(self.EXPERIMENT_START_DATETIME)
		self.experiment_iter = starting_iter
		self.actuation_command = None
		self.curr_droneId_deadline = None
		self.curr_iter_deadline = None
		self.next_droneId = 1
		self.workers = []
		self.kb_controls_which_cf = -1
		self.window_for_kb_input = None
		self.bool_world_coords_pattern = bool_world_coords_pattern
		self.world_to_camera_transf = None
		self.video_capture = None
		self.cv_HSV_thresh_min = np.array([  0,   0,   0], dtype=np.uint8)
		self.cv_HSV_thresh_max = np.array([255, 255, 255], dtype=np.uint8)
		self.cv_blob_detect_params = None
		self.cv_cam_frame = None
		self.cv_filtered_HSV_mask = None
		self.cv_frame_out = None

	def get_experiment_log_folder(self):
		return "{}experiment{}/".format(self.LOG_FOLDER, self.EXPERIMENT_NUMBER)

	def get_iteration_log_folder(self):
		return "{}iteration{}/".format(self.get_experiment_log_folder(), self.experiment_iter)

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
		self.video_capture.do_undistort = False  # We don't need to undistort the image, camera_matrix will take care of it
		logging.info("Camera opened! :)")

		# Initialize cv algorithm too: load settings for color thresholding and blob detector
		self.load_color_thresh_settings()
		self.load_blob_detector_settings()

		# Sometimes, first couple frames take a long time to be obtained, do it before quad goes in flying mode
		self.video_capture.get_frame_robust()

		# If requested, find world_to_camera_transf and F
		pre_experiment_find_extrinsics(self, self.bool_world_coords_pattern)

		# Prepare the folder self.VIDEO_FOLDER so we can store each frame we processed (for debugging)
		if create_video_folder:
			# shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=True)  # Delete the folder and its contents, if it exists (ignore errors if it doesn't)
			try:
				os.makedirs(self.VIDEO_FOLDER)  # Now create the folder, which won't throw any exceptions as we made sure it didn't already exist
			except:
				pass

	def init_UI_window(self):
		"""
		Initializes the appropriate user interface depending on experiment's purpose (see is_input_for_drone_commands)
		:param is_input_for_drone_commands: True if we have communication with the CF and need input to send it commands
		False if we're just debugging the computer vision side (camera settings, color threshold, etc.) and need input
		to debug pixel information
		"""
		# Open an SDL2 window to track keyboard keydown events and use it to send commands to the CF
		sdl2.ext.init()
		self.window_for_kb_input = sdl2.ext.Window("Window to receive keyboard input", size=(400, 300), position=(200,200))
		#self.window_for_kb_input.show()  # Don't show until Matlab command is loaded
		sdl2.ext.fill(self.window_for_kb_input.get_surface(), sdl2.ext.Color(255,0,0))
		self.window_for_kb_input.refresh()

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
					name, value = line.rstrip('\r\n').split(sep)
					setattr(detector_params, name, eval(value))  # Use eval to cast to right type (eg: False instead of "False")
					logging.debug("\tLoaded blob detector setting: '{}' = {}".format(name, value))
		except:
			logging.exception("Something went wrong while loading blob detector settings from '{}'.".format(file_name))
			return False
		return True

	def connect_to_cf(self, connect_to=1, max_timeout=20):
		"""
		Initializes radio drivers, looks for available CrazyRadios, and attempts to connect to the first available one.
		Doesn't return anything, but raises exceptions if anything goes wrong.
		:param connect_to: If int, specifies how many CFs should we be connected to before proceeding with the experiment;
		If a list, specifies the interface names of the CFs to connect to (eg: ["radio://0/80/250K"]).
		:param max_timeout: Time in seconds after which we should give up if we haven't been able to establish comm. yet
		"""
		logging.info("Initializing drivers.")
		crtp.init_drivers()

		logging.info("Setting up the communication link(s). Looking for available CrazyRadios in range.")
		t_deadline = datetime.now() + timedelta(seconds=max_timeout)
		connect_to_num = connect_to if isinstance(connect_to, int) else len(connect_to)
		while len(self.workers) < connect_to_num:
			if datetime.now() > t_deadline:  # Abort after max_timeout seconds if still haven't connected to all of them
				raise Exception("Unable to establish communication with {} CrazyFlie{} after {}s. Given up :(".format(connect_to_num, "s" if connect_to_num > 1 else "", max_timeout))

			available_links = crtp.scan_interfaces()
			if len(available_links) == 0:
				logging.warning("Warning, no Crazyflies found. Trying again in a second.")
				time.sleep(1)
				continue

			logging.info("CrazyFlies found:")  # For debugging purposes, show info about available links
			link_uri = None
			for i in available_links:
				logging.info("\t{}".format(i[0]))
				if link_uri is None and isinstance(connect_to, list) and i[0] in connect_to:  # If we provided a list, look for the first appearance among available interfaces
					link_uri = i[0]  # Save the URI of this interface
					connect_to.remove(link_uri)  # And remove it from the list so we don't try to connect again
					break
			if isinstance(connect_to, int):  # If connect_to is a number, choose first available link
				link_uri = available_links[0][0]

			logging.info("Initializing CrazyFlie (connecting to '{}').".format(link_uri))
			self.workers.append(WorkerDrone(link_uri, "experiment{}/iteration{}".format(self.EXPERIMENT_NUMBER, self.experiment_iter), self.LOG_FOLDER))

		# If we got here, we're attempting to connect to connect_to_num CFs. Let's make sure connection succeeds on time
		while np.any([w.cf_radio_connecting for w in self.workers]):
			if datetime.now() > t_deadline:  # Abort after max_timeout seconds if still haven't connected to all of them
				raise Exception("Unable to establish communication with {} CrazyFlie{} after {}s. Given up :(".format(connect_to_num, "s" if connect_to_num > 1 else "", max_timeout))
			time.sleep(1)

		# If we got here, no CFs are connecting. This means they're either successfully connected, or not (an error occurred)
		if not np.all([w.cf_radio_connected for w in self.workers]):
			raise Exception("Something failed while attempting to connect to {} CrazyFlie{}, exiting :(".format(connect_to_num, "s" if connect_to_num > 1 else ""))

	def fly_cf(self):
		"""
		Provides the structure to make the drone fly (actual flight control is done in hover() though).
		hover() is called until user stops the experiment, then hover is called for a few more secs to record more data
		(usually includes information about the CF crashing...)
		:return: Whether or not to keep (store) the logs, based on whether the CF ever actually took off and flew
		"""

		# t = Timer(10, self.cleanup_experiment)  # Start a timer to automatically disconnect in 10s
		# t.start()

		# Prepare for take off: start logging PIDs, save start time...
		self.t_start = datetime.now()

		# Alright, let's fly!
		tStop = None
		while tStop is None:  # tStop will remain None while everything's fine
			tStop = self.hover()  # hover returns None while everything's fine; the time to end the experiment otherwise

		# If we get here, either the user stopped the experiment or the code detected something went wrong
		print("AT t={}, A KEY WAS PRESSED -> STOPPING!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3]))
		save_logs = True  # Make sure we save the data

		return save_logs  # Return whether or not to keep the logs

	def cleanup_experiment(self, save_logs=True):
		"""
		"Cleans up" the experiment: closes any open windows, saves logs, disconnects camera and drone, etc.
		:param save_logs: Whether or not to plot and save all logs
		"""
		for w in self.workers:
			w.cleanup_cf(save_logs)
		self.video_capture = None  # Destroy the video capture object (this takes care of closing the camera etc.)
		cv2.destroyAllWindows()  # Close all UI windows that could be open
		if self.window_for_kb_input is not None:
			self.window_for_kb_input.hide()
			sdl2.ext.quit()

		if not save_logs:  # Delete the folder (and its contents) where cam frames would have been/were saved if we don't want to save the logs
			shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=True)

	def run_experiment(self, connect_to=1):
		"""
		Spotter's main method: initializes all components (vision, communication, drone params, etc.) then
		runs the experiment.
		:param connect_to: If int, specifies how many CFs should we be connected to before proceeding with the experiment;
		If a list, specifies the interface names of the CFs to connect to (eg: ["radio://0/80/250K"]).
		"""
		logging.disable(logging.DEBUG)  # Seems to work better than .basicConfig(INFO), especially if logging has already been initialized -> Only show messages of level INFO or higher

		save_logs = True
		try:
			self.connect_to_cf(connect_to)  # Connect to the CrazyFlie
			for w in self.workers:
				w.setup_cf()  # Can't initialize LogConfig until we're connected, because it needs to check that the variables we'd like to add are in the TOC. So this function must be called after connect_to_cf()
			self.init_video_cam_and_cv_algorithm()  # Connect to the first available camera, load default settings, etc.
			self.init_UI_window()  # Open a window to receive user input to control the CF
			for w in self.workers:
				try:
					np.savez_compressed(os.path.join(self.get_experiment_log_folder(), "log_experiment_constants.npz"),
						spotter_camera_matrix=self.video_capture.camera_matrix, spotter_dist_coefs=self.video_capture.dist_coefs, spotter_frame_rate=self.video_capture.frame_rate, spotter_frame_size=self.video_capture.frame_size,
						spotter_world_to_camera_transf=self.world_to_camera_transf, spotter_F=self.CAM_FOCAL_LENGTH_IN_PX, spotter_HSV_thresh_min=self.cv_HSV_thresh_min, spotter_HSV_thresh_max=self.cv_HSV_thresh_max,
						worker_max_interval_for_vel_estimation=w.MAX_INTERVAL_FOR_VEL_ESTIMATION, worker_vel_alpha=w.VEL_ALPHA,
						worker_deriv_filt_win_half_size=w.DERIV_FILT_WIN_HALF_SIZE, worker_deriv_filt_poly_order=w.DERIV_FILT_POLY_ORDER)
				except:
					raise Exception("START MATLAB FIRST!!!")
			save_logs = self.fly_cf()  # And finally fly the CF
		except:
			logging.exception("Shutting down due to an exception =( See details below:")

		# If we got here, either the user ended the experiment, or an exception occurred. Same action regardless:
		self.cleanup_experiment(save_logs)  # "Cleanup" the experiment: close windows, save logs, etc.

	# def img_to_cf_world_coords(self, cam_coords):
	# 	if cam_coords is None:
	# 		return None
	#
	# 	pX = -Spotter.CAM_FOCAL_LENGTH_IN_PX				* Spotter.CF_RADIUS_IN_M/cam_coords[2]
	# 	pY = (cam_coords[0] - self.cv_cam_frame.shape[1]/2)	* Spotter.CF_RADIUS_IN_M/cam_coords[2]
	# 	pZ = (self.cv_cam_frame.shape[0]/2 - cam_coords[1])	* Spotter.CF_RADIUS_IN_M/cam_coords[2]
	# 	return np.array([pX, pY, pZ])
	#
	# def cf_world_to_img_coords(self, world_coords):
	# 	if world_coords is None:
	# 		return None
	#
	# 	pX = self.cv_cam_frame.shape[1]/2 - world_coords[1]*Spotter.CAM_FOCAL_LENGTH_IN_PX/world_coords[0]
	# 	pY = self.cv_cam_frame.shape[0]/2 + world_coords[2]*Spotter.CAM_FOCAL_LENGTH_IN_PX/world_coords[0]
	# 	r = -Spotter.CF_RADIUS_IN_M						   *Spotter.CAM_FOCAL_LENGTH_IN_PX/world_coords[0]
	# 	return np.array([pX, pY, r])

	def img_to_cf_world_coords(self, img_coords):
		"""
		Obtains the world coordinates of a worker whose marker has been found by the CV algorithm.
		:param img_coords: 1x3 np.array, where the first 2 components represent the (u,v) image coordinates of the worker
		drone (found by the CV algorithm, in pixels) and the 3rd component is the radius in pixels of the marker/sphere.
		:return: CrazyFlie-transformed (which is a right-handed coordinate system, so y- and z- axes need to be inverted)
		world coordinates of a worker drone.
		"""
		if img_coords is None:
			return None

		depth_in_m = Spotter.CAM_FOCAL_LENGTH_IN_PX * Spotter.CF_RADIUS_IN_M / img_coords[2]
		cam_coords = auxV.img_to_cam_coords(np.hstack((img_coords[0:2], depth_in_m)), self.video_capture.camera_matrix, self.video_capture.dist_coefs)
		world_coords = auxV.cam_to_world_coords(cam_coords, self.world_to_camera_transf)
		return np.array([1, -1, -1]) * world_coords  # Flip y and z axes sign to convert world->cf coords

	def cf_world_to_img_coords(self, cf_world_coords):
		"""
		Obtains the projected image coordinates of a worker whose world position (in CF-world coordinate system) is known.
		:param cf_world_coords: 1x3 np.array containing the x, y, z CF-world coordinates of the worker drone
		(CrazyFlie uses a right-handed coordinate system, so y- and z- axes need to be inverted to obtain actual world coords)
		:return: 1x3 np.array whose first 2 components represent the (u,v) pixel location in image coordinates of
		the Spotter's camera and the last component indicates the radius in pixels of the worker's marker/sphere.
		"""
		if cf_world_coords is None:
			return None
		world_coords = np.array([1, -1, -1]) * cf_world_coords  # Flip y and z axes sign to convert cf->world coords

		cam_coords = auxV.world_to_cam_coords(world_coords, self.world_to_camera_transf)
		img_coords = auxV.cam_to_img_coords(cam_coords, self.video_capture.camera_matrix, self.video_capture.dist_coefs)
		r = Spotter.CF_RADIUS_IN_M * Spotter.CAM_FOCAL_LENGTH_IN_PX/cam_coords[2]
		return np.hstack((img_coords, r))

	def process_kb_input(self):
		"""
		Processes all keydown events, and takes the corresponding action depending on which key was pressed.
		:return: Boolean indicating whether the experiment shall go on: True while everything's fine, False to stop it
		"""
		events = sdl2.ext.get_events()  # Fetch any new input event
		if any([w.droneId > 0 for w in self.workers]): return True  # Don't listen to keyboard while collecting data!!

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
					elif key_orig == sdl2.SDLK_LEFT:
						key = "Left"
					elif key_orig == sdl2.SDLK_RIGHT:
						key = "Right"
					elif key_orig == sdl2.SDLK_RETURN:
						key = "Enter"
					elif key_orig == sdl2.SDLK_ESCAPE:
						key = "Esc"
					else:
						key = hex(key_orig)

				logging.info("Key: '{}'".format(key))
				if '0' <= key <= '9':  # Decide which CF we're going to control (0 controls them all)
					self.kb_controls_which_cf = int(key)-1
					if self.kb_controls_which_cf >= len(self.workers):
						self.kb_controls_which_cf = -1
				self.window_for_kb_input.title = "Controlling CF {}. Last key pressed: '{}' at t={}".format(self.kb_controls_which_cf, key, str(datetime.now().time())[:-3])
				if '0' <= key <= '9': continue
				key = key.lower()  # Convert to lowercase so we don't have to worry about different cases

				if key=='n' and self.actuation_command is not None:  # Only listen to "next" commands after Matlab has issued a new command
					cv2.destroyAllWindows()  # Close windows so fps doesn't drop
					self.window_for_kb_input.hide()
					for w in self.workers:
						w.droneId = self.next_droneId
						w.experiment_log.update(droneId=w.droneId)
						self.next_droneId = 1 + (self.next_droneId % self.NUM_WORKERS)
						self.curr_droneId_deadline = datetime.now() + timedelta(seconds=2)
						self.window_for_kb_input.title = "'{}' -> Drone {} flying (iter {}), at t={}".format(key, w.droneId, self.experiment_iter, str(datetime.now().time())[:-3])
					return True  # Don't listen to more keys until we're done collecting data
				else:  # Any other key ends the experiment
					for w in self.workers:
						w.cf_radio_connected = False
						w.cf_str_status = "DONE"
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
		self.cv_filtered_HSV_mask = cv2.morphologyEx(self.cv_filtered_HSV_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
		self.cv_filtered_HSV_mask = cv2.morphologyEx(self.cv_filtered_HSV_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=6)
		self.t_events.append(datetime.now())

		#########################################################
		#                     DRONE DETECTION                   #
		#########################################################
		if find_blob:
			keypoints = cv2.SimpleBlobDetector_create(self.cv_blob_detect_params).detect(self.cv_filtered_HSV_mask)
			self.t_events.append(datetime.now())
			if keypoints:  # If the cv algorithm detected at least one blob
				# keypoints = sorted(keypoints, key=attrgetter('size'), reverse=True)  # Sort bigger blobs first
				# keypoint = max(keypoints, key=attrgetter('size'))  # Focus on the biggest blob
				# return [np.hstack((keypoint.pt, keypoint.size/2))]*len(self.workers)  # And save the position estimated by the CV algorithm
				kp_pos3d = np.empty((len(keypoints), 3))
				for i, kp in enumerate(keypoints):
					kp_pos3d[i,:] = self.img_to_cf_world_coords(np.hstack((kp.pt, kp.size/2)))
				output = []
				for w in self.workers:
					w_img_coords = self.cf_world_to_img_coords(w.cf_curr_pos-w.POS_OFFSET)
					closest_drone = self.cf_world_to_img_coords(kp_pos3d[np.argmin(np.sum((w.cf_curr_pos - kp_pos3d) ** 2, axis=1)), :])
					if not all(w.cf_curr_pos == 0):  # Let it initialize
						closest_drone[2] = w.POS_Z_ALPHA * w_img_coords[2] + (1 - w.POS_Z_ALPHA) * closest_drone[2]  # Smooth depth
					if isinstance(w, FakeWorkerDrone):
						w.cf_curr_pos = self.img_to_cf_world_coords(closest_drone)
					output.append(closest_drone)
				return output
			# circles = cv2.HoughCircles(self.cv_filtered_HSV_mask, cv2.HOUGH_GRADIENT, 2, 10, param2=15, maxRadius=500)
			# if circles is not None:
			# 	circles = circles[0]
			# 	# circles = circles[np.flipud(circles[:,2].argsort())]
			# 	return [circles[0,:]]*len(self.workers)  # And save the position estimated by the CV algorithm
		return [None]*len(self.workers)

	def save_algo_iteration(self, str_OSD="", newline_separator='\t\t', margin_x=25, margin_y=25, text_color=(200, 200, 200), font=cv2.FONT_HERSHEY_DUPLEX, font_scale=0.7, font_thickness=1, line_type=cv2.LINE_AA, mask_color=(255, 0, 255), img_resize_factor=0.5, save_cam_frame_before_resizing=True):
		"""
		Overlays debug information on the captured camera frame (HSV mask, identified workers, debug OSD text, etc.)
		and saves the resulting image to a file.
		"""
		if str_OSD == "":  # Allow for custom OSD text, but if no text specified, print the default debug info (get_OSD_text)
			str_OSD = self.workers[0].get_OSD_text(self.t_frame, self.t_last_frame, self.t_start)
			self.t_events.append(datetime.now())

		# Resize camera frame and CrazyFlie's current&target positions according to img_resize_factor
		frame_resized = cv2.resize(self.cv_cam_frame, None, fx=img_resize_factor, fy=img_resize_factor)
		mask_resized = cv2.resize(self.cv_filtered_HSV_mask, None, fx=img_resize_factor, fy=img_resize_factor)

		# Save the original camera frame to disk (for post-debugging if necessary)
		###### cv2.imwrite(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("frame_%H-%M-%S-%f.jpg")), self.cv_cam_frame if save_cam_frame_before_resizing else frame_resized)
		self.t_events.append(datetime.now())

		# Plot OSD related to CF's current and target positions (2 circles and a connecting line)
		for w in self.workers:
			curr_pos_resized = self.cf_world_to_img_coords(w.cf_curr_pos-w.POS_OFFSET) * img_resize_factor
			if w.cf_pos_tracked:
				cv2.circle(frame_resized, tuple(curr_pos_resized[0:2].astype(int)), int(curr_pos_resized[2]), self.COLOR_BALL_TRACKED if w.cf_pos_tracked else self.COLOR_BALL_UNTRACKED, -1)
		self.t_events.append(datetime.now())

		# On top of that, overlay the HSV mask (so we can debug color filtering + blob detection steps)
		np.putmask(frame_resized, cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR).astype(bool), list(mask_color))
		for i,w in enumerate(self.workers):
			# Draw command info ON TOP of the mask
			if w.command_info is not None:
				command_ini_pos_resized = self.cf_world_to_img_coords(w.command_info[:3] - w.POS_OFFSET) * img_resize_factor
				command_end_pos_resized = self.cf_world_to_img_coords(w.command_info[:3] + w.command_info[3:] - w.POS_OFFSET) * img_resize_factor
				cv2.circle(frame_resized, tuple(command_end_pos_resized[0:2].astype(int)), int(command_end_pos_resized[2]), self.COLOR_END_COMMAND, -1)
				cv2.circle(frame_resized, tuple(command_ini_pos_resized[0:2].astype(int)), int(command_ini_pos_resized[2]), self.COLOR_INI_COMMAND, 5)

			if not w.cf_pos_tracked: continue
			curr_pos_resized = self.cf_world_to_img_coords(w.cf_curr_pos) * img_resize_factor
			str_cf_id = "{}".format(i+1)  # w.cf_radio_ch
			txt_size = np.array(cv2.getTextSize(str_cf_id, font, font_scale, font_thickness)[0])
			txt_pos = curr_pos_resized[0:2] - txt_size/2
			if all(abs(txt_pos) < 3000):  # Avoid drawing text out of bounds
				cv2.putText(frame_resized, "{}".format(i+1), tuple((curr_pos_resized[0:2]-txt_size/2).astype(int)), font, font_scale, text_color, font_thickness, line_type)
		self.t_events.append(datetime.now())

		# Generate the output image: upper part is the cam frame downsized according to img_resize_factor; lower part, str_OSD
		lines = str_OSD.split(newline_separator)
		self.cv_frame_out = np.zeros(((frame_resized.shape[0] + margin_y*(len(lines)+1)), frame_resized.shape[1], frame_resized.shape[2]), dtype=frame_resized.dtype)
		self.cv_frame_out[0:frame_resized.shape[0], :] = frame_resized
		self.t_events.append(datetime.now())

		for cnt, l in enumerate(lines):  # Add every line of text in str_OSD. Note that putText asks for bottom-left corner of text and that cnt=0 for 1st line. Therefore vertical component should be frame_resized height + OSD padding/border (0.5*margin_y) + text height (1*margin_y)
			cv2.putText(self.cv_frame_out, l.replace('\t', '; '), (margin_x, frame_resized.shape[0] + int(margin_y*(cnt+1.4))), font, font_scale, text_color, font_thickness, line_type)

		# Save the output image to disk (for post-debugging if necessary)
		if self.workers[0].droneId > 0:
			cv2.imwrite(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("out_%H-%M-%S-%f.jpg")), self.cv_frame_out)
		else:
			cv2.imshow("", self.cv_frame_out)
			key = cv2.waitKey(1)
			if key == ord('q'):
				return False
			if any([w.command_info is not None for w in self.workers]):  # Only show kb if there are any workers ready to receive a command
				self.window_for_kb_input.show()
		self.t_events.append(datetime.now())
		return True

	def load_actuation_command_if_needed(self):
		# Only load the command if it hasn't been loaded yet
		if self.actuation_command is not None:
			return

		newCommand_filename = os.path.join(self.get_iteration_log_folder(), "newCommand.txt")
		if os.path.isfile(newCommand_filename):
			with open(newCommand_filename, "r") as f:  # Matlab wrote the file! Let's read it and check if it has finished writing
				lines = [line.strip() for line in f.readlines()]
				if len(lines)<2 or lines.pop()!="Done":  # Wait until the whole file is written (don't want to run into race conditions)
					return  # -> Last line needs to say "Done" otherwise exit (will try again later)

			# Matlab is done writing, parse and save the command
			self.actuation_command = np.array([ [float(x) for x in line.split(',')] for line in lines ])
			self.NUM_WORKERS = self.actuation_command.shape[0]  # Figure out number of workers based on number of lines in the file
			for w in self.workers:
				self.set_worker_command_info(w)

	def set_worker_command_info(self, w):
		w.command_info = self.actuation_command[self.next_droneId-1, :]  # Load info of next command
		w.cf_str_status = "Exp{} i{:02d} d{}".format(self.EXPERIMENT_NUMBER, self.experiment_iter, self.next_droneId)

	def hover(self):
		"""
		Executes the workflow of a single Spotter iteration: grabs a camera frame, identifies workers,
		estimates their position, sends them individual updates, overlays debug/mask information on the camera frame,
		saves it to a file and processes any keyboard input.
		:return: None if everything went well; Datetime with the time at which to stop the flight if something went wrong
		(this allows to save debug information for a few extra seconds even when the drones stopped flying and maybe see better why they crashed)
		"""

		# Check if Matlab has issued a new command
		self.load_actuation_command_if_needed()

		try:  # First, run the cv algorithm to estimate the CF's position
			new_pos_arr = self.detect_cf_in_camera()
		except:  # Only way detect_cf_in_camera raises an Exception is if a camera frame couldn't be grabbed
			logging.exception("Couldn't grab a frame from the camera. Exiting")
			return datetime.now()  # Need to stop now (if I wanted to stop in now+2sec and camera kept throwing exceptions, it would keep delaying the stop and never actually stop)

		# Now that we know where the drone currently is, send messages to control it (roll-pitch-yaw-thrust setpoints)
		for i,w in enumerate(self.workers):
			new_pos_world = self.img_to_cf_world_coords(new_pos_arr[i])
			if new_pos_world is not None: new_pos_world += w.POS_OFFSET  # Add offset to set custom {0,0,0} world origin
			w.control_cf(new_pos_world, self.t_frame)

			# Check if 1s has passed so we stop logging for this drone
			if self.curr_droneId_deadline is not None and datetime.now() > self.curr_droneId_deadline:
				self.curr_droneId_deadline = None  # Make sure we don't enter this block again

				# Check if this is the last drone in this iteration
				if self.next_droneId == 1:  # Equivalent to: w.droneId == self.NUM_WORKERS
					self.curr_iter_deadline = datetime.now() + timedelta(seconds=1.5)  # Give it a couple seconds just in case before ending the iteration and saving logs
					w.command_info = None
					w.cf_str_status = "Saving iteration {}...".format(self.experiment_iter)
				else:
					w.command_info = self.actuation_command[self.next_droneId-1, :]  # Load info of next command
					w.cf_str_status = "Exp{} i{:02d} d{}".format(self.EXPERIMENT_NUMBER, self.experiment_iter, self.next_droneId)


				w.droneId = 0  # And stop "collecting data"
				w.experiment_log.update(droneId=w.droneId)

			# Check if we should save the logs and move on to the next iteration
			if self.curr_iter_deadline is not None and datetime.now() > self.curr_iter_deadline:
				self.curr_iter_deadline = None  # Make sure we don't enter this block again

				# Save the logs
				w.pause_writing_logs = True
				w.experiment_log.save()
				#w.experiment_log.plot(False)
				w.pause_writing_logs = False

				self.actuation_command = None  # Wait until Matlab issues new command
				open("{}dataCollectionDone.txt".format(self.get_iteration_log_folder()), 'w').close()  # Create an empty file to signal Matlab to issue a new command

				self.experiment_iter += 1
				if self.experiment_iter > 30:  # End experiment after 30 iterations
					return datetime.now()
				else:  # Start new logs
					w.ini_logs("experiment{}/iteration{}".format(self.EXPERIMENT_NUMBER, self.experiment_iter), self.LOG_FOLDER)
		self.t_events.append(datetime.now())

		# Last, save output images to disk for debugging and process kb input to control the experiment
		if not self.save_algo_iteration() or not self.process_kb_input():  # Process kb input and take action if necessary (will return False when user wants to stop the experiment)
			return datetime.now() + timedelta(seconds=2)  # For debugging purposes, it's great to have a few additional seconds of video&log after an experiment is stopped (helps see why it crashed)
		self.t_events.append(datetime.now())

		logging.debug("DeltaT = {:5.2f}ms -> Total: {:5.2f}ms{}".format(
			(datetime.now() - self.t_frame).total_seconds()*1000, (self.t_frame - self.t_last_frame).total_seconds()*1000,
			"".join(["\t{}->{}: {}ms".format(i+1, i+2, (self.t_events[i+1]-self.t_events[i]).total_seconds()*1000) for i in range(len(self.t_events) - 1)])))
		self.t_last_frame = self.t_frame  # Remember the time this frame was taken so we can estimate FPS in next iteration

		return None  # Return None to indicate that the experiment shall go on. All good.


if __name__ == '__main__':
	s = Spotter(experiment_number=1, starting_iter=1, bool_world_coords_pattern=True)
	s.run_experiment(['radio://0/75/2M'])
