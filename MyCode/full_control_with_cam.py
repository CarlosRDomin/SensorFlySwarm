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


class GroundTruthThread(threading.Thread):
	"""
	Thread that handles ground-truth collection
	"""

	SETTINGS_ENVIRONMENT = "Lab"
	CAMERA_SETTINGS_FILE = "config/cam_settings/Camera settings - Ground truth camera - {}.txt".format(SETTINGS_ENVIRONMENT)
	TEXT_SEPARATOR = '\t'
	CAM_FOCAL_LENGTH_IN_PX = 1450

	def __init__(self, video_folder, file_prefix):
		super(GroundTruthThread, self).__init__()
		self.VIDEO_FOLDER = video_folder
		self.FILE_PREFIX = file_prefix
		self.video_capture = None
		self.video_writer = None
		self.text_file = None
		self.world_to_camera_transf = None
		self.cam_frame = None
		self.t_frame = None
		self.done = False
		self.init_video_cam()

	def init_video_cam(self):
		self.video_capture = UvcCapture.new_from_settings(self.CAMERA_SETTINGS_FILE)  # Connect to device specified by settings, and load its desired param values
		if self.video_capture is None:  # If unable to connect to device specified by settings, open first available camera
			self.video_capture = UvcCapture(0)
			if self.video_capture is None:  # If still unable to connect, raise an exception
				raise Exception("Couldn't open camera! :(")

			# If we're here, we couldn't connect to device specified by settings but were able to open 1st available cam
			if not self.video_capture.load_settings(self.CAMERA_SETTINGS_FILE):  # Try to load frame size & rate from settings
				self.video_capture.select_best_frame_mode(60)  # If loading settings failed, choose best frame size with fps >= 60
		self.video_capture.do_undistort = False
		logging.info("Ground truth camera opened! :)")

		# Sometimes, first couple frames take a long time to be obtained, do it before quad goes in flying mode
		self.video_capture.get_frame_robust()
		self.video_capture.get_frame_robust()

		# Prepare the folder self.VIDEO_FOLDER so we can store each frame we processed (for debugging)
		shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=True)  # Delete the folder and its contents, if it exists (ignore errors if it doesn't)
		os.makedirs(self.VIDEO_FOLDER)  # Now create the folder, which won't throw any exceptions as we made sure it didn't already exist

	def stop(self):
		self.done = True
		self.join()

	def run(self):
		self.video_writer = cv2.VideoWriter(os.path.join(self.VIDEO_FOLDER, "gt_{}.avi".format(self.FILE_PREFIX)), cv2.VideoWriter_fourcc(*'X264'), self.video_capture.frame_rate, self.video_capture.frame_size)  # Lossless codec: HFYU
		self.text_file = open(os.path.join(self.VIDEO_FOLDER, "gt_{}.txt".format(self.FILE_PREFIX)), "w")
		# img = np.empty(self.video_capture.frame_size[::-1] + (3,), dtype=np.uint8)  # Allocate a buffer for a frame's BGR image
		index = 100  # Store its index (to keep track of how many frames we skip)
		while not self.done:
			try:
				self.cam_frame = self.video_capture.get_frame_robust()  # read() blocks execution until a new frame arrives! -> Obtain t AFTER grabbing the frame
				self.t_frame = datetime.now()
				img = self.cam_frame.bgr  # np.copyto(img, self.cam_frame.bgr)  # For codec HFYU, we have to copy the bgr image into array img. Otherwise it causes a segmentation fault (don't know why!)
				t2 = datetime.now()
				self.video_writer.write(img)
				# cv2.imwrite(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("gt_%H-%M-%S-%f.jpg")), self.cam_frame.bgr)
				# np.save(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("gt_%H-%M-%S-%f.npy")), self.cam_frame.bgr)
				t3 = datetime.now()
				if self.cam_frame.index > index + 1:
					logging.warning("\t\t\t\t\t\t\t\tHEEEEY, skipped {} frame(s)!!! :(".format(self.cam_frame.index-index-1))
				index = self.cam_frame.index
				self.text_file.write("{}{}{}\n".format(index, self.TEXT_SEPARATOR, self.t_frame))
				logging.info("Saved frame {} at {}. Took {:.2f}ms to save! ({:.2f}ms to copy; {:.2f}ms to write)".format(self.cam_frame.index, str(self.t_frame)[:-3], (datetime.now()-self.t_frame).total_seconds()*1000, (t2-self.t_frame).total_seconds()*1000, (t3-t2).total_seconds()*1000))
			except Exception as e:
				raise Exception("Unexpected error accessing the ground truth camera frame :( Details: {}.".format(e.message))

		self.text_file.close()
		self.video_writer.release()
		self.video_capture = None  # Destroy the video capture object (this takes care of closing the camera etc.)


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

	def __init__(self, curr_pos=np.array([0, 0, 0], dtype=float)):
		self.cf_curr_pos = curr_pos


class WorkerDrone:
	"""
	Class that implements all the necessary functionality to communicate/interact with a worker drone (CrazyFlie).
	"""

	MAX_INTERVAL_FOR_VEL_ESTIMATION = 0.5  # Max time in seconds between 2 consecutive position updates for us to estimate the velocity (as a difference of pos over time)
	VEL_ALPHA = 0.9
	VEL_DERIV_WIN_SIZE = 5
	VEL_DERIV_POLY_ORDER = 2
	TAKEOFF_THRUST = 45000
	POS_Z_OFFSET = 5.00
	CF_TARGET_POS_SEND_PERIOD = timedelta(seconds=0.5)  # Resend target pos every 500ms
	USING_KALMAN = False

	def __init__(self, link_uri, experiment_start_datetime):
		self.cf_radio_ch = link_uri.split("/")[-2]  # Extract CF radio channel number from uri (eg: "radio://0/80/250K")
		self.experiment_log = plot_tools.ExperimentLog("{}/{}".format(self.cf_radio_ch, experiment_start_datetime),
				{"Yaw": "log", "pX": "piv", "pY": "piv", "pZ": "piv", "experiment_running": "mag", "pos_tracked": "mag", "update_DR": "mag"})
		self.cf_log_attitude = self.cf_log_PID_x = self.cf_log_PID_y = self.cf_log_PID_z = None
		self.cf_logs = []
		self.cf_radio_connecting = True
		self.cf_radio_connected = False
		self.cf_pos_tracked = False
		self.ignore_cam_until = datetime.now()
		self.cf_taking_off = True
		self.cf_str_status = "TAKING OFF"
		self.cf_roll = self.cf_pitch = self.cf_yaw = self.cf_target_yaw = self.cnt_iteration = 0
		self.cf_target_pos = np.array([0.0, 0.0, 0.0])
		self.cf_curr_pos = np.array([0.0, 0.0, 0.0])
		self.cf_curr_pos_t = datetime.now() - timedelta(seconds=self.MAX_INTERVAL_FOR_VEL_ESTIMATION)
		self.cf_past_pos = []
		self.cf_past_pos_t = []
		self.cf_t_last_target_pos_sent = None
		self.cf_curr_vel = np.array([0.0, 0.0, 0.0])
		self.cf_update_DR = True
		self.experiment_running = False
		self.cf_kalman_ready = not self.USING_KALMAN
		self.log_PID_x = PID.PIDposAndVel()
		self.log_PID_y = PID.PIDposAndVel()
		self.log_PID_z = PID.PIDposAndVel()
		self.log_PID_z.setSetPoint(self.POS_Z_OFFSET)
		self.crazyflie = Crazyflie(ro_cache="cachero", rw_cache="cacherw")  # Create an instance of Crazyflie
		self.crazyflie.connected.add_callback(self.on_cf_radio_connected)  # Set up callback functions for communication feedback
		self.crazyflie.disconnected.add_callback(self.on_cf_radio_disconnected)
		self.crazyflie.connection_failed.add_callback(self.on_cf_radio_conn_failed)
		self.crazyflie.connection_lost.add_callback(self.on_cf_radio_conn_lost)
		self.crazyflie.open_link(link_uri)  # Connect to the CrazyRadio through the selected interface

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
			self.crazyflie.param.set_value('locSrv.ext_pos_std', '{:f}'.format(0.3))
			if self.USING_KALMAN: self.crazyflie.param.set_value('kalman.quadWeight', '{:f}'.format(27.0))
			self.crazyflie.param.set_value('timeout.timeoutStab', '{:d}'.format(1000*1))  # Stabilize (rpy=0) CF if doesn't receive a radio command in 10min
			self.crazyflie.param.set_value('timeout.timeoutShut', '{:d}'.format(1000*3))  # Shutdown CF if doesn't receive a radio command in 20min
			# self.crazyflie.param.set_value('deadReckoning.updateDRpos', '{:d}'.format(self.cf_update_DR))
			# self.crazyflie.param.set_value('deadReckoning.updateDRvel', '{:d}'.format(self.cf_update_DR))
			self.crazyflie.param.set_value('posCtlPid.thrustBase', '{}'.format(self.TAKEOFF_THRUST))

			# self.crazyflie.param.set_value('posCtlPid.xKp', '{}'.format(0.3))
			self.crazyflie.param.set_value('posCtlPid.xKp', '{}'.format(0.7))
			self.crazyflie.param.set_value('velCtlPid.vxKp', '{}'.format(5))
			self.crazyflie.param.set_value('velCtlPid.vxKi', '{}'.format(0.2))
			self.crazyflie.param.set_value('velCtlPid.vxKd', '{}'.format(0.3))

			# self.crazyflie.param.set_value('posCtlPid.yKp', '{}'.format(0.3))
			self.crazyflie.param.set_value('posCtlPid.yKp', '{}'.format(0.7))
			self.crazyflie.param.set_value('velCtlPid.vyKp', '{}'.format(5))
			self.crazyflie.param.set_value('velCtlPid.vyKi', '{}'.format(0.2))
			self.crazyflie.param.set_value('velCtlPid.vyKd', '{}'.format(0.3))

			# self.crazyflie.param.set_value('posCtlPid.zKp', '{}'.format(0.4))
			self.crazyflie.param.set_value('posCtlPid.zKp', '{}'.format(0.7))
			# self.crazyflie.param.set_value('velCtlPid.vzKp', '{}'.format(8000))
			# self.crazyflie.param.set_value('velCtlPid.vzKi', '{}'.format(3000))
			self.crazyflie.param.set_value('velCtlPid.vzKp', '{}'.format(5))
			self.crazyflie.param.set_value('velCtlPid.vzKi', '{}'.format(0.2))
			self.crazyflie.param.set_value('velCtlPid.vzKd', '{}'.format(0.3))

			self.crazyflie.param.set_value("ring.effect", "1")  # Turn off LED ring
			self.crazyflie.param.set_value("ring.headlightEnable", "0")  # Turn off LED headlight

			self.crazyflie.param.add_update_callback(group="posCtlPid", name=None, cb=self.on_cf_param_new_data)  # Receive values of params for debugging
			self.crazyflie.param.add_update_callback(group="velCtlPid", name=None, cb=self.on_cf_param_new_data)  # Receive values of params for debugging
			for g in ["posCtlPid", "velCtlPid"]:
				for p in self.crazyflie.param.toc.toc[g].itervalues():  # Request an update of every param in the group
					self.crazyflie.param.request_param_update("{}.{}".format(p.group, p.name))
			print("Waiting to receive all posCtlPid PID parmams on the CF")
			while self.log_PID_x.PIDpos.Kp==0 or self.log_PID_x.PIDvel.Kp==0 or self.log_PID_y.PIDpos.Kp==0 or self.log_PID_y.PIDvel.Kp==0 or self.log_PID_z.PIDpos.Kp==0 or self.log_PID_z.PIDvel.Kp==0:  # Wait until we receive an update on all PID params on the CF
				time.sleep(1)

		except Exception as e:
			raise Exception("Couldn't initialize CrazyFlie params to their desired values. Details: {}".format(e.message))

		# Create a log configuration and include all variables that want to be logged
		self.cf_log_attitude = LogConfig(name="cf_log_attitude", period_in_ms=10)
		self.cf_log_attitude.add_variable("stabilizer.roll", "float")
		self.cf_log_attitude.add_variable("stabilizer.pitch", "float")
		self.cf_log_attitude.add_variable("stabilizer.yaw", "float")
		self.cf_log_PID_x = LogConfig(name="cf_log_PID_x", period_in_ms=10)
		self.cf_log_PID_x.add_variable("stateWorld.px", "float")
		self.cf_log_PID_x.add_variable("posCtl.targetX", "float")
		self.cf_log_PID_x.add_variable("posCtl.Xp", "float")
		self.cf_log_PID_x.add_variable("posCtl.VXp", "float")
		# self.cf_log_PID_x.add_variable("posCtl.VXi", "float")
		self.cf_log_PID_x.add_variable("posCtl.VXd", "float")
		self.cf_log_PID_y = LogConfig(name="cf_log_PID_y", period_in_ms=10)
		self.cf_log_PID_y.add_variable("stateWorld.py", "float")
		self.cf_log_PID_y.add_variable("posCtl.targetY", "float")
		self.cf_log_PID_y.add_variable("posCtl.Yp", "float")
		self.cf_log_PID_y.add_variable("posCtl.VYp", "float")
		# self.cf_log_PID_y.add_variable("posCtl.VYi", "float")
		self.cf_log_PID_y.add_variable("posCtl.VYd", "float")
		self.cf_log_PID_z = LogConfig(name="cf_log_PID_z", period_in_ms=10)
		self.cf_log_PID_z.add_variable("stateWorld.pz", "float")
		self.cf_log_PID_z.add_variable("posCtl.targetZ", "float")
		self.cf_log_PID_z.add_variable("posCtl.Zp", "float")
		self.cf_log_PID_z.add_variable("posCtl.VZp", "float")
		# self.cf_log_PID_z.add_variable("posCtl.VZi", "float")
		self.cf_log_PID_z.add_variable("posCtl.VZd", "float")
		self.cf_logs = [self.cf_log_attitude, self.cf_log_PID_x, self.cf_log_PID_y, self.cf_log_PID_z]

		try:
			self.crazyflie.log.add_config(self.cf_log_attitude)  # Validate the log configuration and attach it to our CF
			self.crazyflie.log.add_config(self.cf_log_PID_x)
			self.crazyflie.log.add_config(self.cf_log_PID_y)
			self.crazyflie.log.add_config(self.cf_log_PID_z)
		except Exception as e:
			raise Exception("Couldn't attach the log config to the CrazyFlie, bad configuration. Details: {}".format(e.message))
		self.cf_log_attitude.data_received_cb.add_callback(self.on_cf_log_new_data)  # Register appropriate callbacks
		self.cf_log_attitude.error_cb.add_callback(self.on_cf_log_error)
		self.cf_log_PID_x.data_received_cb.add_callback(self.on_cf_log_new_data)
		self.cf_log_PID_x.error_cb.add_callback(self.on_cf_log_error)
		self.cf_log_PID_y.data_received_cb.add_callback(self.on_cf_log_new_data)
		self.cf_log_PID_y.error_cb.add_callback(self.on_cf_log_error)
		self.cf_log_PID_z.data_received_cb.add_callback(self.on_cf_log_new_data)
		self.cf_log_PID_z.error_cb.add_callback(self.on_cf_log_error)
		self.cf_log_attitude.start()

		# Automatically detect the first yaw log packet and set the current orientation as the desired yaw
		while abs(self.cf_yaw) < 0.01:  # Wait until first cf_yaw value is received (cf_yaw=0 by default)
			time.sleep(0.1)
		self.cf_target_yaw = self.cf_yaw
		print("Target yaw set at {:.2f}.".format(self.cf_yaw))
		# self.crazyflie.param.set_value('baro.aslOffset', '{}'.format(self.cf_estimated_z+0.2))

		self.crazyflie.add_port_callback(CRTPPort.CONSOLE, self.print_cf_console)

	def control_cf(self, new_pos, t_frame):
		"""
		Sends position and/or velocity updates to the worker drone.
		:param new_pos: 1x3 np.array indicating the current Spotter's position estimation of the Worker
		:param t_frame: Datetime at which new_pos was estimated
		"""
		self.cf_pos_tracked = (new_pos is not None and datetime.now() > self.ignore_cam_until)
		self.experiment_log.update(experiment_running=self.experiment_running, pos_tracked=(datetime.now() > self.ignore_cam_until), update_DR=self.cf_update_DR)

		if self.cf_t_last_target_pos_sent is None or datetime.now() > self.cf_t_last_target_pos_sent+self.CF_TARGET_POS_SEND_PERIOD:
			self.cf_t_last_target_pos_sent = datetime.now()
			if self.cf_taking_off:
				# self.crazyflie.commander.send_setpoint(0, 0, 0, WorkerDrone.TAKEOFF_THRUST)
				self.crazyflie.commander.send_velocity_world_setpoint(0, 0, 0.25, 0)  # Take off at 25cm/s
			else:
				self.crazyflie.commander.send_setpoint(self.cf_target_pos[1], self.cf_target_pos[0], 0, 1000*(self.cf_target_pos[2] + self.POS_Z_OFFSET))

		if not self.cf_pos_tracked:  # If cv algorithm wasn't able to detect the drone, linearly estimate its position based on previous position and speed
			pass
			# self.cf_curr_pos += self.cf_curr_vel*dt
		else:
			self.cnt_iteration += 1
			self.cf_past_pos.append(new_pos)  # Update history of position and sampling times
			self.cf_past_pos_t.append(t_frame)
			dt = (t_frame - self.cf_curr_pos_t).total_seconds()
			if dt < self.MAX_INTERVAL_FOR_VEL_ESTIMATION:
				# self.cf_curr_vel = self.VEL_ALPHA * self.cf_curr_vel + (1 - self.VEL_ALPHA) * (new_pos - self.cf_curr_pos) / dt
				self.cf_curr_vel = plot_tools.DerivativeHelper.differentiate(self.cf_past_pos, self.cf_past_pos_t, win_size=self.VEL_DERIV_WIN_SIZE, poly_order=self.VEL_DERIV_POLY_ORDER, deriv=1)
				# self.send_cf_dr_update(True, self.cf_curr_vel[0], self.cf_curr_vel[1], self.cf_curr_vel[2], self.cnt_iteration)
				str_debug_vel = "vx={v[0]:.2f}m/s, vy={v[1]:.2f}m/s, vz={v[2]:.2f}m/s".format(v=self.cf_curr_vel)
			else:  # Haven't tracked the worker for some time, so I shouldn't use past information to estimate velocity in the (near) future
				self.cf_past_pos = [new_pos]  # "Forget" about past history, last point was too long ago
				self.cf_past_pos_t = [t_frame]
				str_debug_vel = "can't estimate (dt={:.2f}s)".format(dt)
			self.cf_curr_pos = new_pos  # Update the new current position
			self.cf_curr_pos_t = t_frame  # Update the time at which curr_pos was estimated
			if self.cf_taking_off:  # Avoid problems when taking off and no target pos has been set yet -> Assign to curr pos
				self.cf_target_pos = self.cf_curr_pos
			# self.send_cf_dr_update(False, self.cf_curr_pos[0], self.cf_curr_pos[1], self.cf_curr_pos[2]+self.POS_Z_OFFSET, self.cnt_iteration)
			self.crazyflie.extpos.send_extpos(self.cf_curr_pos[0], self.cf_curr_pos[1], self.cf_curr_pos[2]+self.POS_Z_OFFSET)
			print("@{t}, sending position: px={p[0]:.2f}m, py={p[1]:.2f}m, pz={p[2]:.2f}m; velocity: {v}; n={n}".format(t=t_frame, p=self.cf_curr_pos, v=str_debug_vel, n=self.cnt_iteration))

	def cleanup_cf(self, save_logs=True):
		"""
		"Cleans up" the worker: stops logging, saves/plots logs if necessary, disconnects drone, etc.
		:param save_logs: Whether or not to plot and save all logs
		"""
		# If we get here, either the user ended the experiment, or an exception occurred. Same action regardless:
		try:
			self.send_cf_param('flightmode.posSet', '{:d}'.format(False))
			self.send_cf_param('flightmode.althold', '{:d}'.format(False))  # Make sure we're not on althold mode, so sending a thrust 0 will kill the motors and not just descend
			self.crazyflie.commander.send_setpoint(0, 0, 0, 0)  # Kill the motors (if the experiment went well, they should already be killed, but this is important if there was an Exception)
			self.crazyflie.commander.send_stop_setpoint()

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
		if logconf.name == "cf_log_attitude":
			self.cf_roll = data['stabilizer.roll']
			self.cf_pitch = data['stabilizer.pitch']
			self.cf_yaw = data['stabilizer.yaw']
			# self.cf_estimated_z = data['posEstimatorAlt.estimatedZ']
			# self.cf_vel_z = data['posEstimatorAlt.velocityZ']
			print "\rCurrent yaw: {:.2f}deg".format(self.cf_yaw),
			self.experiment_log.update(yaw=self.cf_yaw)
		elif logconf.name == "cf_log_PID_x":
			self.log_PID_x.PIDpos.setSetPoint(data["posCtl.targetX"])
			# self.log_PID_x.PIDpos.update(self.log_PID_x.PIDpos.SetPoint - (data["posCtl.Xp"]/self.log_PID_x.PIDpos.Kp))
			self.log_PID_x.PIDpos.update(data["stateWorld.px"])
			self.log_PID_x.PIDvel.setSetPoint(data["posCtl.Xp"])  # Since PIDpos only has P, output=pxP -> =PIDvel.SetPoint
			self.log_PID_x.PIDvel.update(self.log_PID_x.PIDvel.SetPoint - (data["posCtl.VXp"]/self.log_PID_x.PIDvel.Kp))
			self.log_PID_x.PIDpos.PTerm = data["posCtl.Xp"]
			self.log_PID_x.PIDpos.ITerm = self.log_PID_x.DTerm = 0
			self.log_PID_x.PIDvel.PTerm = data["posCtl.VXp"]
			# self.log_PID_x.PIDvel.ITerm = 0 if self.log_PID_x.PIDvel.Ki==0 else data["posCtl.VXi"]/self.log_PID_x.PIDvel.Ki  # ITerm and DTerm don't include their K
			self.log_PID_x.PIDvel.DTerm = 0 if self.log_PID_x.PIDvel.Kd==0 else data["posCtl.VXd"]/self.log_PID_x.PIDvel.Kd
			self.experiment_log.update(px=self.log_PID_x)
		elif logconf.name == "cf_log_PID_y":
			self.log_PID_y.PIDpos.setSetPoint(data["posCtl.targetY"])
			# self.log_PID_y.PIDpos.update(self.log_PID_y.PIDpos.SetPoint - (data["posCtl.Yp"]/self.log_PID_y.PIDpos.Kp))
			self.log_PID_y.PIDpos.update(data["stateWorld.py"])
			self.log_PID_y.PIDvel.setSetPoint(data["posCtl.Yp"])
			self.log_PID_y.PIDvel.update(self.log_PID_y.PIDvel.SetPoint - (data["posCtl.VYp"]/self.log_PID_y.PIDvel.Kp))
			self.log_PID_y.PIDpos.PTerm = data["posCtl.Yp"]
			self.log_PID_y.PIDpos.ITerm = self.log_PID_y.DTerm = 0
			self.log_PID_y.PIDvel.PTerm = data["posCtl.VYp"]
			# self.log_PID_y.PIDvel.ITerm = 0 if self.log_PID_y.PIDvel.Ki==0 else data["posCtl.VYi"]/self.log_PID_y.PIDvel.Ki
			self.log_PID_y.PIDvel.DTerm = 0 if self.log_PID_y.PIDvel.Kd==0 else data["posCtl.VYd"]/self.log_PID_y.PIDvel.Kd
			self.experiment_log.update(py=self.log_PID_y)
		elif logconf.name == "cf_log_PID_z":
			self.log_PID_z.PIDpos.setSetPoint(data["posCtl.targetZ"] - self.POS_Z_OFFSET)
			# self.log_PID_z.PIDpos.update(self.log_PID_z.PIDpos.SetPoint - (data["posCtl.Zp"]/self.log_PID_z.PIDpos.Kp))
			self.log_PID_z.PIDpos.update(data["stateWorld.pz"] - self.POS_Z_OFFSET)
			self.log_PID_z.PIDvel.setSetPoint(data["posCtl.Zp"])
			self.log_PID_z.PIDvel.update(self.log_PID_z.PIDvel.SetPoint - (data["posCtl.VZp"]/self.log_PID_z.PIDvel.Kp))
			self.log_PID_z.PIDpos.PTerm = data["posCtl.Zp"]
			self.log_PID_z.PIDpos.ITerm = self.log_PID_z.DTerm = 0
			self.log_PID_z.PIDvel.PTerm = data["posCtl.VZp"]
			# self.log_PID_z.PIDvel.ITerm = 0 if self.log_PID_z.PIDvel.Ki==0 else data["posCtl.VZi"]/self.log_PID_z.PIDvel.Ki
			self.log_PID_z.PIDvel.DTerm = 0 if self.log_PID_z.PIDvel.Kd==0 else data["posCtl.VZd"]/self.log_PID_z.PIDvel.Kd
			self.experiment_log.update(pz=self.log_PID_z)

	def on_cf_param_new_data(self, name, value):
		logging.debug("\t\tParam callback: {}={}".format(name, value))
		name = name.split(".")[1]
		value = float(value)
		log = self.log_PID_x if name[-3]=="x" else self.log_PID_y if name[-3]=="y" else self.log_PID_z if name[-3]=="z" else None
		if log is not None:
			log = log.PIDvel if name[0]=="v" else log.PIDpos
			if log is not None:
				if name[-1] == "p":
					log.setKp(value)
				elif name[-1] == "i":
					log.setKi(value)
				elif name[-1] == "d":
					log.setKd(value)
				logging.info("Received param! {} = {}".format(name, value))

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

	# def send_cf_dr_update(self, is_vel, x, y, z, timestamp):
	# 	"""
	# 	Sends a custom packet that will be interpreted by our modified CF firmware to update its position/velocity estimation.
	# 	:param is_vel: True if data to send is velocity data; False for position updates
	# 	:param x: x coordinate of the position/velocity vector to send
	# 	:param y: y coordinate of the position/velocity vector to send
	# 	:param z: z coordinate of the position/velocity vector to send
	# 	:param timestamp: Time at which position/velocity vector was computed/estimated
	# 	"""
	# 	pk = CRTPPacket()
	# 	pk.port = self.CRTP_PORT_DR
	# 	pk.channel = is_vel
	# 	pk.data = struct.pack('<fffL', x, y, z, timestamp)
	# 	self.crazyflie.send_packet(pk)

	def reset_Kalman(self):
		if self.USING_KALMAN:
			self.cf_kalman_ready = False
			self.crazyflie.param.set_value('kalman.resetEstimation', '1')
			self.start_waiting_for_Kalman_to_stabilize()

	def start_waiting_for_Kalman_to_stabilize(self):
		if self.USING_KALMAN:
			var_x_history = [1000] * 10; var_y_history = [1000] * 10; var_z_history = [1000] * 10
			var_threshold = 0.001

			def on_kalman_log_new_data(timestamp, data, logconf):
				if logconf.name == "KalmanVariance":
					var_x_history.pop(0); var_x_history.append(data['kalman.varPX'])
					var_y_history.pop(0); var_y_history.append(data['kalman.varPY'])
					var_z_history.pop(0); var_z_history.append(data['kalman.varPZ'])
					print "Kalman filter variance: x={:8.4f}, y={:8.4f}, z={:8.4f}; Pos variance: x={:8.4f}, y={:8.4f}, z={:8.4f}".format(data['kalman.varPX'], data['kalman.varPY'], data['kalman.varPZ'], data['kalman.varX'], data['kalman.varY'], data['kalman.varZ'])
					# print "Kalman filter variance: x={:8.4f}, y={:8.4f}, z={:8.4f}".format(data['kalman.varPX'], data['kalman.varPY'], data['kalman.varPZ'])

					if (max(var_x_history)-min(var_x_history)) < var_threshold and (max(var_y_history)-min(var_y_history)) < var_threshold and (max(var_z_history)-min(var_z_history)) < var_threshold:
						log_config.stop()
						self.cf_kalman_ready = True

			log_config = LogConfig(name='KalmanVariance', period_in_ms=250)
			log_config.add_variable('kalman.varPX', 'float')
			log_config.add_variable('kalman.varPY', 'float')
			log_config.add_variable('kalman.varPZ', 'float')
			log_config.add_variable('kalman.varX', 'float')
			log_config.add_variable('kalman.varY', 'float')
			log_config.add_variable('kalman.varZ', 'float')
			try:
				self.crazyflie.log.add_config(log_config)
			except Exception as e:
				raise Exception("Couldn't attach the Kalman filter log config to the CrazyFlie, bad configuration. Details: {}".format(e.message))
			log_config.data_received_cb.add_callback(on_kalman_log_new_data)  # Register appropriate callbacks
			log_config.error_cb.add_callback(self.on_cf_log_error)
			log_config.start()

	def wait_for_Kalman_to_stabilize(self):
		while self.USING_KALMAN and not self.cf_kalman_ready:
			time.sleep(0.25)

	def get_OSD_text(self, t_frame, t_last_frame, t_start):
		"""
		Generates written debug information (OSD=on-screen display) to be displayed at the bottom of the current frame
		:param t_frame: Datetime at which camera frame was obtained (through datetime.now())
		:param t_last_frame: Datetime at which last camera frame was obtained
		:param t_start: Datetime at which the experiment/flight started (so we can display elapsed time)
		:return: String containing relevant debug information (worker estimated position, velocity, roll-pitch-yaw, etc.)
		"""
		return "Pos: px={p[0]:5.2f}m, py={p[1]:5.2f}m, pz={p[2]:5.2f}m\t\tTarget: sx={s[0]:5.2f}m, sy={s[1]:5.2f}m, sz={s[2]:5.2f}m\t\tVel: vx={v[0]:5.2f}m/s, vy={v[1]:5.2f}m/s, vz={v[2]:5.2f}m/s\t\tAttitude: r={:+6.2f} p={:+6.2f} y={:+6.2f}\t\t@{} n={:3} (FPS: {:5.2f}) - {}".format(
				self.cf_roll, self.cf_pitch, self.cf_yaw, str(t_frame-t_start)[3:-3], self.cnt_iteration, 1./(t_frame-t_last_frame).total_seconds(), self.cf_str_status, p=self.cf_curr_pos, s=self.cf_target_pos, v=self.cf_curr_vel)


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
	SETTINGS_ENVIRONMENT = "AiFi"
	CAMERA_SETTINGS_FILE = "config/cam_settings/Camera settings - USB 2.0 Camera - {}.txt".format(SETTINGS_ENVIRONMENT)
	COLOR_THRESH_SETTINGS_FILE = "config/color_thresh/Color threshold settings - {}.txt".format(SETTINGS_ENVIRONMENT)
	BLOB_DETECTOR_SETTINGS_FILE = "config/blob_detector/Blob detector settings.txt"
	SETTINGS_SEPARATOR = UvcCapture.SETTINGS_SEPARATOR  # We save files in a csv type of way
	CAM_FOCAL_LENGTH_IN_PX = 1250.0
	CF_RADIUS_IN_M = 0.02
	KB_TARGET_POS_INC = 0.2  # Move CF's target position by 0.2m on each kb input command

	def __init__(self, bool_ground_truth=False, bool_world_coords_pattern=False):
		self.t_start = self.t_frame = self.t_last_frame = datetime.now()
		self.t_events = []
		self.EXPERIMENT_START_DATETIME = str(self.t_start)[:-7].replace(':', '-')
		self.VIDEO_FOLDER = "img-ns/{}".format(self.EXPERIMENT_START_DATETIME)
		self.ground_truth_thread = GroundTruthThread("{}/GT".format(self.VIDEO_FOLDER), self.EXPERIMENT_START_DATETIME) if bool_ground_truth else None
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
			if isinstance(connect_to, int):  # If connect_to is a number, choose first available link
				link_uri = available_links[0][0]

			logging.info("Initializing CrazyFlie (connecting to '{}').".format(link_uri))
			self.workers.append(WorkerDrone(link_uri, self.EXPERIMENT_START_DATETIME))

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
		print "Giving you 2s to prepare for take-off... (waiting for Kalman to stabilize meanwhile...)"
		for w in self.workers:  # Force a reset of all workers' Kalman filters
			w.reset_Kalman()
		while any([not w.cf_kalman_ready for w in self.workers]):
			try:  # First, run the cv algorithm to estimate the CF's position
				new_pos_arr = self.detect_cf_in_camera()
			except:  # Only way detect_cf_in_camera raises an Exception is if a camera frame couldn't be grabbed
				logging.exception("Couldn't grab a frame from the camera. Exiting")
				return False  # Exit without saving logs

			# Now that we know where the drone currently is, send position updates so Kalman filter variance converges
			for i, w in enumerate(self.workers):
				w.cf_curr_pos = self.img_to_cf_world_coords(new_pos_arr[i])
				w.crazyflie.extpos.send_extpos(w.cf_curr_pos[0], w.cf_curr_pos[1], w.cf_curr_pos[2] + w.POS_Z_OFFSET)

		time.sleep(2)
		for w in self.workers:  # Make sure all Kalman filters have converged to a position estimation, otherwise for them
			w.wait_for_Kalman_to_stabilize()

		# t = Timer(10, self.cleanup_experiment)  # Start a timer to automatically disconnect in 10s
		# t.start()

		# Prepare for take off: start logging PIDs, save start time...
		self.t_start = datetime.now()
		for w in self.workers:
			w.experiment_log.update(experiment_running=False)
			w.cf_log_PID_x.start()  # Start logging PID data
			w.cf_log_PID_y.start()
			w.cf_log_PID_z.start()
			w.cf_str_status = "TAKING OFF"
			w.crazyflie.commander.send_setpoint(0, 0, 0, 0)  # New firmware version requires to send thrust=0 at least once to "unlock thrust"
			# w.crazyflie.commander.send_setpoint(0, 0, 0, WorkerDrone.TAKEOFF_THRUST)  #w.cf_target_yaw
			w.crazyflie.commander.send_velocity_world_setpoint(0, 0, 0.25, 0)  # Take off at 25cm/s
			w.cf_t_last_target_pos_sent = datetime.now()

			# w.crazyflie.param.set_value('flightmode.posSet', '{:d}'.format(True))
			# w.crazyflie.extpos.send_extpos(w.cf_curr_pos[0], w.cf_curr_pos[1], w.cf_curr_pos[2] + w.POS_Z_OFFSET)
			# w.crazyflie.commander.send_setpoint(w.cf_curr_pos[1], w.cf_curr_pos[0], 0, 1000*(w.cf_curr_pos[2] + w.POS_Z_OFFSET + 0.5))

		# Alright, let's fly!
		tStop = None
		while tStop is None:  # tStop will remain None while everything's fine
			tStop = self.hover()  # hover returns None while everything's fine; the time to end the experiment otherwise

		# If we get here, either the user stopped the experiment or the code detected something went wrong
		print("AT t={}, A KEY WAS PRESSED -> STOPPING!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3]))
		save_logs = np.any([w.cf_str_status != "TAKING OFF" for w in self.workers])  # Only store the logs if a drone ever started flying (not just took off)
		for w in self.workers:
			w.experiment_log.update(experiment_running=False)
			w.cf_str_status = "STOPPED"  # Updated new drone status

		while datetime.now() < tStop:  # Keep calling hover until tStop, so data is still logged
			self.hover()

		return save_logs  # Return whether or not to keep the logs

	def cleanup_experiment(self, save_logs=True):
		"""
		"Cleans up" the experiment: closes any open windows, saves logs, disconnects camera and drone, etc.
		:param save_logs: Whether or not to plot and save all logs
		"""
		if self.ground_truth_thread is not None and self.ground_truth_thread.isAlive():
			self.ground_truth_thread.stop()

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

		save_logs = False  # Use this auxiliary variable to prevent saving logs if the drone never took off
		try:
			self.connect_to_cf(connect_to)  # Connect to the CrazyFlie
			for w in self.workers:
				w.setup_cf()  # Can't initialize LogConfig until we're connected, because it needs to check that the variables we'd like to add are in the TOC. So this function must be called after connect_to_cf()
			self.init_video_cam_and_cv_algorithm()  # Connect to the first available camera, load default settings, etc.
			self.init_UI_window()  # Open a window to receive user input to control the CF
			if self.ground_truth_thread is not None:
				pre_experiment_find_extrinsics(self.ground_truth_thread, self.bool_world_coords_pattern)
				self.ground_truth_thread.start()
			for w in self.workers:
				log_folder = "log/{}/{}".format(w.cf_radio_ch, self.EXPERIMENT_START_DATETIME)
				os.makedirs(log_folder)
				if self.ground_truth_thread is not None:
					np.savez_compressed("{}/log_experiment_constants.npz".format(log_folder),
						gt_camera_matrix=self.ground_truth_thread.video_capture.camera_matrix, gt_dist_coefs=self.ground_truth_thread.video_capture.dist_coefs, gt_frame_rate=self.ground_truth_thread.video_capture.frame_rate, gt_frame_size=self.ground_truth_thread.video_capture.frame_size,
						gt_world_to_camera_transf=self.ground_truth_thread.world_to_camera_transf, gt_F=self.ground_truth_thread.CAM_FOCAL_LENGTH_IN_PX,
						spotter_camera_matrix=self.video_capture.camera_matrix, spotter_dist_coefs=self.video_capture.dist_coefs, spotter_frame_rate=self.video_capture.frame_rate, spotter_frame_size=self.video_capture.frame_size,
						spotter_world_to_camera_transf=self.world_to_camera_transf, spotter_F=self.CAM_FOCAL_LENGTH_IN_PX, spotter_HSV_thresh_min=self.cv_HSV_thresh_min, spotter_HSV_thresh_max=self.cv_HSV_thresh_max,
						worker_max_interval_for_vel_estimation=w.MAX_INTERVAL_FOR_VEL_ESTIMATION, worker_vel_alpha=w.VEL_ALPHA,
						worker_kPID_pX=[w.log_PID_x.PIDpos.Kp, w.log_PID_x.PIDpos.Ki, w.log_PID_x.PIDpos.Kd], worker_kPID_vX=[w.log_PID_x.PIDvel.Kp, w.log_PID_x.PIDvel.Ki, w.log_PID_x.PIDvel.Kd],
						worker_kPID_pY=[w.log_PID_y.PIDpos.Kp, w.log_PID_y.PIDpos.Ki, w.log_PID_y.PIDpos.Kd], worker_kPID_vY=[w.log_PID_y.PIDvel.Kp, w.log_PID_y.PIDvel.Ki, w.log_PID_y.PIDvel.Kd],
						worker_kPID_pZ=[w.log_PID_z.PIDpos.Kp, w.log_PID_z.PIDpos.Ki, w.log_PID_z.PIDpos.Kd], worker_kPID_vZ=[w.log_PID_z.PIDvel.Kp, w.log_PID_z.PIDvel.Ki, w.log_PID_z.PIDvel.Kd])
				else:
					np.savez_compressed("{}/log_experiment_constants.npz".format(log_folder),
						spotter_camera_matrix=self.video_capture.camera_matrix, spotter_dist_coefs=self.video_capture.dist_coefs, spotter_frame_rate=self.video_capture.frame_rate, spotter_frame_size=self.video_capture.frame_size,
						spotter_world_to_camera_transf=self.world_to_camera_transf, spotter_F=self.CAM_FOCAL_LENGTH_IN_PX, spotter_HSV_thresh_min=self.cv_HSV_thresh_min, spotter_HSV_thresh_max=self.cv_HSV_thresh_max,
						worker_max_interval_for_vel_estimation=w.MAX_INTERVAL_FOR_VEL_ESTIMATION, worker_vel_alpha=w.VEL_ALPHA,
						worker_kPID_pX=[w.log_PID_x.PIDpos.Kp, w.log_PID_x.PIDpos.Ki, w.log_PID_x.PIDpos.Kd], worker_kPID_vX=[w.log_PID_x.PIDvel.Kp, w.log_PID_x.PIDvel.Ki, w.log_PID_x.PIDvel.Kd],
						worker_kPID_pY=[w.log_PID_y.PIDpos.Kp, w.log_PID_y.PIDpos.Ki, w.log_PID_y.PIDpos.Kd], worker_kPID_vY=[w.log_PID_y.PIDvel.Kp, w.log_PID_y.PIDvel.Ki, w.log_PID_y.PIDvel.Kd],
						worker_kPID_pZ=[w.log_PID_z.PIDpos.Kp, w.log_PID_z.PIDpos.Ki, w.log_PID_z.PIDpos.Kd], worker_kPID_vZ=[w.log_PID_z.PIDvel.Kp, w.log_PID_z.PIDvel.Ki, w.log_PID_z.PIDvel.Kd])
			save_logs = True  # Now that everything is set up, set save_logs to True so in case fly_cf crashes, data is recorded
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
		return np.array([1, -1, -1]) * world_coords  # Flip y and z axes sign to convert world->cf coords. USE THIS FOR HORIZONTAL CHESSBOARD WITH +X AXIS POINTING TOWARDS CAMERA (+Y AXIS LEFT)
		# return np.array([-1, 1, 1]) * world_coords[[2,1,0]] + [0,0,0.8]  # This means: CF_x=-world_z, CF_y=world_y, CF_z=world_x. USE THIS FOR VERTICAL CHESSBOARD WITH +X AXIS UP (+Y AXIS RIGHT)

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
		world_coords = np.array([1, -1, -1]) * cf_world_coords  # Flip y and z axes sign to convert cf->world coords. USE THIS FOR HORIZONTAL CHESSBOARD WITH +X AXIS POINTING TOWARDS CAMERA (+Y AXIS LEFT)
		# world_coords = np.array([1, 1, -1]) * (cf_world_coords[[2, 1, 0]] - [0.8,0,0])  # This means: world_x=CF_z, world_y=CF_y, world_z=-CF_x. USE THIS FOR VERTICAL CHESSBOARD WITH +X AXIS UP (+Y AXIS RIGHT)

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
				send_posSet = False
				target_pos_offset = np.array([0, 0, 0])

				if key == 'a':    # Move left
					target_pos_offset[1] = +1
				elif key == 's':  # Move back
					target_pos_offset[0] = -1
				elif key == 'd':  # Move right
					target_pos_offset[1] = -1
				elif key == 'w':  # Move forward
					target_pos_offset[0] = +1
				elif key == 'u':  # Move up
					target_pos_offset[2] = +1
				elif key == 'h':  # Move down
					target_pos_offset[2] = -1
				elif key == 'e':  # Stop taking off and start flying (hover at current position)
					for w in self.workers:
						w.cf_taking_off = False
						w.cf_str_status = "FLYING"
						target_pos_offset = np.array([0, 0, 0.25])  # Make them hover slightly above current position
						send_posSet = True
				elif key == 'x':
					for w in self.workers:
						w.experiment_running = not w.experiment_running
				elif key == 'z':
					for w in self.workers:
						w.ignore_cam_until = datetime.now() + timedelta(seconds=0.5)
						if not w.cf_update_DR:  # If not updating DR, then just stay level
							w.cnt_iteration += 1
							# w.send_cf_dr_update(True, 0, 0, 0, w.cnt_iteration)
							# w.send_cf_dr_update(False, w.cf_target_pos[0], w.cf_target_pos[1], w.cf_target_pos[2]+w.POS_Z_OFFSET, w.cnt_iteration)
							w.crazyflie.extpos.send_extpos(w.cf_target_pos[0], w.cf_target_pos[1], w.cf_target_pos[2]+w.POS_Z_OFFSET)
				elif key == 'f':
					for w in self.workers:
						w.cf_update_DR = not w.cf_update_DR
						# w.send_cf_param('deadReckoning.updateDRpos', '{:d}'.format(w.cf_update_DR))
						# w.send_cf_param('deadReckoning.updateDRvel', '{:d}'.format(w.cf_update_DR))
				else:  # Any other key ends the experiment
					for w in self.workers:
						w.send_cf_param('flightmode.posSet', '{:d}'.format(False))
						w.crazyflie.commander.send_setpoint(0, 0, 0, 0)
						w.crazyflie.commander.send_stop_setpoint()
						w.cf_radio_connected = False
						w.experiment_running = False
					return False

				# If user pressed any valid control key, there's a new setpoint -> Send it
				for w in (self.workers if self.kb_controls_which_cf < 0 else [self.workers[self.kb_controls_which_cf]]):
					if not w.cf_radio_connected:
						continue  # Make sure we don't send new commands after the user has request to terminate the experiment
					w.cf_str_status = "{} - {}DR".format(w.cf_str_status.split(" - ")[0], "No " if not w.cf_update_DR else "")

					w.cf_target_pos += self.KB_TARGET_POS_INC*target_pos_offset
					w.cf_target_pos[2] = max(w.cf_target_pos[2], -w.POS_Z_OFFSET)  # Can't send a negative z value, make sure w.cf_target_pos[2] >= w.POS_Z_OFFSET
					print("CF rc{} holding position at: x={p[0]:.2f}m, y={p[1]:.2f}m, z={p[2]:.2f}m".format(w.cf_radio_ch, p=w.cf_target_pos))
					w.crazyflie.commander.send_setpoint(w.cf_target_pos[1], w.cf_target_pos[0], 0, 1000*(w.cf_target_pos[2]+w.POS_Z_OFFSET))
					w.cf_t_last_target_pos_sent = datetime.now()
					if send_posSet:
						w.send_cf_param('flightmode.posSet', '{:d}'.format(True))
						w.crazyflie.commander.send_setpoint(w.cf_target_pos[1], w.cf_target_pos[0], 0, 1000*(w.cf_target_pos[2]+w.POS_Z_OFFSET))  # Just in case, send the new position again
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
					output.append(self.cf_world_to_img_coords(kp_pos3d[np.argmin(np.sum((w.cf_curr_pos - kp_pos3d) ** 2, axis=1)), :]))
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
			curr_pos_resized = self.cf_world_to_img_coords(w.cf_curr_pos) * img_resize_factor
			target_pos_resized = self.cf_world_to_img_coords(w.cf_target_pos) * img_resize_factor
			if w.cf_pos_tracked:
				cv2.circle(frame_resized, tuple(curr_pos_resized[0:2].astype(int)), int(curr_pos_resized[2]), self.COLOR_BALL_TRACKED if w.cf_pos_tracked else self.COLOR_BALL_UNTRACKED, -1)
				cv2.line(frame_resized, tuple(curr_pos_resized[0:2].astype(int)), tuple(target_pos_resized[0:2].astype(int)), self.COLOR_LINE_TRACKED if w.cf_pos_tracked else self.COLOR_LINE_UNTRACKED, int(10*img_resize_factor))
			cv2.circle(frame_resized, tuple(target_pos_resized[0:2].astype(int)), int(target_pos_resized[2]), self.COLOR_TARGET_TRACKED if w.cf_pos_tracked else self.COLOR_TARGET_UNTRACKED, -1)
		self.t_events.append(datetime.now())

		# On top of that, overlay the HSV mask (so we can debug color filtering + blob detection steps)
		np.putmask(frame_resized, cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR).astype(bool), list(mask_color))
		for i,w in enumerate(self.workers):
			curr_pos_resized = self.cf_world_to_img_coords(w.cf_curr_pos) * img_resize_factor
			str_cf_id = "{}".format(i+1)  # w.cf_radio_ch
			txt_size = np.array(cv2.getTextSize(str_cf_id, font, font_scale, font_thickness)[0])
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
		cv2.imwrite(os.path.join(self.VIDEO_FOLDER, self.t_frame.strftime("out_%H-%M-%S-%f.jpg")), self.cv_frame_out)
		self.t_events.append(datetime.now())

	def hover(self):
		"""
		Executes the workflow of a single Spotter iteration: grabs a camera frame, identifies workers,
		estimates their position, sends them individual updates, overlays debug/mask information on the camera frame,
		saves it to a file and processes any keyboard input.
		:return: None if everything went well; Datetime with the time at which to stop the flight if something went wrong
		(this allows to save debug information for a few extra seconds even when the drones stopped flying and maybe see better why they crashed)
		"""
		try:  # First, run the cv algorithm to estimate the CF's position
			new_pos_arr = self.detect_cf_in_camera()
		except:  # Only way detect_cf_in_camera raises an Exception is if a camera frame couldn't be grabbed
			logging.exception("Couldn't grab a frame from the camera. Exiting")
			return datetime.now()  # Need to stop now (if I wanted to stop in now+2sec and camera kept throwing exceptions, it would keep delaying the stop and never actually stop)

		# Now that we know where the drone currently is, send messages to control it (roll-pitch-yaw-thrust setpoints)
		for i,w in enumerate(self.workers):
			w.control_cf(self.img_to_cf_world_coords(new_pos_arr[i]), self.t_frame)
		self.t_events.append(datetime.now())

		# And save the intermediate and output frames/images to disk for debugging
		self.save_algo_iteration()

		# Last, process kb input to control the experiment
		if not self.process_kb_input():  # Process kb input and take action if necessary (will return False when user wants to stop the experiment)
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
	s = Spotter(bool_ground_truth=False, bool_world_coords_pattern=True)
	s.run_experiment(1)
