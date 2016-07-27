"""
	Main code that implements full control (X, Y, Z [, yaw?]) of a Crazyflie using a camera through OpenCV.
	The algorithm basically does the following:
	 - Fly at a set constant thrust to take off, until a key ("e") is pressed.
		When that happens, current position is used as target position to hold
	 - Using the camera, we find the x,y coordinates of the drone, and estimate z based on size of a known-size object
	 - Independent PIV loops (position + velocity PIDs) control movement in each direction (x, y, z)
	 - ASDW keys can modify the target position to hold, any other key will kill the algorithm and stop the drone.
		When that happens, all the control data (from PIV loops) that's been logged will be displayed and saved to files
	 - NOTE: Z control still hasn't been implemented and X and Y PID constants should be tuned to reduce overshoot
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

from uvc_capture import UvcCapture
from operator import attrgetter
import PID
import plot_tools

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib import crtp


class Hover:
	COLOR_BALL_TRACKED = (255, 0, 0)
	COLOR_BALL_UNTRACKED = (0, 0, 255)
	COLOR_LINE_TRACKED = (255, 0, 0)
	COLOR_LINE_UNTRACKED = (0, 0, 255)
	COLOR_TARGET_TRACKED = (0, 255, 0)
	COLOR_TARGET_UNTRACKED = (0, 0, 255)
	FIGURE_NAME = "Output"
	VIDEO_FOLDER = "img/{}".format(str(datetime.now())[:-7])
	CAMERA_SETTINGS_FILE = "UVCcam settings - USB 2.0 Camera.txt"
	# CAMERA_SETTINGS_FILE = "UVCcam settings - USB 2.0 Camera - TBSI3.txt"
	ASK_FOR_TARGET_YAW = False
	TAKEOFF_THRUST = 43000  # 47000

	def __init__(self):
		self.m_bConnecting = True
		self.m_bConnected = False
		self.m_bTakingOff = True
		self.m_CrazyFlie = None
		self.m_log_control_data = plot_tools.OverallControlLog()
		self.m_log_stab = None
		self.m_roll = 0
		self.m_pitch = 0
		self.m_yaw = 0
		self.m_str_status = "TAKING OFF"
		self.m_t_start = self.m_t_last_frame = datetime.now()
		self.m_video_capture = None
		self.m_imshow_out = None
		self.m_window_kb = None
		self.m_HSV_thresh_min = np.array([10,  90,  70], dtype=np.uint8)     # Orange ping pong ball
		# self.m_HSV_thresh_min = np.array([10,  100,  60], dtype=np.uint8)     # Orange ping pong ball
		self.m_HSV_thresh_max = np.array([30, 255, 255], dtype=np.uint8)    # Orange ping pong ball
		self.DRONE_DETECTOR_PARAMS = self.init_detector_params()
		self.m_drone_pos_tracked = False
		self.m_PID_roll = PID.PIDposAndVel(posP=0.5, velP=0.05, velI=0.01, vel_offs=0, pos_out_max=300, vel_out_max=30, vel_invert_error=True)  # old_offs=7
		self.m_PID_pitch = PID.PIDposAndVel(posP=0.5, velP=0.2, velI=0.002, vel_offs=0, pos_out_max=30, vel_out_max=30)
		self.m_PID_yaw = PID.PID(P=0.5, I=0.3, D=0, offs=0, out_max=20, invert_error=True, error_in_degrees=True)
		self.m_PID_thrust = PID.PIDposAndVel(posP=1, velP=35, velI=25, vel_offs=41000, pos_out_max=300, vel_out_max=6000, pos_invert_error=True, vel_invert_input=True)  # old_offs=46000

	def init_detector_params(self):
		detector_params = cv2.SimpleBlobDetector_Params()

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

		return detector_params

	def init_video_cam(self, create_video_folder=True):
		self.m_video_capture = UvcCapture(0)
		if self.m_video_capture is None:
			raise Exception("Couldn't open camera! :(")

		logging.info("Camera opened! :)")
		if self.CAMERA_SETTINGS_FILE:  # Settings file already contains frame size & rate
			self.m_video_capture.load_settings(self.CAMERA_SETTINGS_FILE)
		else:  # If no settings specified, choose best frame size with fps >= 60
			self.m_video_capture.select_best_frame_mode(60)

		# Sometimes, first couple frames take a long time to be obtained, do it before quad goes in flying mode
		self.m_video_capture.get_frame_robust()
		self.m_video_capture.get_frame_robust()

		self.m_PID_roll.setSetPoint(self.m_video_capture.frame_size[0]/2)
		self.m_PID_thrust.setSetPoint(self.m_video_capture.frame_size[1]/2)
		self.m_PID_pitch.setSetPoint(40)
		self.m_PID_roll.PIDpos.curr_input = self.m_PID_roll.getSetPoint()
		self.m_PID_thrust.PIDpos.curr_input = self.m_PID_thrust.getSetPoint()
		self.m_PID_pitch.PIDpos.curr_input = self.m_PID_pitch.getSetPoint()
		cv2.namedWindow(self.FIGURE_NAME)
		cv2.setMouseCallback(self.FIGURE_NAME, self.on_mouse_callback)
		cv2.moveWindow(self.FIGURE_NAME, 200, 200)

		if create_video_folder:
			# Prepare the folder self.m_video_folder so we can log each frame processed (for debugging)
			shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=True)
			# if os.path.exists(self.VIDEO_FOLDER):
			# 	for f in os.listdir(self.VIDEO_FOLDER): os.remove(os.path.join(self.VIDEO_FOLDER, f))
			# else:
			os.makedirs(self.VIDEO_FOLDER)

	def run(self):
		# logging.basicConfig(level=logging.INFO)  # Initialize logger, only show messages of level INFO or higher
		logging.disable(logging.DEBUG)  # Seems to work better than basicConfig(INFO), especially if logging has already been initialized

		logging.info("Initializing drivers.")
		crtp.init_drivers(enable_debug_driver=False)

		logging.info("Setting radio link.")
		availableLinks = crtp.scan_interfaces()
		if len(availableLinks) == 0:
			logging.error("Error, no Crazyflies found. Exiting.")
			return
		else:
			linkUri = availableLinks[0][0]  # Choose first available link
			logging.info("Crazyflies found:")
			for i in availableLinks:
				logging.info("\t" + i[0])

		logging.info("Initializing Crazyflie (connecting to first available interface, '{}').".format(linkUri))
		self.m_CrazyFlie = Crazyflie(ro_cache="cachero", rw_cache="cacherw")
		self.m_CrazyFlie.connected.add_callback(self.on_connected)
		self.m_CrazyFlie.disconnected.add_callback(self.on_disconnected)
		self.m_CrazyFlie.connection_failed.add_callback(self.on_connection_failed)
		self.m_CrazyFlie.connection_lost.add_callback(self.on_connection_lost)
		self.m_CrazyFlie.open_link(linkUri)

		save_logs = True  # Use this auxiliary variable to prevent saving logs if the drone never took off
		try:
			cnt = 0
			while self.m_bConnecting:
				cnt += 1
				# if cnt >= 20:
				#     raise Exception("Connection to Crazyflie timed out, unable to establish a useful link after 20sec.")
				# if cnt % 20 == 0:
				#     logging.warning("Unable to establish a connection with Crazyflie (%s) after 20 sec. Retrying..." % linkUri)
				#     self.m_CrazyFlie.close_link()
				#     self.m_bConnecting = True
				#     self.m_CrazyFlie.open_link(linkUri)
				time.sleep(1)

			if not self.m_bConnected:
				print "Something failed while attempting to connect to the drone, exiting."
				return

			# Can't initialize LogConfig until we're connected, because it needs to check that the variables we'd like to add are in the TOC
			self.m_log_stab = LogConfig(name="Stabilizer", period_in_ms=10)
			self.m_log_stab.add_variable("stabilizer.roll", "float")
			self.m_log_stab.add_variable("stabilizer.pitch", "float")
			self.m_log_stab.add_variable("stabilizer.yaw", "float")

			try:
				self.m_CrazyFlie.param.set_value("ring.effect", "0")  # Turn off LED ring
				self.m_CrazyFlie.param.set_value("ring.headlightEnable", "0")  # Turn off LED ring
				self.m_CrazyFlie.log.add_config(self.m_log_stab)
				self.m_log_stab.data_received_cb.add_callback(self.log_stabilizer_data)
				self.m_log_stab.error_cb.add_callback(self.log_stabilizer_error)
				self.m_log_stab.start()
			except KeyError as e:
				raise Exception("Couldn't start log configuration, %s not found in TOC!" % str(e))
			except AttributeError:
				raise Exception("Couldn't add Stabilizer log config, bad configuration.")

			if self.ASK_FOR_TARGET_YAW:
				raw_input("\nRotate the drone so it faces the camera, press Enter when you're ready...\n")
			else:
				while abs(self.m_yaw) < 0.01:
					time.sleep(0.1)  # Wait until first m_yaw value is received
			self.m_PID_yaw.SetPoint = self.m_yaw
			print "Target yaw set at %.2f." % self.m_yaw

			self.init_video_cam()
			sdl2.ext.init()
			self.m_window_kb = sdl2.ext.Window("Window to receive keyboard input", size=(400, 300))
			self.m_window_kb.show()

			# Also take an image from the background, for background substraction
			print "Giving you 5s to prepare for take-off..."
			time.sleep(5)
			# frame = self.m_video_capture.get_frame_robust()
			# cv2.imshow(self.FIGURE_NAME, frame)
			# cv2.waitKey(5000)  # Give 5 sec to prepare for take-off

			# t = Timer(20, self.m_CrazyFlie.close_link)  # Start a timer to disconnect in 10s
			# t.start()

			self.m_str_status = "TAKING OFF"
			self.m_t_start = datetime.now()
			self.m_PID_roll.clear()
			self.m_PID_pitch.clear()
			self.m_PID_yaw.clear()
			self.m_PID_thrust.clear()

			tStop = None
			while tStop is None:
				tStop = self.hover()

			print "AT t={}, A KEY WAS PRESSED -> STOPPING!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])
			save_logs = (self.m_str_status != "TAKING OFF")
			self.m_str_status = "STOPPED"

			while datetime.now() < tStop:
				self.hover()

			self.m_CrazyFlie.commander.send_setpoint(0, 0, 0, 0)

		except Exception as e:
			logging.exception(" Shutting down due to an exception =( See details below:")
			# exc_type, exc_obj, exc_tb = sys.exc_info()
			# logging.error("Shutting down due to an exception -> %s: %s (at %s:%d)." % (str(exc_type), str(e), os.path.split(exc_tb.tb_frame.f_code.co_filename)[1], exc_tb.tb_lineno))
		self.m_CrazyFlie.close_link()
		self.m_video_capture = None
		cv2.destroyAllWindows()
		self.m_window_kb.hide()
		sdl2.ext.quit()
		if save_logs:
			self.m_log_control_data.plot(False)
		else:
			shutil.rmtree(self.VIDEO_FOLDER, ignore_errors=False)

	def on_connected(self, linkUri):
		logging.info("Successfully connected to Crazyflie at '%s'!" % linkUri)
		self.m_bConnecting = False
		self.m_bConnected = True

	def on_connection_failed(self, link_uri, msg):
		logging.error("Connection to '%s' failed: %s." % (link_uri, msg))  # Initial connection fails (i.e no Crazyflie at the speficied address)
		self.m_bConnecting = False
		self.m_bConnected = False

	def on_connection_lost(self, link_uri, msg):
		logging.warning("Connection to '%s' lost: %s." % (link_uri, msg))  # Disconnected after a connection has been made (i.e Crazyflie moves out of range)

	def on_disconnected(self, link_uri):
		logging.error("Disconnected from '%s'." % link_uri)  # Crazyflie is disconnected (called in all cases)
		self.m_bConnecting = False
		self.m_bConnected = False

	def on_mouse_callback(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN: # & self.m_imshow_out is not None:
			try:
				px = self.m_imshow_out[y, x]
				px_HSV = cv2.cvtColor(np.array([[px]]), cv2.COLOR_BGR2HSV).ravel()
				print "MOUSE_CLICK! At: (x=%d, y=%d). Pixel value: [H=%d, S=%d, V=%d] = [R=%d, G=%d, B=%d]" % (x, y, px_HSV[0], px_HSV[1], px_HSV[2], px[2], px[1], px[0])
			except:
				print "MOUSE_CLICK! At: (x={}, y={}). Outside cam image or no pixel value available".format(x, y)

	def print_Crazyflie_param_TOC(self):
		print "List of Crazyflie params (available in the TOC):"

		p_toc = self.m_CrazyFlie.param.toc.toc
		for group in sorted(p_toc.keys()):
			print group
			for param in sorted(p_toc[group].keys()):
				print "\t" + param

	def log_stabilizer_error(self, logconf, msg):
		logging.error("Error when logging %s: %s." % (logconf.name, msg))

	def log_stabilizer_data(self, timestamp, data, logconf):
		logging.debug("[%d][%s]: %s" % (timestamp, logconf.name, data))
		self.m_roll = data['stabilizer.roll']
		self.m_pitch = data['stabilizer.pitch']
		self.m_yaw = data['stabilizer.yaw']
		print "\rCurrent yaw: %.2fdeg" % self.m_yaw,

	def get_OSD_text(self, t):
		formatNum = "{:+6.2f}"
		strPrint = ("ROLLp. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"ROLLv. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"PITCHp={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"PITCHv={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"YAW..  ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"THRUSp={:3.0f};{:6.0f} [{:+6.0f}, {:+6.0f}, {:+6.0f}]\t\t" +
					"THRUSv={:3.0f};{:6.0f} [{:+6.0f}, {:+6.0f}, {:+6.0f}]\t\t" +
					"[x:{:4.0f}, y:{:4.0f}, z:{:4.0f}], [vx:{:4.0f}, vy:{:4.0f}, vz:{:4.0f}], rpy: " + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
					"@{} (FPS: {:5.2f}) - " + self.m_str_status).format(
			self.m_PID_roll.PIDpos.SetPoint, self.m_PID_roll.PIDpos.output, self.m_PID_roll.PIDpos.PTerm, self.m_PID_roll.PIDpos.Ki*self.m_PID_roll.PIDpos.ITerm, self.m_PID_roll.PIDpos.Kd*self.m_PID_roll.PIDpos.DTerm,
			self.m_PID_roll.PIDvel.SetPoint, self.m_PID_roll.PIDvel.output, self.m_PID_roll.PIDvel.PTerm, self.m_PID_roll.PIDvel.Ki*self.m_PID_roll.PIDvel.ITerm, self.m_PID_roll.PIDvel.Kd*self.m_PID_roll.PIDvel.DTerm,
			self.m_PID_pitch.PIDpos.SetPoint, self.m_PID_pitch.PIDpos.output, self.m_PID_pitch.PIDpos.PTerm, self.m_PID_pitch.PIDpos.Ki*self.m_PID_pitch.PIDpos.ITerm, self.m_PID_pitch.PIDpos.Kd*self.m_PID_pitch.PIDpos.DTerm,
			self.m_PID_pitch.PIDvel.SetPoint, self.m_PID_pitch.PIDvel.output, self.m_PID_pitch.PIDvel.PTerm, self.m_PID_pitch.PIDvel.Ki*self.m_PID_pitch.PIDvel.ITerm, self.m_PID_pitch.PIDvel.Kd*self.m_PID_pitch.PIDvel.DTerm,
			self.m_PID_yaw.SetPoint, self.m_PID_yaw.output, self.m_PID_yaw.PTerm, self.m_PID_yaw.Ki * self.m_PID_yaw.ITerm, self.m_PID_yaw.Kd * self.m_PID_yaw.DTerm,
			self.m_PID_thrust.PIDpos.SetPoint, self.m_PID_thrust.PIDpos.output, self.m_PID_thrust.PIDpos.PTerm, self.m_PID_thrust.PIDpos.Ki*self.m_PID_thrust.PIDpos.ITerm, self.m_PID_thrust.PIDpos.Kd*self.m_PID_thrust.PIDpos.DTerm,
			self.m_PID_thrust.PIDvel.SetPoint, self.m_PID_thrust.PIDvel.output, self.m_PID_thrust.PIDvel.PTerm, self.m_PID_thrust.PIDvel.Ki*self.m_PID_thrust.PIDvel.ITerm, self.m_PID_thrust.PIDvel.Kd*self.m_PID_thrust.PIDvel.DTerm,
			self.m_PID_roll.getCurrPos(), self.m_PID_thrust.getCurrPos(), self.m_PID_pitch.getCurrPos(),
			self.m_PID_roll.getCurrVel(), self.m_PID_thrust.getCurrVel(), self.m_PID_pitch.getCurrVel(), self.m_roll, self.m_pitch, self.m_yaw,
			str(t-self.m_t_start)[3:-3], 1./(t-self.m_t_last_frame).total_seconds())

		# logging.debug(strPrint)
		return "          SP | SENT  [   P   ,   I   ,   D  ]\t\t" + strPrint

	def add_OSD_to_img(self, img, mask, drone_curr_pos, drone_target_pos, str_OSD, newline_separator='\t\t', margin_x=25, margin_y=25, text_color=(200,200,200), font=cv2.FONT_HERSHEY_DUPLEX, font_scale=0.7, font_thickness=1, line_type=cv2.LINE_AA, mask_color=[255,0,255], img_resize_factor=0.5, save_orig=False, t_arr=[]):
		if save_orig:
			self.m_imshow_out = cv2.resize(img, None, fx=img_resize_factor, fy=img_resize_factor)
			t_arr.append(datetime.now())

		cv2.circle(img, tuple(drone_curr_pos[0:2].astype(int)), int(drone_curr_pos[2]), self.COLOR_BALL_TRACKED if self.m_drone_pos_tracked else self.COLOR_BALL_UNTRACKED, -1)
		cv2.line(img, tuple(drone_curr_pos[0:2].astype(int)), tuple(drone_target_pos[0:2].astype(int)), self.COLOR_LINE_TRACKED if self.m_drone_pos_tracked else self.COLOR_LINE_UNTRACKED, 10)
		cv2.circle(img, tuple(drone_target_pos[0:2].astype(int)), int(drone_target_pos[2]), self.COLOR_TARGET_TRACKED if self.m_drone_pos_tracked else self.COLOR_TARGET_UNTRACKED, -1)  # Plot circle at desired drone location
		t_arr.append(datetime.now())

		np.putmask(img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)==255, list(mask_color))
		t_arr.append(datetime.now())

		lines = str_OSD.split(newline_separator)
		out = np.zeros((int(img_resize_factor*img.shape[0] + (len(lines)+1)*margin_y), int(img_resize_factor*img.shape[1]), img.shape[2]), dtype=img.dtype)
		out[0:int(img_resize_factor*img.shape[0]), :] = cv2.resize(img, None, fx=img_resize_factor, fy=img_resize_factor)
		t_arr.append(datetime.now())

		for cnt, l in enumerate(lines):
			cv2.putText(out, l.replace('\t', '; '), (margin_x, out.shape[0] + int(margin_y*(cnt+0.5-len(lines)))), font, font_scale, text_color, font_thickness, line_type)

		return out

	def hover(self, sendCommand=True, saveImg=True):
		#########################################################
		#                       CAPTURE FRAME                   #
		#########################################################
		t_arr = [datetime.now()]
		try:
			uvc_frame = self.m_video_capture.get_frame_robust()  # read() blocks execution until a new frame arrives! -> Obtain t AFTER grabbing the frame
			t_arr.append(datetime.now())
			t = t_arr[-1]
			frame = uvc_frame.bgr   # .copy()
			t_arr.append(datetime.now())
		except:
			logging.error("Unexpected error accessing the camera frame :(")
			return datetime.now()

		#########################################################
		#                     COLOR THRESHOLD                   #
		#########################################################
		# frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
		# t_arr.append(datetime.now())
		# frame = cv2.GaussianBlur(frame, (3, 3), 0)  # Not needed, camera is already physically "blurring" (sharpness parameter set to 0)
		# t_arr.append(datetime.now())
		frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		t_arr.append(datetime.now())
		# mask = cv2.inRange(frame_HSV, np.array([110, 30, 150], np.uint8), np.array([130, 160, 255], np.uint8))  # Office
		# mask = cv2.inRange(frame_HSV, np.array([120, 40, 50], np.uint8), np.array([150, 160, 255], np.uint8))  # Bedroom old cam
		mask = cv2.inRange(frame_HSV, self.m_HSV_thresh_min, self.m_HSV_thresh_max)  # Bedroom new cam
		t_arr.append(datetime.now())
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
		t_arr.append(datetime.now())
		# print "{:.3f} ms".format((t_arr[-1]-t_arr[-2]).total_seconds()*1000)

		#########################################################
		#                     DRONE DETECTION                   #
		#########################################################
		keypoints = cv2.SimpleBlobDetector_create(self.DRONE_DETECTOR_PARAMS).detect(mask)
		t_arr.append(datetime.now())
		if keypoints:
			keypoint = max(keypoints, key=attrgetter('size'))   # Keep only the biggest blob
		else:
			keypoint = type('', (), {'pt': np.array([self.m_PID_roll.getCurrPos(), self.m_PID_thrust.getCurrPos()]) +
										   np.array([self.m_PID_roll.getCurrVel(), -self.m_PID_thrust.getCurrVel()])*(t-self.m_PID_thrust.PIDvel.curr_time).total_seconds(),
									 'size': max(1, self.m_PID_pitch.getCurrPos() + self.m_PID_pitch.getCurrVel()*(t-self.m_PID_thrust.PIDvel.curr_time).total_seconds())})  # Since PIDs haven't been updated with current values yet, don't have to multiply velocity by (curr_time-last_time) but rather by (t-curr_time)

		self.m_drone_pos_tracked = bool(keypoints)
		drone_curr_pos = np.hstack((keypoint.pt, keypoint.size/2))  # And update current position based on the CV algorithm output
		drone_target_pos = np.array([int(round(x)) for x in [self.m_PID_roll.getSetPoint(), self.m_PID_thrust.getSetPoint(), self.m_PID_pitch.getSetPoint()]])
		self.m_PID_roll.update(drone_curr_pos[0], t)
		self.m_PID_pitch.update(drone_curr_pos[2], t)
		self.m_PID_yaw.update(self.m_yaw, t)
		self.m_PID_thrust.update(drone_curr_pos[1], t)
		self.m_log_control_data.update(self.m_PID_roll, self.m_PID_pitch, self.m_PID_yaw, self.m_PID_thrust)

		if sendCommand:
			if self.m_bConnected:
				if self.m_bTakingOff:
					self.m_CrazyFlie.commander.send_setpoint(self.m_PID_roll.PIDvel.out_offs, self.m_PID_pitch.PIDvel.out_offs, self.m_PID_yaw.output, self.TAKEOFF_THRUST)
				else:
					self.m_CrazyFlie.commander.send_setpoint(self.m_PID_roll.getOutput(), self.m_PID_pitch.getOutput(), self.m_PID_yaw.output, self.m_PID_thrust.getOutput())
			else:
				self.m_CrazyFlie.commander.send_setpoint(0, 0, 0, 0)
				self.m_PID_roll.clear(); self.m_PID_pitch.clear(); self.m_PID_yaw.clear(); self.m_PID_thrust.clear()

		t_arr.append(datetime.now())
		str_OSD = self.get_OSD_text(t)
		t_arr.append(datetime.now())
		if saveImg or not sendCommand:
			out = self.add_OSD_to_img(frame, mask, drone_curr_pos, drone_target_pos, str_OSD, save_orig=(not sendCommand), t_arr=t_arr)
			t_arr.append(datetime.now())
			if False:
				t1 = datetime.now()
				_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
				contour = contours[0]
				r = cv2.boundingRect(contour)
				c = cv2.minEnclosingCircle(contour)
				cv2.rectangle(out, r[0:2], (r[0]+r[2], r[1]+r[3]), (0,255,0), 2, cv2.LINE_AA)
				cv2.circle(out, tuple(int(x) for x in c[0]), int(c[1]), (0, 255, 0), 2, cv2.LINE_AA)
				t2 = datetime.now()
				print "\t\t{} ms;\t\t\tc:{}\t\tblob:{}".format((t2 - t1).total_seconds() * 1000, c[1], keypoint.size/2)
			if saveImg:
				cv2.imwrite(os.path.join(self.VIDEO_FOLDER, t.strftime("out_%H-%M-%S-%f.jpg")), out)
				t_arr.append(datetime.now())
			if not sendCommand:  # Only display the images if in CV-only mode, otherwise I'm looking at the drone so who cares if the image is shown live, I'll look at the (saved) individual frames later
				cv2.imshow(self.FIGURE_NAME, out)
				if cv2.waitKey(1) >= 0:
					return datetime.now()
				t_arr.append(datetime.now())

		if sendCommand:
			events = sdl2.ext.get_events()
			for event in events:
				if event.type == sdl2.SDL_KEYDOWN:
					try:
						key = chr(event.key.keysym.sym).lower()
						self.m_window_kb.title = "Last key pressed: {} at t={}".format(chr(event.key.keysym.sym), str(datetime.now().time())[:-3])
						print "Key: '{}'".format(key)
						if key == 'a':
							self.m_PID_roll.setSetPoint(self.m_PID_roll.getSetPoint() + 20)
						elif key == 's':
							self.m_PID_pitch.setSetPoint(max(self.m_PID_pitch.getSetPoint() - 2, 1))
						elif key == 'd':
							self.m_PID_roll.setSetPoint(self.m_PID_roll.getSetPoint() - 20)
						elif key == 'w':
							self.m_PID_pitch.setSetPoint(self.m_PID_pitch.getSetPoint() + 2)
						elif key == 'u':
							self.m_PID_thrust.setSetPoint(self.m_PID_thrust.getSetPoint() - 20)
						elif key == 'h':
							self.m_PID_thrust.setSetPoint(self.m_PID_thrust.getSetPoint() + 20)
						elif key == 'e':
							self.m_bTakingOff = False
							self.m_str_status = "FLYING"
							self.m_PID_roll.setSetPoint(self.m_PID_roll.getCurrPos())
							self.m_PID_pitch.setSetPoint(self.m_PID_pitch.getCurrPos() + 2)
							self.m_PID_thrust.setSetPoint(self.m_PID_thrust.getCurrPos())
							self.m_PID_roll.clear()
							self.m_PID_pitch.clear()
							self.m_PID_thrust.clear()
						else:
							self.m_bConnected = False
							return datetime.now() + timedelta(seconds=2)
					except:
						key = event.key.keysym.sym
						self.m_window_kb.title = "Last key pressed: \\x{:x} at t={}".format(key, str(datetime.now().time())[:-3])
						if key == sdl2.SDLK_UP:
							print "Up!"
						elif key == sdl2.SDLK_DOWN:
							print "Down!"
						elif key == sdl2.SDLK_ESCAPE:
							print "ESC!"
						self.m_bConnected = False
						return datetime.now() + timedelta(seconds=2)
			t_arr.append(datetime.now())

		logging.debug("DeltaT = {:5.2f}ms -> Total: {:5.2f}ms{}".format((datetime.now()-t).total_seconds()*1000, (t-self.m_t_last_frame).total_seconds()*1000, "".join(["\t{}->{}: {}ms".format(i+1, i+2, (t_arr[i+1]-t_arr[i]).total_seconds()*1000) for i in range(len(t_arr)-1)])))
		self.m_t_last_frame = t

		return None

def HSV_thresh_trackbar_changed(self, is_min, hsv_index, new_value):
	if is_min:
		self.m_HSV_thresh_min[hsv_index] = new_value
	else:
		self.m_HSV_thresh_max[hsv_index] = new_value

if __name__ == '__main__':
	sendCommands = True
	sendCommands = False
	h = Hover()
	if sendCommands:
		h.run()
	else:
		logging.disable(logging.DEBUG)
		h.init_video_cam(False)
		h.m_bConnected = True
		cv2.namedWindow(h.FIGURE_NAME)
		cv2.createTrackbar("H_min", h.FIGURE_NAME, h.m_HSV_thresh_min[0], 179, lambda x: HSV_thresh_trackbar_changed(h, True, 0, x))
		cv2.createTrackbar("H_max", h.FIGURE_NAME, h.m_HSV_thresh_max[0], 179, lambda x: HSV_thresh_trackbar_changed(h, False, 0, x))
		cv2.createTrackbar("S_min", h.FIGURE_NAME, h.m_HSV_thresh_min[1], 255, lambda x: HSV_thresh_trackbar_changed(h, True, 1, x))
		cv2.createTrackbar("S_max", h.FIGURE_NAME, h.m_HSV_thresh_max[1], 255, lambda x: HSV_thresh_trackbar_changed(h, False, 1, x))
		cv2.createTrackbar("V_min", h.FIGURE_NAME, h.m_HSV_thresh_min[2], 255, lambda x: HSV_thresh_trackbar_changed(h, True, 2, x))
		cv2.createTrackbar("V_max", h.FIGURE_NAME, h.m_HSV_thresh_max[2], 255, lambda x: HSV_thresh_trackbar_changed(h, False, 2, x))
		if False:  # if True, allows the tuning of the BlobDetector_Params
			# cv2.createTrackbar("useArea", h.FIGURE_NAME, h.DRONE_DETECTOR_PARAMS.filterByArea, 1, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "filterByArea", bool(x)))
			h.DRONE_DETECTOR_PARAMS.filterByArea = True
			cv2.createTrackbar("minArea", h.FIGURE_NAME, int(h.DRONE_DETECTOR_PARAMS.minArea/10), 100, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "minArea", x*10))
			cv2.createTrackbar("maxArea", h.FIGURE_NAME, int(h.DRONE_DETECTOR_PARAMS.maxArea/1000), 100, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "maxArea", x*1000))
			# cv2.createTrackbar("useCirc", h.FIGURE_NAME, h.DRONE_DETECTOR_PARAMS.filterByCircularity, 1, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "filterByCircularity", bool(x)))
			h.DRONE_DETECTOR_PARAMS.filterByCircularity = True
			cv2.createTrackbar("minCirc", h.FIGURE_NAME, int(h.DRONE_DETECTOR_PARAMS.minCircularity*100), 100, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "minCircularity", x/100.0))
			cv2.createTrackbar("maxCirc", h.FIGURE_NAME, int(h.DRONE_DETECTOR_PARAMS.maxCircularity*100), 100, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "maxCircularity", x/100.0))
			# cv2.createTrackbar("useInertia", h.FIGURE_NAME, h.DRONE_DETECTOR_PARAMS.filterByInertia, 1, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "filterByInertia", bool(x)))
			h.DRONE_DETECTOR_PARAMS.filterByInertia = True
			cv2.createTrackbar("minInertia", h.FIGURE_NAME, int(h.DRONE_DETECTOR_PARAMS.minInertiaRatio*100), 100, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "minInertiaRatio", x/100.0))
			cv2.createTrackbar("maxInertia", h.FIGURE_NAME, int(h.DRONE_DETECTOR_PARAMS.maxInertiaRatio*100), 100, lambda x: setattr(h.DRONE_DETECTOR_PARAMS, "maxInertiaRatio", x/100.0))

		while h.hover(False, False) is None:
			pass
