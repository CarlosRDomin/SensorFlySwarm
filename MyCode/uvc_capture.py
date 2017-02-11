"""
	Wrapper class around uvc.Capture to add custom methods to the camera capture class, like saving and loading settings!
"""

import uvc
import re
import logging
import numpy as np
try:  # OpenCV is not necessary for most of the functionality, so just keep track of whether it's installed or not
	import cv2
except:
	CV2_EXISTS = False
else:
	CV2_EXISTS = True


def ensure_cv2_exists():
	"""
	OpenCV is not necessary for most of the functionality in this library, but some functions do require it.
	This function simply raises an Exception if a cv2-dependent function is called but cv2 is not installed.
	"""
	if not CV2_EXISTS:  # Make sure we have imported cv2
		raise NotImplementedError("Module cv2 is not installed, but is required for camera calibration/undistortion.")


class UvcCapture(uvc.Capture):

	SETTINGS_SEPARATOR = '\t'  # String used to separate setting name from setting value (don't use ',' as some setting names might contain commas)
	CALIB_FILE_NONE = "<No calibration file provided>"
	logging.basicConfig(level=logging.DEBUG)

	def __new__(cls, cam_name, cam_vend_id=-1, cam_prod_id=-1, cam_serial_num=None, *more):
		"""
		Class creator which prevents an UvcCapture object from being created if we can't connect to the specified camera.
		It also finds the UID of the camera for us, so we can choose the camera more easily through cam name or index.
		:param cam_name: If str, it's the desired camera name; If int, the index in the list of available devices to connect to
		:param cam_vend_id: If specified (-1=any), it indicates the vendor id of the camera we want to connect to.
		:param cam_prod_id: If specified (-1=any), it indicates the product id of the camera we want to connect to.
		:param cam_serial_num: If specified (None=any), it indicates the serial number of the camera we want to connect to.
		This is particularly useful when we use more than one camera with the same USB camera name, vendor ID, product ID...
		:param more: Any other params that would be passed on to the uvc.Capture creator
		:return: UvcCapture object connected to the specified camera, or None if there was an error (eg: camera not found)
		"""
		cam = None
		device_list = uvc.device_list()
		if isinstance(cam_name, int):  # cam_name is int -> Find by camera index
			if cam_name < len(device_list):
				cam = device_list[cam_name]  # If index is within bounds, use that device, but check for additional constraints
				if (cam_vend_id >= 0 and cam['idVendor'] != cam_vend_id) or (cam_prod_id >= 0 and cam['idProduct'] != cam_prod_id) or (cam_serial_num is not None and cam['serialNumber'] != cam_serial_num):
					logging.info("Camera with index cam_name={} doesn't match additional constraints (vend_id={}, prod_id={}, serial_num={})".format(cam_name, cam_vend_id, cam_prod_id, cam_serial_num))
					cam = None  # Additional constraints not met, don't use this device
			else:
				logging.info("Provided camera index (cam_name={}) out of range (there's only {} devices)".format(cam_name, len(device_list)))
		else:
			for d in device_list:  # From the available devices list, find the one we want
				if d['name'] == cam_name and (cam_vend_id < 0 or cam_vend_id == d['idVendor']) and (cam_prod_id < 0 or cam_prod_id == d['idProduct']) and (cam_serial_num is None or cam_serial_num == d['serialNumber']):
					cam = d
					break

		if cam is None:  # This only happens if we couldn't find the camera from the information we were provided
			logging.warning("Couldn't find camera {} in the list of available UVC devices, aborting creation of {} object.".format(UvcCapture.get_human_friendly_identifier(cam_name, cam_vend_id, cam_prod_id, cam_serial_num), UvcCapture))
			return None  # Don't allow the creation of the object -> Return None

		obj = super(UvcCapture, cls).__new__(cls, more)  # Create a new UvcCapture object by calling the parent constructor
		obj.vend_id = cam['idVendor']
		obj.prod_id = cam['idProduct']
		obj.serial_num = cam['serialNumber']
		obj.provided_vend_id = (cam_vend_id >= 0)
		obj.provided_prod_id = (cam_prod_id >= 0)
		obj.provided_serial_num = (cam_serial_num is not None)
		try:
			super(UvcCapture, obj).__init__(cam['uid'])  # Initialize the object using the camera UID (uvc.Capture opens the device)
			return obj  # Finally return the newly created object if everything went fine
		except:
			logging.error("Something went wrong opening camera {}, is it already open by another process? Aborting creation of {} object.".format(UvcCapture.get_human_friendly_identifier(cam_name, cam_vend_id, cam_prod_id, cam_serial_num), UvcCapture))
			return None  # In case of an error (eg: the camera is already in use) return None

	def __init__(self, *args, **kwargs):  # Initialization of underlying uvc.Capture object was already done in __new__
		# We still need to explicitly add this method or else the super.__init__ method will be called (and it'll try to open a camera with uid=cam_name, so it'll obviously fail)
		self.calib_file = self.CALIB_FILE_NONE
		self.calib_frame_size = self.orig_camera_matrix = self.camera_matrix = self.new_camera_matrix = self.dist_coefs = self.calib_at = None
		self.undist_maps = None
		self.do_undistort = False  # True/False to indicate whether we want to undistort

	@staticmethod
	def device_list():
		"""
		Returns a list of currently available devices.
		:return: List containing currently available devices
		"""
		return uvc.device_list()

	@staticmethod
	def get_human_friendly_identifier(cam_name, cam_vend_id=-1, cam_prod_id=-1, cam_serial_num=None):
		"""
		Produces a human-friendly string identifier of the camera (eg: "'USB Camera' [VID:5024, PID:3546, #serialnumhere]")
		:param cam_name: str indicating the camera's name or int as the index of the camera within the list of available devices
		:param cam_vend_id: (Optional) constrains the vendor id of the camera
		:param cam_prod_id: (Optional) constrains the product id of the camera
		:param cam_serial_num: (Optional) constrains the serial number of the camera
		:return: Human-friendly str containing the name of the camera, as well as any additional constraints
		provided by the user to identify it (vendor id, product id, serial number), if any.
		"""
		str_cam_name = "'{}'".format(cam_name) if isinstance(cam_name, str) else str(cam_name)  # Surround by quotes if cam_name is str, otherwise (int) simply convert to str and use the number to identify the camera
		if cam_vend_id >= 0 or cam_prod_id >= 0 or cam_serial_num is not None:
			str_cam_name += " [{}]".format(", ".join(["" if cam_vend_id < 0 else "VID:{}".format(cam_vend_id), "" if cam_prod_id < 0 else "PID:{}".format(cam_prod_id), "" if cam_serial_num is None else "#{}".format(cam_serial_num)]))

		return str_cam_name

	# def get_human_friendly_identifier(self):
	# 	"""
	# 	Produces a human-friendly string identifier of the camera (eg: "'USB Camera' [VID:5024, PID:3546, #serialnumhere]")
	# 	:return: Human-friendly str containing the name of the camera, as well as any additional constraints
	# 	provided by the user to identify it (vendor id, product id, serial number), if any.
	# 	"""
	# 	return UvcCapture.get_human_friendly_identifier(self.name, self.vend_id if self.provided_vend_id else -1, self.prod_id if self.provided_prod_id else -1, self.serial_num if self.provided_serial_num else None)

	@staticmethod
	def parse_human_friendly_identifier(cam_str):
		"""
		Parses the camera information from a human-friendly string generated by get_human_friendly_identifier().
		:param cam_str: String containing camera name, vendor id, product id and serial number, generated using get_human_friendly_identifier()
		:return: A tuple containing the camera name, vendor id, product id and serial number.
		"""
		name = str(None)
		vend_id = prod_id = -1
		serial_num = None

		# info = re.search('^(.+) \(([---0-9]+)[^---0-9]+([---0-9]+)\)$', str(cam_str))  # Use this one to accept negative vend and prod ids (<0 means accept any that matches the cam name)
		# info = re.search('^(.+) \(([0-9]+)[^0-9]+([0-9]+)\)$', cam_str)
		info = re.search(ur"^(?:'(?P<name_str>.+)'|(?P<name_int>\d+))(?: \[(?:VID:(?P<vid>\d+))?(?:,\s)?(?:PID:(?P<pid>\d+))?(?:,\s)?(?:#(?P<serial>\w+))?\])?$", cam_str)
		if info is not None:
			name = int(info.group("name_int")) if info.group("name_int") is not None else str(info.group("name_str"))
			if info.group("vid") is not None: vend_id = int(info.group("vid"))
			if info.group("pid") is not None: prod_id = int(info.group("pid"))
			if info.group("serial") is not None: serial_num = str(info.group("serial"))

		return name, vend_id, prod_id, serial_num

	@staticmethod
	def _parse_first_line_of_settings(first_line):
		"""
		Provides camera name, vendor id, product id, serial number and calibration file given the first line of a settings file.
		:param first_line: str with the contents of the first line read from a UvcCapture settings file
		:return: A tuple containing the camera name (str), vendor id (int), product id (int), camera serial number (str [or None]), and camera calibration file (str)
		"""
		first_line = first_line.rstrip('\r\n').split(UvcCapture.SETTINGS_SEPARATOR)  # Split first line using SETTINGS_SEPARATOR
		arg_cnt = 0

		cam_name = first_line[arg_cnt]; arg_cnt += 1
		vend_id = int(first_line[arg_cnt]) if len(first_line) > arg_cnt else -1; arg_cnt += 1
		prod_id = int(first_line[arg_cnt]) if len(first_line) > arg_cnt else -1; arg_cnt += 1
		serial_num = first_line[arg_cnt] if len(first_line) > arg_cnt and first_line[arg_cnt] != "None" else None; arg_cnt += 1
		calibration_file = first_line[arg_cnt] if len(first_line) > arg_cnt else UvcCapture.CALIB_FILE_NONE; arg_cnt += 1

		return cam_name, vend_id, prod_id, serial_num, calibration_file

	def save_settings(self, file_name, calib_file=""):
		"""
		Saves current settings (camera identification info as well as parameter values) to a file.
		:param file_name: Name of the file to use when saving the settings
		:param calib_file: (Optional) str containing the path of the camera calibration file, to be included in the settings
		:return: True if everything went well; False if settings couldn't be saved
		"""
		try:
			logging.debug("Saving current UVC camera settings to file '{}'".format(file_name))
			with open(file_name, 'w') as f:
				f.write("{}{}{}{}{}{}{}{}{}\n".format(self.name, self.SETTINGS_SEPARATOR, self.vend_id, self.SETTINGS_SEPARATOR, self.prod_id, self.SETTINGS_SEPARATOR, self.serial_num, self.SETTINGS_SEPARATOR, calib_file))
				f.write("{},{}{}{}\n".format(self.frame_size[0], self.frame_size[1], self.SETTINGS_SEPARATOR, self.frame_rate))
				for c in self.controls:
					f.write("{}{}{}\n".format(c.display_name, self.SETTINGS_SEPARATOR, c.value))
		except:
			logging.exception("Something went wrong while saving current UVCcam settings to '{}'.".format(file_name))
			return False
		return True

	def load_settings(self, file_name):
		"""
		Loads camera settings from a file.
		:param file_name: Path to the file to load the settings from
		:return: True if everything went well; False if settings couldn't be loaded
		"""
		try:
			logging.debug("Loading UVC camera settings from file '{}'".format(file_name))
			with open(file_name, 'r') as f:
				cam_name, vend_id, prod_id, serial_num, calibration_file = UvcCapture._parse_first_line_of_settings(f.readline())  # Read camera info that the settings were created for
				size, fps = f.readline().split(self.SETTINGS_SEPARATOR)
				size = tuple([int(x) for x in size.split(',')])
				fps = int(fps)
				try:  # Especially if settings were saved for a different cam_name, exact same frame size or fps might not be avail.
					self.frame_size = size
					self.frame_rate = fps
				except:  # If something fails, just select the best frame mode that meets the criteria (or highest resolution mode if none meet the minimum requirements)
					logging.info("Couldn't set frame mode to {}x{}@{}fps, selecting best available frame mode. Note that these settings were saved for '{}', is this the same device ('{}')?".format(size[0], size[1], fps, cam_name, self.name))
					self.select_best_frame_mode(min_fps=fps, min_width=size[0], min_height=size[1])
				logging.debug("\tLoaded camera settings: frame_size = {}x{}, frame_rate = {} fps".format(self.frame_size[0], self.frame_size[1], self.frame_rate))
				try:
					self.init_undistort_maps(calibration_file)
				except Exception as e:
					logging.error("\tDidn't initialize undistortion maps (maybe the settings file doesn't include the calibration file in the first line). Details: {}".format(e))

				failed_controls = []  # Some controls might fail to set at first because some other control is set to Auto. Store them in a list and try again after the first pass.
				for line in f:
					name, value = line.split(self.SETTINGS_SEPARATOR)
					for c in self.controls:
						if c.display_name == name:
							try:
								c.set_value(int(value))
								logging.debug("\tLoaded control setting: '{}' = {}".format(c.display_name, c.value))
							except:
								failed_controls.append((c, int(value)))
								logging.debug("\tCouldn't load control setting: '{}' = {}, maybe it's disabled by other setting like 'Auto' modes? Will retry after loading all other params".format(c.display_name, int(value)))
							break
				for c, v in failed_controls:
					try:
						c.set_value(v)
						logging.debug("\tLoaded control setting: '{}' = {}".format(c.display_name, c.value))
					except:
						logging.debug("\tCouldn't load control setting: '{}' = {}, maybe it's disabled by other setting like 'Auto' modes? Won't retry :(".format(c.display_name, v))
		except:
			logging.exception("Something went wrong while loading UVCcam settings from '{}'.".format(file_name))
			return False
		return True

	@staticmethod
	def new_from_settings(file_name):
		"""
		Creates a new UvcCapture instance from a camera settings file.
		:param file_name: Path to the file to load the settings from
		:return: A new UvcCapture instance if everything went well; None if settings couldn't be loaded
		"""
		try:
			logging.debug("Creating new UvcCapture instance from settings file '{}'".format(file_name))
			with open(file_name, 'r') as f:
				cam_name, vend_id, prod_id, cam_serial_num, _ = UvcCapture._parse_first_line_of_settings(f.readline())  # Read camera info that the settings were created for

			cap = UvcCapture(cam_name, vend_id, prod_id, cam_serial_num)
			if cap is not None:
				cap.load_settings(file_name)
		except:
			logging.exception("Something went wrong while loading UVCcam settings from '{}'.".format(file_name))
			return None
		return cap

	def init_calibration_params(self, calibration_file):
		"""
		Loads calibration parameters (camera matrix, distortion coefficients, etc.) from a given file.
		:param calibration_file: str containing the path of the camera calibration file (*.npz)
		"""
		ensure_cv2_exists()  # Make sure we have imported cv2, otherwise raise an error
		self.calib_file = calibration_file

		with np.load(calibration_file) as data:  # Load required params from *.npz file
			self.dist_coefs = data["dist_coefs"][0]
			self.calib_frame_size = tuple(data["cam_frame_size"])
			scaling_factor = np.array(self.frame_size, dtype=float)/np.array(self.calib_frame_size, dtype=float)  # Might need to rescale camera_matrix
			self.orig_camera_matrix = data["camera_matrix"]
			self.camera_matrix = self.orig_camera_matrix * np.array([[scaling_factor[0], 1, scaling_factor[0]],[1, scaling_factor[1], scaling_factor[1]],[1, 1, 1]])
			self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coefs, self.frame_size, 1)
			self.calib_at = data["calib_at"]

	def init_undistort_maps(self, calibration_file):
		"""
		Initializes undistortion maps for the current camera frame size and provided calibration settings.
		These maps can be resued for all camera frames, and caching them has a huge performance benefit
		(~3ms to undistort a frame vs. ~50ms if each undistortion generates the maps before undistorting)
		:param calibration_file: str containing the path of the camera calibration file (*.npz)
		"""
		ensure_cv2_exists()  # Make sure we have imported cv2, otherwise raise an error

		self.init_calibration_params(calibration_file)  # Load required params (camera_matrix, dist_coefs, etc.) from *.npz file
		self.undist_maps = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coefs, None, self.new_camera_matrix, self.frame_size, cv2.CV_16SC2)  # Initialize maps using cv2's method
		self.do_undistort = True
		logging.debug("\tSuccessfully initialized undistortion maps from '{}'.".format(calibration_file))

	@staticmethod
	def undistort_img(img, undist_maps, interp=None):
		"""
		Undistorts a given frame using the given undistortion maps (wraps around cv2's remap method)
		:param img: np.array containing the camera frame that wants to be undistorted
		:param undist_maps: undistortion maps initialized through init_undistort_maps
		:param interp: Interpolation method to use for undistortion. Should be a constant from cv2.INTER_*
		:return: Nothing is returned. Instead, the provided camera frame img is overwritten with the undistorted image
		"""
		ensure_cv2_exists()  # Make sure we have imported cv2, otherwise raise an error
		if interp is None:  # Can't initialize interp directly in the method definition in case cv2 was not imported
			interp = cv2.INTER_LINEAR  # Default to bilinear interpolation (fast, good results)

		if undist_maps is not None:  # Prevent crashing if maps haven't been initialized by only undistorting if not None
			np.copyto(img, cv2.remap(img, undist_maps[0], undist_maps[1], interp))  # Use cv2's remap method to undistort

	def get_frame(self):
		"""
		Redifened get_frame() method to undistort the frame after it's captured if do_undistort flag is set.
		Otherwise (if do_undistort is False), it behaves exactly like uvc.Capture's get_frame().
		"""
		frame = super(UvcCapture, self).get_frame()  # Obtain the frame through super() class

		if self.do_undistort:  # Undistort the image if undistortion maps have been provided and do_undistort flag is set
			UvcCapture.undistort_img(frame.bgr, self.undist_maps)

		return frame

	def sorted_available_modes(self):
		"""
		Sorts available modes by decreasing frame width, height and fps (in that order, in case there's a tie)
		:return: A sorted np.array of available camera modes supported by the current camera
		"""
		return np.array(sorted(self.avaible_modes, reverse=True))

	def select_best_frame_mode(self, min_fps=1, min_width=1, min_height=1):
		"""
		Selects the best camera mode available which meets the criteria specified by the function parameters.
		:param min_fps: Minimum FPS that a camera mode needs to provide to be elegible
		:param min_width: Minimum frame width that a camera mode needs to provide to be elegible
		:param min_height: Minimum frame height that a camera mode needs to provide to be elegible
		"""
		avail_modes = self.sorted_available_modes()  # Rank all available modes (best one first, see sorted_available_modes
		if len(avail_modes) == 0: return
		condition = (avail_modes[:,0] >= min_width) & (avail_modes[:,1] >= min_height) & (avail_modes[:,2] >= min_fps)
		try:
			self.frame_mode = tuple(avail_modes[np.argwhere(condition).ravel()[0]])  # Choose biggest size (because avail_modes is sorted) meeting criteria 'condition'
		except:  # In case no modes meet the criteria, choose the biggest size & highest frame rate
			self.frame_mode = tuple(avail_modes[0])
			logging.debug("No modes met the criteria of frame rate >= {}fps, width >= {}px, height >= {}px. Selected mode with biggest size and highest frame rate: {}".format(min_fps, min_width, min_height, self.frame_mode))
