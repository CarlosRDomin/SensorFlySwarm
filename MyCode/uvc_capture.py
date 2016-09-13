"""
	Wrapper class around uvc.Capture to add custom methods to the camera capture class, like saving and loading settings!
"""


import uvc
import numpy as np
import logging


class UvcCapture(uvc.Capture):

	SETTINGS_SEPARATOR = '\t'  # String used to separate setting name from setting value (don't use ',' as some setting names might contain commas)
	logging.basicConfig(level=logging.DEBUG)

	def __new__(cls, cam_name, *more):
		"""
		Class creator which prevents an UvcCapture object from being created if we can't connect to the specified camera.
		It also finds the UID of the camera for us, so we can choose the camera more easily through cam name or index.
		:param cam_name: Either the desired camera name or the index in the list of available devices to connect to
		:param more: Any other params that would be passed on to the uvc.Capture creator
		:return: A new object connected to the specified camera, or None if there was an error (eg: camera not found).
		"""
		cam_uid = ""
		if isinstance(cam_name, str):  # str -> Find by camera name
			for d in uvc.device_list():  # From the available devices list, find the one we want
				if d['name'] == cam_name:
					cam_uid = d['uid']  # And figure out its UID
					break
		elif isinstance(cam_name, int):  # int -> Find by camera index
			if cam_name < len(uvc.device_list()):
				cam_uid = uvc.device_list()[cam_name]['uid']  # If index is within bounds, figure out camera's UID

		if cam_uid == "":  # This only happens if we couldn't find the camera from the information we were provided
			logging.warning("Couldn't find camera '{}' in the list of available UVC devices, aborting creation of {} object.".format(cam_name, UvcCapture))
			return None  # Don't allow the creation of the object -> Return None

		obj = super(UvcCapture, cls).__new__(cls, more)  # Create a new object by calling the parent constructor
		try:
			super(UvcCapture, obj).__init__(cam_uid)  # Initialize the object using the camera UID (uvc.Capture opens the device)
			return obj  # Finally return the newly created object if everything went fine
		except:
			logging.error("Something went wrong opening camera '{}', is it already open by another process? Aborting creation of {} object.".format(cam_name, UvcCapture))
			return None  # In case of an error (eg: the camera is already in use) return None

	def __init__(self, *more):  # Initialization of underlying uvc.Capture object was already done in __new__
		pass  # We still need to explicitly add this method or else the super.__init__ method will be called (and it'll try to open a camera with uid=cam_name, so it'll obviously fail)

	def save_settings(self, file_name):
		"""
		Saves current settings to a file.
		:param file_name: Name of the file to use when saving the settings
		:return: True if everything went well; False if settings couldn't be saved
		"""
		try:
			logging.debug("Saving current UVC camera settings to file '{}'".format(file_name))
			with open(file_name, 'w') as f:
				f.write("{}\n".format(self.name))
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
				cam_name = f.readline().rstrip('\r\n')  # Read camera name that the settings were created for
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
				for line in f:
					name, value = line.split(self.SETTINGS_SEPARATOR)
					for c in self.controls:
						if c.display_name == name:
							c.value = int(value)
							logging.debug("\tLoaded control setting: '{}' = {}".format(c.display_name, c.value))
							break
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
				cam_name = f.readline().rstrip('\r\n')

			cap = UvcCapture(cam_name)
			if cap is not None:
				cap.load_settings(file_name)
		except:
			logging.exception("Something went wrong while loading UVCcam settings from '{}'.".format(file_name))
			return None
		return cap

	def sorted_available_modes(self):
		"""
		Sorts available modes by decreasing frame width, height and fps (in that order, in case there's a tie)
		:return: The sorted array (np.array) of available camera modes to choose from
		"""
		return np.array(sorted(self.avaible_modes, reverse=True))

	def select_best_frame_mode(self, min_fps=1, min_width=1, min_height=1):
		"""
		Selects the best camera mode available which meets the criteria specified by the function parameters.
		:param min_fps: Minimum FPS that a camera mode needs to provide to be elegible
		:param min_width: Minimum frame width that a camera mode needs to provide to be elegible
		:param min_height: Minimum frame height that a camera mode needs to provide to be elegible
		"""
		avail_modes = self.sorted_available_modes()  # Rank all available modes (best one first, see sorted_available_modes())
		condition = (avail_modes[:,0] >= min_width) & (avail_modes[:,1] >= min_height) & (avail_modes[:,2] >= min_fps)
		try:
			self.frame_mode = tuple(avail_modes[np.argwhere(condition).ravel()[0]])  # Choose biggest size (because avail_modes is sorted) meeting criteria 'condition'
		except:  # In case no modes meet the criteria, choose the biggest size & highest frame rate
			self.frame_mode = tuple(avail_modes[0])
			logging.debug("No modes met the criteria of frame rate >= {}fps, width >= {}px, height >= {}px. Selected mode with biggest size and highest frame rate: {}".format(min_fps, min_width, min_height, self.frame_mode))
