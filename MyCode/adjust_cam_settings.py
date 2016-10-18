"""
	Auxiliary tool created to set up all computer-vision related system settings: camera controls (exposure time,
	white balance, etc.), color thresholds (HSV boundaries to detect the ping pong ball glued on top of each drone),
	and blob detector parameters (after HSV thresholding, the binary mask has to be processed to detect which blob,
	if there were multiple, corresponds to the drone). The tool also allows to save all these values to a file.
"""

import numpy as np
import cv2
import re
import sys
from os import path
from datetime import datetime
from uvc_capture import UvcCapture
from full_control_with_cam import Spotter, FakeWorkerDrone, FakeVideoCapture
from calibrate_cam_params import CALIB_FOLDER
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot, QSize, QObject, QEvent
from PyQt5.QtGui import QImage, QPixmap, QResizeEvent, QMouseEvent
from PyQt5.QtWidgets import QApplication, QSpinBox, QSpacerItem, QSizePolicy, QCheckBox, QAbstractSpinBox, QFileDialog, QMessageBox
from PyQt5.uic import loadUi
import logging
logging.disable(logging.DEBUG)


class CamDeviceListRefresher(QThread):
	"""
	This class implements a background task that constantly looks for connected cameras,
	keeping an updated list of available devices in dev_list
	"""

	sig_new_dev = pyqtSignal()  # Signal that's emitted every time a new device is connected or an old one disconnected
	REFRESH_RATE = 2  # Refresh interval (in seconds)

	def __init__(self):
		QThread.__init__(self)
		self.done = False
		self.dev_list = []

	def __del__(self):
		self.wait()

	@pyqtSlot()
	def stop_thread(self):  # This method is used to externally request this thread to terminate
		self.done = True    # Flag to indicate the outer while loop in run_experiment() to finish thread execution
		if not self.wait(3*1000*self.REFRESH_RATE):  # Should never happen, but just in case ;)
			logging.warning("Uh-ooohh, CamDeviceListRefresher didn't terminate even after waiting for 5sec. Force quitting :S")
			self.terminate()
		else:
			logging.info("CamDeviceListRefresher exited cleanly :)")

	def run(self):
		while not self.done:
			logging.info("Scanning for new devices...")
			dev_list = UvcCapture.device_list()  # Find available devices
			if self.dev_list != dev_list:        # If found a new device or an old device was disconnected
				self.dev_list = dev_list         # Update the list of devices
				self.sig_new_dev.emit()          # And fire a signal
				print "Compatible devices found:\n\t" + str(self.dev_list)
			self.sleep(self.REFRESH_RATE)


class CamFrameGrabberThread(QThread):
	"""
	This class implements a background task that constantly grabs new frames from the selected camera
	"""

	DEFAULT_CAM_NAME = "None"  # "USB 2.0 Camera"
	DEFAULT_CAM_VEND_ID = -1  # 1443
	DEFAULT_CAM_PROD_ID = -1  # 37424
	DEFAULT_FPS = 60
	FPS_ESTIMATION_ALPHA = 0.98
	FRAME_SIZE_STR_SPLITTER = " x "

	sig_update_cam_ui = pyqtSignal()        # Signal that's emitted whenever the UI associated to the camera controls needs to be refreshed (eg: new frame size selected -> need to resize window, error grabbing camera frame -> disconnect camera...)
	sig_new_dev_selected = pyqtSignal(str)  # Signal that's emitted when a device is selected (could be the device that was already selected). It's emitted either from the UI when the user changes the selected item in drpInput, or from the code when sig_new_dev is emitted (as a new device is connected or an old one disconnected)
	sig_new_img = pyqtSignal()              # Signal that's emitted every time a new camera frame becomes available
	sig_error = pyqtSignal(str)             # Signal that's emitted every time an error occurred

	def __init__(self):
		QThread.__init__(self)
		self.done = False
		self.dev_selected_name = self.DEFAULT_CAM_NAME
		self.dev_selected_vend_id = self.DEFAULT_CAM_VEND_ID
		self.dev_selected_prod_id = self.DEFAULT_CAM_PROD_ID
		self.cap = None
		self.qPix = QPixmap(1, 1)
		self.cv = Spotter()
		self.cv.workers.append(FakeWorkerDrone())  # Add a fake worker (we just need the cv algorithm to look for 1 ping pong ball)
		self.cv.video_capture = FakeVideoCapture()
		self.cv.world_to_camera_transf = np.hstack((np.eye(3), np.zeros((3, 1))))
		self.cv.load_color_thresh_settings()
		self.cv.load_blob_detector_settings()
		self.cv_do_color = True
		self.cv_do_blob = True
		self.cv_evaluate_px_at = np.array([0, 0])
		self.cv_evaluate_px_value = np.array([0, 0, 0], dtype=np.uint8)
		self.index = 0
		self.actual_fps = 0

	def __del__(self):
		self.wait()

	@pyqtSlot()
	def stop_thread(self):  # This method is used to externally request this thread to terminate
		self.done = True    # Flag to indicate the outer while loop in run_experiment() to finish thread execution
		self.change_selected_device("EXIT! :P")  # Choose a void device, to force the inner while loop in run_experiment() to exit after grabbing at most one frame
		if self.wait(5000) is not True or self.cap is not None:  # Worst-case scenario: force quit if self.cap is still not None after 5sec
			logging.warning("Uh-ooohh, CamFrameGrabberThread didn't terminate even after waiting for 5sec. Force quitting :S")
			self.cap = None
			self.terminate()
		else:
			logging.info("CamFrameGrabberThread exited cleanly :)")

	@pyqtSlot(str)
	def change_selected_device(self, dev_selected):
		if self.dev_selected_name != dev_selected: logging.info("New device selected: {}".format(dev_selected))
		self.dev_selected_name, self.dev_selected_vend_id, self.dev_selected_prod_id = cam_str_to_info(dev_selected)

	@pyqtSlot(str, bool)
	def change_cam_frame_size(self, new_frame_size, emit_signal):
		if self.cap is not None and window.drpSize.count() > 0:  # Sanity check (in case capture device was just closed or we're clearing drpSize to re-add available sizes)
			logging.info("Changing frame size to {}".format(new_frame_size))
			self.cap.do_undistort = False  # Pause undistortion while frame_size is changing to avoid problems in the frame capture thread
			self.cap.frame_size = tuple([int(x) for x in new_frame_size.split(self.FRAME_SIZE_STR_SPLITTER)])
			self.cap.init_undistort_maps(self.cap.calib_file)
			if emit_signal:  # If the user was the one who clicked on this combobox item (as opposed to me, manually from the code), update cam UI
				self.sig_update_cam_ui.emit()  # drpFPS might need to be updated (different frame sizes might have different fps available)

	@pyqtSlot(str)
	def change_cam_frame_rate(self, new_frame_rate):
		if self.cap is not None and window.drpFPS.count() > 0:  # Sanity check (in case capture device was just closed or we're clearing drpSize to re-add available sizes)
			logging.info("Changing frame rate to {}".format(new_frame_rate))
			self.cap.frame_rate = int("".join(x for x in new_frame_rate if x.isdigit()))  # Keep only the numbers (= remove the " fps" part, if exists)

	@pyqtSlot(str, int, bool)
	def change_cam_control_setting(self, ctrl_name, new_value, is_bool):
		"""
		Process a request (from the UI) to change a particular camera setting.
		:param ctrl_name: Particular camera setting that wants to be modified (eg: exposure time, white balance...)
		:param new_value: New value desired for the camera setting
		:param is_bool:  True if the UI control is a CheckBox, False if it's a SpinBox (ie: TextBox with up&down arrows)
		"""
		if self.cap is not None:  # Sanity check (in case capture device was just closed)
			for c in self.cap.controls:
				if c.display_name == ctrl_name:
					try:
						if is_bool:
							c.set_value(c.min_val if (new_value == Qt.Unchecked) else c.max_val)
						else:
							c.set_value(new_value)
							logging.info("'{}' changed to {}".format(c.display_name, c.value))
					except:
						self.sig_error.emit("Unable to change '{}' property to '{}'! Make sure the value is valid (within bounds, not disabled by other setting like 'Auto' modes, etc).".format(ctrl_name, new_value))
					return

	@pyqtSlot(bool)
	def set_cv_do_color(self, checked):
		self.cv_do_color = checked
		window.grpBlob.setEnabled(checked)
		self.set_cv_do_blob(False if not checked else window.grpBlob.isChecked())

	@pyqtSlot(bool)
	def set_cv_do_blob(self, checked):
		self.cv_do_blob = checked

	@pyqtSlot(int, str, bool)
	def change_HSV_setting(self, new_value, letter, is_max):
		"""
		Updates the corresponding field (H,S or V variable, min or max) of the HSV color thresholding filter.
		:param new_value: New value to set the corresponding threshold to
		:param letter: either 'H', 'S', or 'V' (case insensitive), depending on which threshold should be modified
		:param is_max: True to change the upper threshold; False to change the lower one
		"""
		letter = letter.lower()  # To avoid problems, convert the letter to lowercase and only compare to lowercase
		index = 0 if letter == 'h' else 1 if letter == 's' else 2  # Convert H, S, V to 0, 1, 2

		# Now update the right variable based on is_max
		if is_max:
			self.cv.cv_HSV_thresh_max[index] = new_value
		else:
			self.cv.cv_HSV_thresh_min[index] = new_value

	@pyqtSlot(int, str, bool)
	def change_blob_setting(self, new_value, letter, is_max):
		"""
		Updates the corresponding setting (area, circularity, convexity or inertia, min or max) of the blob detector.
		:param new_value: If int: New value to set the corresponding threshold to; if bool: activate/deactivate filter
		:param letter: either 'A', 'C', 'V', or 'I' (case insensitive), depending on which setting should be modified
		:param is_max: True to change the upper threshold; False to change the lower one [ignored if new_value is bool]
		"""
		letter = letter.lower()  # To avoid problems, convert the letter to lowercase and only compare to lowercase
		param = "Area" if letter == 'a' else "Circularity" if letter == 'c' else "Convexity" if letter == 'v' else "Inertia"

		if type(new_value) is bool:
			setattr(self.cv.cv_blob_detect_params, "filterBy{}".format(param), new_value)
		else:
			setattr(self.cv.cv_blob_detect_params, "{}{}{}".format("max" if is_max else "min", param, "Ratio" if letter == 'i' else ""), new_value)

	def run(self):
		while not self.done:
			self.sleep(1)  # Outer while loop just waits until a device is selected

			self.cap = UvcCapture(self.dev_selected_name, self.dev_selected_vend_id, self.dev_selected_prod_id)
			if self.cap is None:  # If we didn't find the desired cam, don't continue
				self.qPix = QPixmap(1, 1)  # Default to a black pixel
				self.cv_evaluate_px_value = np.array([0, 0, 0], dtype=np.uint8)
				logging.info("No compatible cameras found or chosen camera name not available :(")
				self.sig_update_cam_ui.emit()  # Make sure camera-related UI is updated (eg: don't show exposure time control if no camera is selected...)
				continue

			# If we made it here, we successfully connected to the selected camera. Print camera info and select default values
			for c in self.cap.controls:
				logging.info("\t{} = {} {} (def:{}, min:{}, max:{})".format(c.display_name, str(c.value), str(c.unit), str(c.def_val), str(c.min_val), str(c.max_val)))
			self.cap.select_best_frame_mode(self.DEFAULT_FPS)
			logging.info(self.cap.name + " has the following available modes:\n\t" + str([tuple(x) for x in self.cap.sorted_available_modes()]) + "\nSelected mode: " + str(self.cap.frame_mode))
			self.cap.print_info()
			logging.debug("LOADING SETTINGS returned {}".format(self.cap.load_settings(self.cv.CAMERA_SETTINGS_FILE)))
			self.sig_update_cam_ui.emit()  # Update camera-related UI (eg: add controls for exposure time, white balance, etc.)

			img = np.zeros(self.cap.frame_size)  # Resize qPix to fit new image size
			self.qPix = QPixmap.fromImage(QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888))

			self.actual_fps = self.cap.frame_rate
			tt = datetime.now()  # Initialize tt (used to estimate actual frame rate) to prevent an error on the first loop iteration
			while self.cap.name == self.dev_selected_name and self.cap.vend_id == self.dev_selected_vend_id and self.cap.prod_id == self.dev_selected_prod_id:  # Run an inner while loop to capture frames as long as the selected device is kept constant
				ok = False
				t = datetime.now()
				for a in range(3):
					try:
						frame = self.cap.get_frame_robust()
						self.index = frame.index
						ok = True
						break
					except Exception as e:
						logging.error('DEBUG - Could not get Frame. Error: "{}". Tried {} time{}.'.format(e.message, a+1, "s" if a > 0 else ""))
				if not ok:
					self.sig_error.emit("Couldn't get camera frame after 3 tries, reconnecting...")
					break  # Exit inner loop (force an iteration on the outer loop) after 3 consecutive failed attempts to grab a frame

				# We successfully grabbed a frame, let's store the value of the pixel cv_evaluate_px_at for debugging purposes
				try:
					self.cv_evaluate_px_value = np.array(frame.bgr[self.cv_evaluate_px_at[1], self.cv_evaluate_px_at[0]])  # Create a new np.array so that the value at the pixel is not cached, but copied (ie: if frame.bgr is modified, cv_evaluate_px_value is not). Also note that images should be indexed [y,x]!
				except:
					self.cv_evaluate_px_at = np.array([0, 0])

				# Now we can run the CF detection algorithm on it (if grpColor is checked) and modify frame.bgr if we want
				t2 = datetime.now()
				if self.cv_do_color:
					cf_curr_pos = self.cv.detect_cf_in_camera(frame.bgr, self.cv_do_blob)[0]  # Run CF detection (and blob detection if checked)

					# Now overlay the mask over the camera frame
					t3 = datetime.now()
					if cf_curr_pos is not None and self.cv_do_blob:
						cv2.circle(self.cv.cv_cam_frame, tuple(cf_curr_pos[0:2].astype(int)), int(cf_curr_pos[2]+5), [0, 255, 0], -1)
					# np.putmask(self.cv.cv_cam_frame[:,:,0], self.cv.cv_filtered_HSV_mask, 255)
					# np.putmask(self.cv.cv_cam_frame[:,:,1], self.cv.cv_filtered_HSV_mask, 0)
					# np.putmask(self.cv.cv_cam_frame[:,:,2], self.cv.cv_filtered_HSV_mask, 255)
					mask = cv2.cvtColor(self.cv.cv_filtered_HSV_mask, cv2.COLOR_GRAY2BGR).astype(bool)
					np.putmask(self.cv.cv_cam_frame, mask, np.array([255, 0, 255], dtype=np.uint8))
				else:
					t3 = datetime.now()  # For consistency, save current time as t3 (even if we didn't do anything

				# And finally convert the resulting image to qPix and emit the appropriate signal so the UI can refresh it
				t4 = datetime.now()
				img = cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2RGB)
				self.qPix.convertFromImage(QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888))
				self.sig_new_img.emit()

				t5 = datetime.now()
				self.actual_fps = self.FPS_ESTIMATION_ALPHA*self.actual_fps + (1-self.FPS_ESTIMATION_ALPHA)/(t5-tt).total_seconds()
				logging.debug("At t={} picture #{:03d} was taken; deltaTtotal={:6.2f}ms [{:6.2f}ms] (capture_frame->{:6.2f}ms + detect->{:6.2f}ms + blob->{:6.2f}ms + putmask->{:6.2f}ms + toQPixmap->{:6.2f}ms) -> Estimated FPS: {:.2f}".format(t2, frame.index, (t5-tt).total_seconds()*1000, (t5-t).total_seconds()*1000, (t2-t).total_seconds()*1000, (self.cv.t_events[-2]-t2).total_seconds()*1000 if self.cv_do_color else 0, (self.cv.t_events[-1]-self.cv.t_events[-2]).total_seconds()*1000 if self.cv_do_blob else 0, (t4-t3).total_seconds()*1000, (t5-t4).total_seconds()*1000, self.actual_fps))
				tt = datetime.now()

			self.cap = None  # Set cap to None to automatically release & disconnect from the camera


class FilterIgnoreScroll(QObject):
	"""
	This class implements an eventFilter programmed to ignore Scroll events.
	This is very useful to prevent ComboBoxes and SpinBoxes from changing their value when we scroll over them.
	"""

	sig_scroll = pyqtSignal()  # Signal that's emitted every time an ignored scroll event is intercepted

	def eventFilter(self,  obj,  event):
		if event.type() == QEvent.Wheel:  # Detect a scroll event
			self.sig_scroll.emit()  # In case someone is interested in catching the ignored scroll event, emit a signal
			event.ignore()  # Ignore unwanted event
			return True
		return super(FilterIgnoreScroll, self).eventFilter(obj, event)  # Not a scroll event, pass it on to super


class FilterNotifyWindowCloses(QObject):
	"""
	This class implements an eventFilter which emits a signal when the window is about to get closed.
	"""

	sig_close = pyqtSignal()  # Signal that's emitted when a window is about to get closed.

	def __init__(self, parent):
		super(FilterNotifyWindowCloses, self).__init__(parent)
		self.sig_close.connect(threadFrameGrabber.stop_thread)  # Stop both background threads before the window closes
		self.sig_close.connect(threadDevLister.stop_thread)

	def eventFilter(self,  obj,  event):
		if event.type() == QEvent.Close:  # Detect the Window Close event
			self.sig_close.emit()
			return True
		return super(FilterNotifyWindowCloses, self).eventFilter(obj, event)  # Not a Close event, pass it on to super


def make_control_ignore_scroll(ctrl):
	"""
	Makes sure the specified control ignores scroll events (ie: scroll parent container instead of modifying its value).
	:param ctrl: Control (element) that should ignore scroll events
	"""
	ctrl.installEventFilter(FilterIgnoreScroll(ctrl))
	try:
		ctrl.setFocusPolicy(Qt.StrongFocus)  # Prevent the control from gaining focus when we scroll over them
	except:  # Some controls don't have this method, ignore the error in that case
		pass

def make_all_children_ignore_scroll(layout):
	"""
	Recursive function to make all elements (controls) attached to a given layout ignore scroll events.
	:param layout: Control layout whose children controls are to ignore scroll events
	"""
	if layout is None: return  # Nothing to install (avoid exception calling layout.children())

	for item in layout.children():  # For every child
		if type(item) is not FilterIgnoreScroll:  # Avoid infinite recursion by not installing a filter on a filter itself!!
			make_all_children_ignore_scroll(item)  # Recursively install the filter on its children (if any)
			make_control_ignore_scroll(item)  # Then install the "ignore scroll" filter on the item itself

def remove_all_layout_children(layout):
	"""
	Recursive function to remove all elements (controls) attached to a given layout.
	:param layout: Control layout whose children controls are to be deleted.
	"""
	if layout is None: return  # Nothing to remove! - Recursive condition end

	while layout.count():
		item = layout.takeAt(0)  # Get first item and remove it from the layout
		widget = item.widget()
		if widget is not None:  # Then, delete the actual widget
			widget.deleteLater()
		else:  # Unless it is a nested layout, in which case recursively delete all its children
			remove_all_layout_children(item.layout())

def cam_info_to_str(cam_name, cam_vend_id, cam_prod_id):
	"""
	Converts camera info (name, vendor id and product id) to a human-friendly string.
	:param cam_name: Camera name, as provided by the uvc library
	:param cam_vend_id: Camera vendor id, as provided by the uvc library
	:param cam_prod_id: Camera product id, as provided by the uvc library
	:return: String containing the name, vendor id and product id of the camera in a human-friendly format.
	"""
	return "{} ({}, {})".format(cam_name, cam_vend_id, cam_prod_id)

def cam_str_to_info(cam_str):
	"""
	Parses the camera information from a string generated by cam_info_to_str().
	:param cam_str: String containing camera name, vendor id and product id, generated using cam_info_to_str()
	:return: A tuple containing the camera name, vendor id and product id.
	"""
	# info = re.search('^(.+) \(([---0-9]+)[^---0-9]+([---0-9]+)\)$', str(cam_str))  # Use this one to accept negative vend and prod ids (<0 means accept any that matches the cam name)
	info = re.search('^(.+) \(([0-9]+)[^0-9]+([0-9]+)\)$', cam_str)
	if info is None:
		return "None", -1, -1

	return str(info.group(1)), int(info.group(2)), int(info.group(3))

@pyqtSlot(str)
def show_error_message(str_msg):
	logging.error("Showing error message: '{}'".format(str_msg))
	QMessageBox.critical(window, "ERROR! :(", str_msg)

@pyqtSlot(QResizeEvent)
def on_resize(event):
	window.lblCamImg.setPixmap(threadFrameGrabber.qPix.scaled(window.lblCamImg.size(), Qt.KeepAspectRatio))  # Resize pixmap inside label to fit new size

@pyqtSlot(QMouseEvent)
def on_lblCamImg_mouseEvent(event):
	global last_mouse_event  # Use a global variable to store information about last event. Used for updating pixel value on new frame UI repaint

	event_was_none = (event is None)
	if event_was_none:
		event = last_mouse_event
		if last_mouse_event is None:
			return

	try:
		pixmap_w = window.lblCamImg.pixmap().width()  # At the very beginning, pixmap() might still be None if we haven't painted qPixmap yet -> Surround it by try/except
		pixmap_h = window.lblCamImg.pixmap().height()
		if threadFrameGrabber.cap is not None:  # Prevent exceptions when no camera is selected
			rescale_factor = threadFrameGrabber.cap.frame_size[0]/pixmap_w  # Sometimes, frame_size might be None (eg: right after turning on camera or changing resolution) -> Surround it by try/except
		else:
			rescale_factor = 0  # Force cv_evaluate_px_at to be point (0, 0) if no camera is selected
	except:
		return  # In that case, just ignore the event

	# Pixmap might be resized: one of the dimensions might not fill the whole lblCamImg container -> Change frame of reference so (0,0) corresponds to pixmap's top left corner
	x = event.x() - (window.lblCamImg.width()-pixmap_w)/2
	y = event.y() - (window.lblCamImg.height()-pixmap_h)/2

	# If out of bounds, don't update cv_evaluate_px_at (would cause an error to evaluate out of bounds)
	if x < 0 or y < 0 or x >= pixmap_w or y >= pixmap_h:
		# threadFrameGrabber.cv_evaluate_px_at = np.array([0, 0])
		# window.statusbar.showMessage("")
		return

	# If "in bounds", calculate the corresponding point coordinates in the original resolution of the image
	x_rescaled = x*rescale_factor
	y_rescaled = y*rescale_factor
	threadFrameGrabber.cv_evaluate_px_at = np.array([int(x_rescaled), int(y_rescaled)])

	# c = window.lblCamImg.pixmap().toImage().pixel(x, y)
	# c_rgb = np.array(QColor(c).getRgb()[0:3], dtype=np.uint8)
	c_rgb = threadFrameGrabber.cv_evaluate_px_value[::-1]  # Remember: cv_evaluate_px_value is in BGR -> Flip to get RGB
	c_hsv = cv2.cvtColor(np.array([[c_rgb]], dtype=np.uint8), cv2.COLOR_RGB2HSV).ravel()
	str_msg = "Color value at pixel [x:{:4}, y:{:4}] (point [x:{:4}, y:{:4}]) is [H:{HSV[0]:4}, S:{HSV[1]:4}, V:{HSV[2]:4}] = [R:{RGB[0]:4}, G:{RGB[1]:4}, B:{RGB[2]:4}]".format(x_rescaled, y_rescaled, x, y, HSV=c_hsv, RGB=c_rgb)
	window.statusbar.showMessage(str_msg)
	if not event_was_none and event.type() == event.MouseButtonPress:  # Print to console debug info on click
		print "\t\t{}".format(str_msg)
	last_mouse_event = QMouseEvent(event)  # Create a copy of current event so its info is not modified by other functions

@pyqtSlot()
def repaint_cam_img():
	window.lblCamImg.setPixmap(threadFrameGrabber.qPix.scaled(window.lblCamImg.size(), Qt.KeepAspectRatio))
	# window.lblCamImg.update()
	# window.lblCamImg.repaint()
	window.setWindowTitle("Frame {} - FPS = {:.3f}".format(threadFrameGrabber.index, threadFrameGrabber.actual_fps))
	on_lblCamImg_mouseEvent(None)

@pyqtSlot()
def refresh_cam_device_list():
	old_dev_selected = window.drpInput.currentText()
	window.drpInput.clear()
	window.drpInput.addItem("None")  # Add a 'None' item so user can turn off camera
	for dev in threadDevLister.dev_list:
		txt = cam_info_to_str(dev['name'], dev['idVendor'], dev['idProduct'])
		window.drpInput.addItem(txt)
		if txt == old_dev_selected:
			window.drpInput.setCurrentIndex(window.drpInput.count()-1)
	threadFrameGrabber.sig_new_dev_selected.emit(window.drpInput.currentText())

@pyqtSlot()
def update_cam_ui():
	"""
	Refreshes all UI/controls associated with the selected device
	(comboboxes for frame size/rate, controls for exposure time, white balance, etc.).
	If no camera is selected, all those controls are removed. Otherwise, they're initialized based on selected settings.
	"""
	window.drpSize.blockSignals(True)  # First, block frame size and FPS signals, so we don't receive changeIndex events while adding/removing items
	window.drpFPS.blockSignals(True)
	window.drpSize.clear()  # Then, clear frame size and FPS comboboxes, as well as remove all items from the settings tab
	window.drpFPS.clear()
	window.chk_save_cam.setEnabled(threadFrameGrabber.cap is not None)  # Only allow saving cam settings if a camera is selected
	remove_all_layout_children(window.layoutGrpCam)
	if threadFrameGrabber.cap is None:  # User hasn't selected a device -> Add the default options ("N/A") to comboboxes
		window.drpSize.addItem("N/A")   # No need to reenable signals, since it would be useless to get notified that user selected option "N/A"
		window.drpFPS.addItem("N/A")
		window.layoutGrpCam.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding), 0, 0, 1, 1)  # Adding this spacer forces grpCam to resize, otherwise it would look empty but a vertical scrollbar would show up
		return  # Only continue if a valid camera was selected!

	# Resize (and center) window according to selected camera's frame size
	frame_qsize = QSize(threadFrameGrabber.cap.frame_size[0], threadFrameGrabber.cap.frame_size[1])
	new_win_size = window.size() - window.lblCamImg.size() + frame_qsize
	window.setMaximumSize(new_win_size)
	window.setMinimumSize(new_win_size - frame_qsize + frame_qsize/2)
	window.resize(new_win_size)
	window.move((app.desktop().screenGeometry().width()-window.width())/2, (app.desktop().screenGeometry().height()-window.height())/2)
	logging.info("Changed UI pic size to {}".format(threadFrameGrabber.cap.frame_size))

	# Populate frame size combobox (drpSize) and select the appropiate item (based on threadFrameGrabber.cap.frame_size)
	for s in threadFrameGrabber.cap.frame_sizes:
		window.drpSize.addItem("{}{}{}".format(s[0], threadFrameGrabber.FRAME_SIZE_STR_SPLITTER, s[1]))
		if s == threadFrameGrabber.cap.frame_size:
			window.drpSize.setCurrentIndex(window.drpSize.count()-1)
	if window.drpSize.currentIndex() < 0:
		window.drpSize.setCurrentIndex(0)  # If no matching value found, choose the first one
		threadFrameGrabber.change_cam_frame_size(window.drpSize.currentText(), False)
	window.drpSize.blockSignals(False)  # Reenable signals for drpSize, as no more items will be added/removed nor currentIndex will be changed

	# Populate frame rate combobox (drpFPS) and select the appropiate item (based on threadFrameGrabber.cap.frame_rate)
	for f in threadFrameGrabber.cap.frame_rates:
		window.drpFPS.addItem("{} fps".format(f))
		if f == threadFrameGrabber.cap.frame_rate:
			window.drpFPS.setCurrentIndex(window.drpFPS.count()-1)
	window.drpFPS.blockSignals(False)  # Reenable signals for drpFPS too, as no more items will be added/removed nor currentIndex will be changed
	if window.drpFPS.currentIndex() < 0: window.drpSize.setCurrentIndex(0)  # If no matching value found, choose the first one

	# Populate layoutGrpCam based on the device's available controls/settings (exposure time, white balance, etc)
	i = 0
	for c in threadFrameGrabber.cap.controls:
		if c.max_val - c.min_val == 1:  # Boolean -> CheckBox
			ctrl = QCheckBox(window.grpCam)
			ctrl.setObjectName("chk_cam_{}".format(i+1))
			ctrl.setText(c.display_name)
			ctrl.setChecked(c.value == c.max_val)
			ctrl.stateChanged.connect(lambda v, n=c.display_name: threadFrameGrabber.change_cam_control_setting(n, v, True))
		else:  # Non-boolean -> SpinBox (TextBox for numbers with up&down arrows)
			ctrl = QSpinBox(window.grpCam)
			ctrl.setObjectName("spin_cam_{}".format(i+1))
			ctrl.setAccelerated(True)
			ctrl.setCorrectionMode(QAbstractSpinBox.CorrectToNearestValue)
			ctrl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
			make_control_ignore_scroll(ctrl)

			ctrl.setPrefix("{}: ".format(c.display_name))
			ctrl.setMaximum(c.max_val)
			ctrl.setMinimum(c.min_val)
			ctrl.setValue(c.value)
			ctrl.setSingleStep(max(1, (c.max_val-c.min_val)/25))
			ctrl.valueChanged.connect(lambda v, n=c.display_name: threadFrameGrabber.change_cam_control_setting(n, v, False))

		ctrl.setStatusTip("{}. Default value: {}. Valid range: [{}, {}]. Units: {}".format(c.display_name, c.def_val, c.min_val, c.max_val, c.unit))
		window.layoutGrpCam.addWidget(ctrl, i, 0, 1, 1)
		i += 1
	window.layoutGrpCam.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding), i, 0, 1, 1)  # Add a spacer which will take all remaining vertical space (if any, if the window is very tall)

@pyqtSlot()
def save_current_settings():
	logging.info("User requested to save current settings.")
	for chk, default_path in [(window.chk_save_cam, threadFrameGrabber.cv.CAMERA_SETTINGS_FILE),
							  (window.chk_save_color, threadFrameGrabber.cv.COLOR_THRESH_SETTINGS_FILE),
							  (window.chk_save_blob, threadFrameGrabber.cv.BLOB_DETECTOR_SETTINGS_FILE)]:
		if not chk.isChecked() or not chk.isEnabled():  # Only save these settings if user selected the corresponding CheckBox
			logging.info("Not going to {}, user didn't check this box or isn't allowed to.".format(chk.text().lower()))
			continue

		settings_desc = chk.text().replace("Save ", "").capitalize()  # Description of the settings (eg: Color threshold)
		default_file_name = settings_desc if chk != window.chk_save_cam else "{} - {} - {} - {}".format(settings_desc, threadFrameGrabber.dev_selected_name, threadFrameGrabber.dev_selected_vend_id, threadFrameGrabber.dev_selected_prod_id)
		file_name, _ = QFileDialog.getSaveFileName(window, "Save current {} to a file".format(settings_desc.lower()),
			path.join(path.dirname(path.abspath(default_path)), default_file_name), "{} (*.txt)".format(settings_desc))

		if file_name:
			result = False  # Initialize result variable to keep track of the saving settings outcome
			if chk == window.chk_save_cam and threadFrameGrabber.cap is not None:
				calib_file_name, _ = QFileDialog.getOpenFileName(window, "Do you want to include a reference to the camera calibration file in the settings?", path.abspath(CALIB_FOLDER), "NumPy Compressed Array format (*.npz)")
				result = threadFrameGrabber.cap.save_settings(file_name, calib_file_name)
			elif chk == window.chk_save_color:
				result = threadFrameGrabber.cv.save_color_thresh_settings(threadFrameGrabber.cv.cv_HSV_thresh_min, threadFrameGrabber.cv.cv_HSV_thresh_max, file_name)
			elif chk == window.chk_save_blob:
				result = threadFrameGrabber.cv.save_blob_detector_settings(threadFrameGrabber.cv.cv_blob_detect_params, file_name)

			if result:
				QMessageBox.information(window, "Current settings", "Congratulations, current {} have been successfully saved!".format(settings_desc.lower()))
			else:
				QMessageBox.critical(window, "Current settings", "Unfortunately, there was an error while attempting to save current {}. Try again later or read the log for more details.".format(settings_desc.lower()))
		else:
			logging.info("User canceled saving the {}.".format(settings_desc.lower()))

def load_default_settings():
	"""
	Updates color and blob spinBox and checkBox values to reflect the current color and blob settings.
	"""
	window.spin_color_Hmin.setValue(threadFrameGrabber.cv.cv_HSV_thresh_min[0])
	window.spin_color_Smin.setValue(threadFrameGrabber.cv.cv_HSV_thresh_min[1])
	window.spin_color_Vmin.setValue(threadFrameGrabber.cv.cv_HSV_thresh_min[2])
	window.spin_color_Hmax.setValue(threadFrameGrabber.cv.cv_HSV_thresh_max[0])
	window.spin_color_Smax.setValue(threadFrameGrabber.cv.cv_HSV_thresh_max[1])
	window.spin_color_Vmax.setValue(threadFrameGrabber.cv.cv_HSV_thresh_max[2])
	window.chk_blob_area.setChecked(threadFrameGrabber.cv.cv_blob_detect_params.filterByArea)
	window.spin_blob_areaMin.setValue(threadFrameGrabber.cv.cv_blob_detect_params.minArea)
	window.spin_blob_areaMax.setValue(threadFrameGrabber.cv.cv_blob_detect_params.maxArea)
	window.chk_blob_circ.setChecked(threadFrameGrabber.cv.cv_blob_detect_params.filterByCircularity)
	window.spin_blob_circMin.setValue(round(threadFrameGrabber.cv.cv_blob_detect_params.minCircularity*100))
	window.spin_blob_circMax.setValue(round(threadFrameGrabber.cv.cv_blob_detect_params.maxCircularity*100))
	window.chk_blob_conv.setChecked(threadFrameGrabber.cv.cv_blob_detect_params.filterByConvexity)
	window.spin_blob_convMin.setValue(round(threadFrameGrabber.cv.cv_blob_detect_params.minConvexity*100))
	window.spin_blob_convMax.setValue(round(threadFrameGrabber.cv.cv_blob_detect_params.maxConvexity*100))
	window.chk_blob_inertia.setChecked(threadFrameGrabber.cv.cv_blob_detect_params.filterByInertia)
	window.spin_blob_inertiaMin.setValue(round(threadFrameGrabber.cv.cv_blob_detect_params.minInertiaRatio*100))
	window.spin_blob_inertiaMax.setValue(round(threadFrameGrabber.cv.cv_blob_detect_params.maxInertiaRatio*100))


if __name__ == '__main__':
	app = QApplication(sys.argv)  # First of all, instantiate a QApp
	threadDevLister = CamDeviceListRefresher()  # Initialize background threads (so we can access their methods&variables)
	threadFrameGrabber = CamFrameGrabberThread()
	last_mouse_event = None
	window = loadUi('adjust_cam_settings.ui')  # Load UI from file
	# window.resizeEvent = on_resize  # Custom on_resize method to resize the QPixmap containing the current camera frame -- Not needed anymore, repaint_cam_img() will already scale the image, and is called as often as possible by timer_repaint
	window.installEventFilter(FilterNotifyWindowCloses(window))  # Install an event filter to stop background threads when the user manually closes the window
	make_all_children_ignore_scroll(window.scrollAreaWidgetContents)  # Make all input controls ignore scroll events (so their value doesn't change when the user scrolls over them, but rather the parent container is actually scrolled up/down)
	refresh_cam_device_list()  # Add item "None" to drpInput and select it...
	update_cam_ui()  # Also remove all items from grpCam
	load_default_settings()  # Update color and blob spinBox and checkBox values based on loaded settings

	# Connect input controls' "value changed" events to their corresponding processing slot (=function)
	window.drpInput.currentIndexChanged[str].connect(threadFrameGrabber.change_selected_device)
	window.drpSize.currentIndexChanged[str].connect(lambda x: threadFrameGrabber.change_cam_frame_size(x, True))
	window.drpFPS.currentIndexChanged[str].connect(threadFrameGrabber.change_cam_frame_rate)
	window.btn_save_settings.clicked.connect(save_current_settings)
	window.grpColor.clicked.connect(lambda x: threadFrameGrabber.set_cv_do_color(x))
	window.grpBlob.clicked.connect(lambda x: threadFrameGrabber.set_cv_do_blob(x))
	window.spin_color_Hmin.valueChanged.connect(lambda x: threadFrameGrabber.change_HSV_setting(x, 'H', False))
	window.spin_color_Hmax.valueChanged.connect(lambda x: threadFrameGrabber.change_HSV_setting(x, 'H', True))
	window.spin_color_Smin.valueChanged.connect(lambda x: threadFrameGrabber.change_HSV_setting(x, 'S', False))
	window.spin_color_Smax.valueChanged.connect(lambda x: threadFrameGrabber.change_HSV_setting(x, 'S', True))
	window.spin_color_Vmin.valueChanged.connect(lambda x: threadFrameGrabber.change_HSV_setting(x, 'V', False))
	window.spin_color_Vmax.valueChanged.connect(lambda x: threadFrameGrabber.change_HSV_setting(x, 'V', True))
	window.spin_blob_areaMin.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x, 'A', False))
	window.spin_blob_areaMax.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x, 'A', True))
	window.spin_blob_circMin.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x/100., 'C', False))
	window.spin_blob_circMax.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x/100., 'C', True))
	window.spin_blob_convMin.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x/100., 'V', False))
	window.spin_blob_convMax.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x/100., 'V', True))
	window.spin_blob_inertiaMin.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x/100., 'I', False))
	window.spin_blob_inertiaMax.valueChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x/100., 'I', True))
	window.chk_blob_area.stateChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x>0, 'A', True))  # CheckBox state seems to be: 0=Unchecked, 2=Checked -> Convert to bool by doing x>0
	window.chk_blob_circ.stateChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x>0, 'C', True))
	window.chk_blob_conv.stateChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x>0, 'V', True))
	window.chk_blob_inertia.stateChanged.connect(lambda x: threadFrameGrabber.change_blob_setting(x>0, 'I', True))
	window.lblCamImg.mouseMoveEvent = on_lblCamImg_mouseEvent
	window.lblCamImg.mousePressEvent = on_lblCamImg_mouseEvent

	# Connect background threads' signals to their corresponding slots (=functions)
	threadDevLister.sig_new_dev.connect(refresh_cam_device_list)
	threadFrameGrabber.sig_update_cam_ui.connect(update_cam_ui)
	threadFrameGrabber.sig_new_dev_selected.connect(threadFrameGrabber.change_selected_device)
	threadFrameGrabber.sig_error.connect(show_error_message)
	# threadFrameGrabber.sig_new_img.connect(repaint_cam_img)  # No need to do this anymore, now have a timer repainting:
	timer_repaint = QTimer()
	timer_repaint.timeout.connect(repaint_cam_img)
	timer_repaint.start(50)  # Delay of 0ms = Execute as soon as the event loop is done processing (as often as possible)

	# Finally, start background threads
	threadDevLister.start()
	threadFrameGrabber.start()

	# Now make the window visible and start the event loop (script will exit when the window is closed)
	window.show()  # Show window
	window.raise_()  # Bring it to the front
	sys.exit(app.exec_())  # Execute app event loop and exit when loop's done (window is closed)
