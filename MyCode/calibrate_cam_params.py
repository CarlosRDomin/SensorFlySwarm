"""
Camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

usage:
	calibrate.py [--debug <output path>] [--cell_size] [<image mask>]

default values:
	--debug:    ./output/
	--cell_size: 0.02
	<image mask> defaults to ../data/left*.jpg
"""

from __future__ import print_function  # Python 2/3 compatibility

import numpy as np
import cv2
import sys
import os
import getopt
import vision_aux_functions as auxV
from glob import glob
from datetime import datetime


CALIB_PREFIX = "cam_calibration"
CALIB_FOLDER = "./{}/".format(CALIB_PREFIX)
CALIB_INPUT_PREFIX = "input"
CALIB_INPUT_FORMAT = "jpg"

def generate_calibration_filename(name):
	return os.path.join(CALIB_FOLDER, "{}_{}".format(CALIB_PREFIX, name))

def ensure_folder_exists(folder):
	if not os.path.isdir(folder):
		os.mkdir(folder)

def split_filename(filename):
	path, filename = os.path.split(filename)
	name, ext = os.path.splitext(filename)
	return path, name, ext

def take_calibration_images(cam_settings=None):
	"""
	Capture calibration images by showing a live feed of the camera and capturing consecutive images when Space is pressed.
	"""
	from full_control_with_cam import Spotter, GroundTruthThread  # Don't require full_control_with_cam unless they need to capture calibration images
	cv = Spotter()  # Create a Spotter object so we use the same USB cam as the one the Spotter will use
	if cam_settings is not None: cv.CAMERA_SETTINGS_FILE = cam_settings if True else GroundTruthThread.CAMERA_SETTINGS_FILE
	cv.init_video_cam_and_cv_algorithm(False)  # Init cam
	cv.video_capture.do_undistort = False
	for c in cv.video_capture.controls:
		if c.display_name == ["Sharpness"]:
			c.value = c.max_val
			break

	ensure_folder_exists(CALIB_FOLDER)  # Make sure folder exists to prevent errors on file saving
	for file_name in glob(generate_calibration_filename("{}*".format(CALIB_INPUT_PREFIX))):
		os.remove(file_name)  # Remove any file from a previous calibration so they don't get mixed

	cnt_img = 0
	win_title = "Capturing calibration images - {} captured".format(cnt_img)
	cv2.namedWindow(win_title)
	cv2.moveWindow(win_title, 100,100)
	while True:
		cv.detect_cf_in_camera(find_blob=False)
		cv2.imshow(win_title, cv.cv_cam_frame)

		key = cv2.waitKey(1)
		if key == ord(' '):  # Save a new image when Space is pressed
			cnt_img += 1
			file_name = generate_calibration_filename("{}_{}.{}".format(CALIB_INPUT_PREFIX, cnt_img, CALIB_INPUT_FORMAT))
			cv2.setWindowTitle(win_title, "{} - {} captured".format(win_title.split(" - ")[0], cnt_img))
			cv2.imwrite(file_name, cv.cv_cam_frame)
			print("Captured new calibration image: {}".format(file_name))
		elif key >= 0:  # Exit if a key other than Space was pressed
			break

	cv2.destroyAllWindows()
	return cv.video_capture.name, cv.video_capture.vend_id, cv.video_capture.prod_id


if __name__ == '__main__':
	args, img_mask = getopt.getopt(sys.argv[1:], '', ['debug=', 'cell_size=', 'is_chessboard='])
	args = dict(args)
	args.setdefault('--debug', CALIB_FOLDER)
	args.setdefault('--cell_size', 0.02)
	args.setdefault('--is_chessboard', False)

	cam_name, cam_vend_id, cam_prod_id = "<No camera name provided>", -1, -1  # Initialize calibration cam info
	if not img_mask or img_mask[0] == 'take_pics':  # If an image mask wasn't provided or user requests to capture calib images
		if img_mask:  # if img_mask[0] == 'take_pics', first take the calibration pictures
			cam_name, cam_vend_id, cam_prod_id = take_calibration_images()
		img_mask = generate_calibration_filename("{}*.{}".format(CALIB_INPUT_PREFIX, CALIB_INPUT_FORMAT))
	else:  # If an image mask was provided, use it
		img_mask = img_mask[0]

	img_names = glob(img_mask)
	debug_dir = args.get('--debug')
	ensure_folder_exists(debug_dir)  # Make sure folder exists to prevent errors on file saving
	cell_size = float(args.get('--cell_size'))
	is_chessboard = bool(args.get('--is_chessboard'))

	pattern_grid, pattern_points = auxV.get_calib_pattern_info(is_chessboard, cell_size)
	obj_points = []
	img_points = []
	h, w = 0, 0
	img_names_undistort = []
	for filename in img_names:
		print('Processing {}... '.format(filename), end='')
		img = cv2.imread(filename, 0)
		if img is None:
			print("Failed to load", filename)
			continue

		h, w = img.shape[:2]
		found, corners = auxV.find_calib_pattern(img, is_chessboard, pattern_grid)

		if debug_dir:
			vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
			cv2.drawChessboardCorners(vis, pattern_grid, corners, found)
			path, name, ext = split_filename(filename)
			outfile = os.path.join(debug_dir, '{}_chess.png'.format(name))
			cv2.imwrite(outfile, vis)
			if found:
				img_names_undistort.append(outfile)

		if not found:
			print('Chessboard not found')
			continue

		img_points.append(corners.reshape(-1, 2))
		obj_points.append(pattern_points)

		print('ok')

	# calculate camera distortion
	rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
	new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1)

	print("\nRMS:", rms)
	print("Camera matrix:\n", camera_matrix)
	print("New camera matrix:\n", new_camera_matrix)
	print("Distortion coefficients: ", dist_coefs.ravel())
	print("ROI: ", roi)
	np.savez_compressed(generate_calibration_filename("output.npz"), rms=rms, camera_matrix=camera_matrix, dist_coefs=dist_coefs, rvecs=rvecs, tvecs=tvecs, cam_frame_size=(w, h), cam_name=cam_name, cam_vend_id=cam_vend_id, cam_prod_id=cam_prod_id, calib_at=datetime.now())

	# Undistort the image with the calibration
	print('')
	for file_name in glob(generate_calibration_filename("*undistorted*")):
		os.remove(file_name)  # Remove any file from a previous calibration so they don't get mixed

	undist_maps = cv2.initUndistortRectifyMap(camera_matrix, dist_coefs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)  # Generate undistortion mappings (will make undistort() much faster, since we are reusing the same mappings over and over)
	for img_found in img_names_undistort:
		img = cv2.imread(img_found)

		# h, w = img.shape[:2]
		t_start = datetime.now()
		dst = cv2.remap(img, undist_maps[0], undist_maps[1], cv2.INTER_LINEAR)
		# x, y, w, h = roi, dst = dst[y:y+h, x:x+w]  # crop and save the image
		t_end = datetime.now()

		path, name, ext = split_filename(img_found)
		outfile = os.path.join(path, '{}_undistorted.png'.format(name))
		print('Undistorted image written to: {}; Took {:.2f}ms'.format(outfile, (t_end - t_start).total_seconds()*1000))
		cv2.imwrite(outfile, dst)

	total_error = 0
	for i in xrange(len(obj_points)):
		img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coefs)
		error = cv2.norm(img_points[i], np.squeeze(img_points2), cv2.NORM_L2)/len(img_points2)
		total_error += error

	print("Total error: {}. Mean error: {}".format(total_error, total_error / len(obj_points)))
