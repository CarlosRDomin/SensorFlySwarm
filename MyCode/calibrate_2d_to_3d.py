"""
	Auxiliary functions to assess the accuracy/robustness of the vision tracking algorithm without actually flying worker
	drones. Thus, current settings (HSV thresholds, blob detector, camera controls such as exposure time...) can be tested
	to make sure the CV algorithm is doing its job before flying any drones.
"""

import logging
import cv2
import vision_aux_functions as auxV
from full_control_with_cam import Spotter, FakeWorkerDrone, GroundTruthThread
from uvc_capture import UvcCapture
from calibrate_cam_params import CALIB_FOLDER, generate_calibration_filename, ensure_folder_exists
from datetime import datetime


def find_focal_length_in_px(radius_in_m, dist_in_m):
	"""
	Estimates the camera's focal length (in px) by finding an object of known size at a known distance.
	:param radius_in_m: Radius (in m) of the marker/sphere to look for (worker's ping pong ball)
	:param dist_in_m: Distance (in m) between the camera and the marker
	:return: Focal length (in px) of the camera, which is the apparent length (in px) of an object that measures "x" m and is placed "x" m away from the camera.
	"""
	cv = Spotter()
	cv.init_video_cam_and_cv_algorithm(False)
	cv.video_capture.do_undistort = False
	cv.workers.append(FakeWorkerDrone())  # Add a fake worker (we just need the cv algorithm to look for 1 ping pong ball)

	cf_curr_pos = None
	while cf_curr_pos is None:  # Keep processing frames until we detect a CF in one
		cf_curr_pos = cv.detect_cf_in_camera()[0]  # detect_cf_in_camera() now returns an array -> Element [0] is the "worker" we're interested in

	radius_in_px = cf_curr_pos[2]
	F = radius_in_px*dist_in_m/radius_in_m
	print("True rad: {:.2f}m\nMeasured rad: {:.6f}px; At a d={:.2f}m -> F={:.6f}px".format(radius_in_m, radius_in_px, dist_in_m, F))

	ensure_folder_exists(CALIB_FOLDER)
	cv2.imwrite(generate_calibration_filename("2d_to_3d_orig.jpg"), cv.cv_cam_frame)
	cv2.circle(cv.cv_cam_frame, tuple(cf_curr_pos[0:2].astype(int)), int(cf_curr_pos[2]), (255, 0, 0), -1)
	cv2.putText(cv.cv_cam_frame, "True rad: {}, measured: {}; At a d={} -> F={}".format(radius_in_m, radius_in_px, dist_in_m, F), (50, cv.cv_cam_frame.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 0, 200), 1, cv2.LINE_AA)
	cv2.imwrite(generate_calibration_filename("2d_to_3d_OSD.jpg"), cv.cv_cam_frame)
	cv2.imshow("Calibration result!", cv.cv_cam_frame)
	cv2.waitKey(5000)
	cv2.destroyAllWindows()

	return F

def estimate_3d_location(use_spotter_cam=True):
	"""
	Opens a GUI and displays the camera's output. If a worker drone can be found, it also displays its world coordinates.
	This is very useful to assess the accuracy and/or resilience of the vision tracking algorithm.
	:param use_spotter_cam: If True, use Spotter's camera settings; If False, use GroundTruthThread's cam settings
	"""
	if not use_spotter_cam:
		Spotter.CAMERA_SETTINGS_FILE = GroundTruthThread.CAMERA_SETTINGS_FILE
	cv = Spotter(bool_world_coords_pattern=True)
	cv.init_video_cam_and_cv_algorithm(False)
	cv.workers.append(FakeWorkerDrone())  # Add a fake worker (we just need the cv algorithm to look for 1 ping pong ball)

	win_title = "3D position estimation"
	cv2.namedWindow(win_title)
	cv2.moveWindow(win_title, 100,100)
	while cv2.waitKey(1) == 255:  # OpenCV 3.1 and older used to return <0 when no key was pressed. Seems like now 255 is returned in that case...
		cf_curr_pos = cv.detect_cf_in_camera()[0]  # detect_cf_in_camera() now returns an array -> Element [0] is the "worker" we're interested in
		if cf_curr_pos is not None:
			world_coords = cv.img_to_cf_world_coords(cf_curr_pos)
			cv2.circle(cv.cv_cam_frame, tuple(cf_curr_pos[0:2].astype(int)), int(cf_curr_pos[2]), (255, 0, 0), 1)
			cv2.putText(cv.cv_cam_frame, "x={p[0]:.2f}m, y={p[1]:.2f}m, z={p[2]:.2f}m; dist={d:.2f}cm".format(p=world_coords, d=auxV.world_to_cam_coords([world_coords[0], -world_coords[1], -world_coords[2]], cv.world_to_camera_transf)[2]), (50, cv.cv_cam_frame.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 0, 200), 1, cv2.LINE_AA)
		else:
			cv2.putText(cv.cv_cam_frame, "CAN'T TRACK CF!!", (50, cv.cv_cam_frame.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
		cv2.imshow(win_title, cv.cv_cam_frame)

	cv2.destroyAllWindows()

def estimate_cam_to_world_transform(calibration_file, is_chessboard=False, cell_size=0.02, use_spotter_cam=True):
	"""
	Opens a GUI and displays the camera's output. For every frame, it also looks for a calibration pattern (eg: OpenCV's
	chessboard pattern), and computes the cam->world transformation matrix if found.
	:param calibration_file: Str indicating the name of the calibration file to load camera_matrix and dist_coefs from
	:param is_chessboard: True if the calibration pattern is a chessboard. False, for asymmetric circle grid
	:param cell_size: Float indicating the distance (in m) between two consecutive points in the pattern grid
	:param use_spotter_cam: If True, use Spotter's camera settings; If False, use GroundTruthThread's cam settings
	"""
	cam = UvcCapture.new_from_settings(Spotter.CAMERA_SETTINGS_FILE if use_spotter_cam else GroundTruthThread.CAMERA_SETTINGS_FILE)
	if cam is None:
		logging.error("Please connect the camera, can't do this without you :P")
		return
	cam.do_undistort = False  # Don't undistort, we're already taking into account camera_matrix in the equations
	# for c in cam.controls:
	# 	if c.display_name == "Sharpness":
	# 		c.value = c.max_val
	# 		break

	win_title = "Estimating camera->world coordinate transform"
	cv2.namedWindow(win_title)
	cv2.moveWindow(win_title, 100,100)
	while cv2.waitKey(1) < 0:
		t_start = datetime.now()
		img = cam.get_frame_robust().bgr
		world_to_camera_transf, F = auxV.find_world_to_cam_and_F(img, calibration_file, is_chessboard=is_chessboard, pattern_grid_or_cell_size=cell_size)
		if world_to_camera_transf is not None:
			pattern_grid, _ = auxV.get_calib_pattern_info(is_chessboard, cell_size)
			found, corners = auxV.find_calib_pattern(img, is_chessboard, pattern_grid)
			cv2.drawChessboardCorners(img, pattern_grid, corners, found)
			cam_pos = auxV.cam_to_world_coords([0, 0, 0], world_to_camera_transf)
			d = auxV.world_to_cam_coords([0, 0, 0], world_to_camera_transf)[2]
			cv2.putText(img, "[x={p[0]:.2f}, y={p[1]:.2f}, z={p[2]:.2f}] cm; dist={d:.2f}cm; F={F:.2f}px".format(p=100*cam_pos, d=100*d, F=F), (50, img.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 200), 1, cv2.LINE_AA)

			# # Test img->world + world->img transforms to make sure everything works correctly
			# dist_img_coords = [img.shape[1], img.shape[0], 20]
			# RADIUS_IN_M = 0.02
			# depth = F * RADIUS_IN_M/dist_img_coords[2]  # If an object that's dist_img_coords[2] pixels wide measures RADIUS_IN_M meters, how far away from the camera is it?
			# cam_coords = auxV.img_to_cam_coords(np.hstack((dist_img_coords[0:2], depth)), calibration_file, is_dist_img_coords=True)
			# world_coords = auxV.cam_to_world_coords(cam_coords, world_to_camera_transf)
			# new_cam_coords = auxV.world_to_cam_coords(world_coords, world_to_camera_transf)
			# new_dist_img_coords = auxV.cam_to_img_coords(new_cam_coords, calibration_file, want_dist_img_coords=True)
			# cv2.putText(img, "[u={p[0]:.2f}, v={p[1]:.2f}]".format(p=new_dist_img_coords), (50, img.shape[0]-100), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 200), 1, cv2.LINE_AA)
			# cv2.circle(img, tuple([int(x) for x in new_dist_img_coords]), int(dist_img_coords[2]), (0, 200, 0), -1)
		else:
			cv2.putText(img, "CAN'T FIND PATTERN!!", (50, img.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

		cv2.imshow(win_title, img)
		t_end = datetime.now()
		print("Frame took {:.2f}ms to process!".format((t_end-t_start).total_seconds()*1000))

	cv2.destroyAllWindows()

if __name__ == '__main__':
	if True:
		calibrate_cam = False
		F = find_focal_length_in_px(0.02, 1.00) if calibrate_cam else Spotter.CAM_FOCAL_LENGTH_IN_PX #1900 #1250.0
		estimate_3d_location()
	else:
		estimate_cam_to_world_transform("/Users/Carlitos/GoogleDrive/UNI CARLOS/Grad school/Research/SensorFlySwarm/MyCode/cam_calibration/cam_calibration_output_spotterL36.npz")
		estimate_cam_to_world_transform("/Users/Carlitos/GoogleDrive/UNI CARLOS/Grad school/Research/SensorFlySwarm/MyCode/cam_calibration/cam_calibration_output_GT.npz", use_spotter_cam=False)
