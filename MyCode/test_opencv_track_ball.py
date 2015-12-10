import cv2
import numpy as np
from operator import attrgetter

col_pos_tracked = (255, 0, 0)
col_pos_unknown = (0, 0, 255)

def get_detector_params():
    detector_params = cv2.SimpleBlobDetector_Params()

    # Filter by color
    detector_params.filterByColor = True
    detector_params.blobColor = 255

    # Change thresholds
    detector_params.minThreshold = 250
    detector_params.maxThreshold = 255

    # Filter by Area.
    detector_params.filterByArea = True
    detector_params.minArea = 50
    detector_params.maxArea = 50000

    # Filter by Circularity
    detector_params.filterByCircularity = False
    detector_params.minCircularity = 0.2

    # Filter by Convexity
    detector_params.filterByConvexity = False
    detector_params.minConvexity = 0.6

    # Filter by Inertia
    detector_params.filterByInertia = True
    detector_params.minInertiaRatio = 0.2

    detector_params.minRepeatability = 1
    detector_params.minDistBetweenBlobs = 50

    return detector_params

video_capture = cv2.VideoCapture(0)
video_size = (int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
video_fps = video_capture.get(cv2.CAP_PROP_FPS)
if video_fps <= 0: video_fps = 50.0
drone_target_pos = (200, 200)
drone_curr_pos = (0, 0)
drone_pos_tracked = False
if video_capture.isOpened():
    print "Camera 0 opened! :)"
    while cv2.waitKey(int(1000/video_fps)) <= 0:
        ret, frame = video_capture.read()
        if not ret:
            break
        else:
            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_HSV, np.array([110, 90, 50], np.uint8), np.array([160, 255, 255], np.uint8))
            # mask = cv2.inRange(frame_HSV, np.array([0, 0, 200], np.uint8), np.array([180,255,255], np.uint8))
            mask3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            keypoints = cv2.SimpleBlobDetector_create(get_detector_params()).detect(mask)
            if keypoints:
                drone_pos_tracked = True
                keypoint = max(keypoints, key=attrgetter('size'))   # Keep only the biggest blob
                mask_with_keypoints = cv2.drawKeypoints(mask, [keypoint], np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                cv2.circle(mask_with_keypoints, tuple(int(round(x)) for x in keypoint.pt), int(round(keypoint.size/2)), (210, 0, 210), -1)
                cv2.line(mask_with_keypoints, tuple(int(round(x)) for x in keypoint.pt), drone_target_pos, (255, 0, 0), 10)
            else:
                drone_pos_tracked = False
                mask_with_keypoints = mask3
            cv2.circle(mask_with_keypoints, drone_target_pos, 25, (0, 255, 0), -1)  # Plot circle at desired drone location

            np.putmask(frame, mask3==255, [255, 0, 0])

            cv2.imshow('Video', cv2.resize(np.vstack((frame, mask_with_keypoints)), None, fx=0.5, fy=0.5))
    video_capture.release()
    cv2.waitKey(1000)
else:
    print "Couldn't open camera! :("
