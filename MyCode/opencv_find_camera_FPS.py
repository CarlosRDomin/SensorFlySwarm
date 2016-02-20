"""
    Simple script to estimate camera frame-rate (FPS) using OpenCV (it just tries to read frames as fast as possible).
"""

import cv2
import numpy as np
from datetime import datetime, timedelta
import os


def find_FPS(cam_id=0, test_len=5.0, saveimg=False, saveimgFolder="fps_test"):
    video_capture = cv2.VideoCapture(cam_id)

    fps = video_capture.get(cv2.CAP_PROP_FPS)
    print "OpenCV says FPS is {:.2f}".format(fps)

    if fps > 0:
        return fps
    if not video_capture.isOpened():
        print "Couldn't open camera! :("
        return -1
    if saveimg:
        if os.path.exists(saveimgFolder):
            for f in os.listdir(saveimgFolder): os.remove(os.path.join(saveimgFolder, f))
        else:
            os.makedirs(saveimgFolder)

    cnt_diff_frames = 0
    last_frame = np.zeros((video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT), video_capture.get(cv2.CAP_PROP_FRAME_WIDTH), 3))
    startT = datetime.now()
    endT = startT + timedelta(seconds=test_len)
    while (datetime.now() < endT):
        t = datetime.now()
        ret, frame = video_capture.read()
        if not ret:
            print "Couldn't grab camera frame! :("
            return -1

        tt = datetime.now()
        # imgDiffPx = np.any(frame != last_frame, 2)
        imgDiffPx = (frame == last_frame)
        nDiffPx = np.sum(imgDiffPx)
        if saveimg:
            cv2.imwrite(os.path.join(saveimgFolder, t.strftime("frame_%H-%M-%S-%f.jpg")), frame)
            cv2.imwrite(os.path.join(saveimgFolder, t.strftime("diff_%H-%M-%S-%f.jpg")), imgDiffPx.astype(np.uint8)*255)
        if nDiffPx > 0:
            cnt_diff_frames += 1
            strAdd = " - New frame detected!"
        print "At {} (took {}|{}): {:7} different pixels{}".format(str(t-startT)[:-3], str(tt-t)[6:-3], str(datetime.now()-tt)[6:-3], nDiffPx, strAdd)
        # print "{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}\t{:f}".format(video_capture.get(cv2.CAP_PROP_APERTURE), video_capture.get(cv2.CAP_PROP_AUTO_EXPOSURE), video_capture.get(cv2.CAP_PROP_BACKLIGHT), video_capture.get(cv2.CAP_PROP_BRIGHTNESS), video_capture.get(cv2.CAP_PROP_CONTRAST), video_capture.get(cv2.CAP_PROP_EXPOSURE), video_capture.get(cv2.CAP_PROP_EXPOSUREPROGRAM), video_capture.get(cv2.CAP_PROP_FRAME_COUNT), video_capture.get(cv2.CAP_PROP_FOCUS), video_capture.get(cv2.CAP_PROP_GAIN), video_capture.get(cv2.CAP_PROP_GAMMA), video_capture.get(cv2.CAP_PROP_IRIS), video_capture.get(cv2.CAP_PROP_ISO_SPEED), video_capture.get(cv2.CAP_PROP_OPENNI_FOCAL_LENGTH), video_capture.get(cv2.CAP_PROP_POS_FRAMES), video_capture.get(cv2.CAP_PROP_POS_MSEC), video_capture.get(cv2.CAP_PROP_ROLL), video_capture.get(cv2.CAP_PROP_SATURATION), video_capture.get(cv2.CAP_PROP_SHARPNESS), video_capture.get(cv2.CAP_PROP_SPEED), video_capture.get(cv2.CAP_PROP_TEMPERATURE), video_capture.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U))
        last_frame = frame

    return cnt_diff_frames/float(test_len)

if __name__ == '__main__':
    print "FPS: {:6.3f}".format(find_FPS())