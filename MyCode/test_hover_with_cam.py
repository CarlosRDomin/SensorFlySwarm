import logging
import time
from datetime import datetime
import sys
import os
from threading import Timer
import cv2
import numpy as np
from operator import attrgetter


sys.path.insert(0, "../Crazyflie client/build/lib")
from cflib.crazyflie import Crazyflie
from cflib import crtp
from cfclient.utils.logconfigreader import LogConfig  # noqa


class Hover:

    def __init__(self):
        self.m_bConnecting = True
        self.m_bConnected = False
        self.m_CrazyFlie = None
        self.m_log_stab = None
        self.m_yaw = 0
        self.m_takeOff_start = None
        self.m_takeOff_end = None
        self.m_video_capture = cv2.VideoCapture()
        self.m_video_size = (0, 0)
        self.m_video_folder = "img"
        self.m_figure_name = "Output"
        self.m_imshow_out = None
        ### self.m_background_image = None
        self.m_drone_detector = self.init_detector_params()
        self.m_drone_pos_tracked = False
        self.m_drone_curr_pos = np.array([0, 0])
        self.m_drone_target_pos = np.array([0, 0])
        self.m_drone_target_yaw = 0
        self.m_col_ball_tracked = (255, 0, 0)
        self.m_col_line_tracked = (255, 0, 0)
        self.m_col_target_tracked = (0, 255, 0)
        self.m_col_target_untracked = (0, 0, 255)
        self.m_PIDs = {"thrust_offs": 46500, "thrust_p": 4, "roll_offs": 5, "roll_p": 1/25., "pitch_offs": 0, "pitch_p": 0, "yaw_offs": 0, "yaw_p": 1}

    def init_detector_params(self):
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

    def init_video_cam(self):
        self.m_video_capture.open(0)
        if self.m_video_capture.isOpened():
            logging.info("Camera opened! :)")
            self.m_video_size = (int(self.m_video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.m_video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            self.m_drone_target_pos = (np.array(self.m_video_size)/2).astype(int)
            self.m_drone_curr_pos = self.m_drone_target_pos  # Initialize drone's position at target position so PID loop doesn't go crazy at the beginning
            cv2.namedWindow(self.m_figure_name)
            cv2.setMouseCallback(self.m_figure_name, self.on_mouse_callback)
            cv2.moveWindow(self.m_figure_name, 200, 200)
        else:
            raise Exception("Couldn't open camera! :(")

        # Prepare the folder self.m_video_folder so we can log each frame processed (for debugging)
        if os.path.exists(self.m_video_folder):
            for f in os.listdir(self.m_video_folder): os.remove(os.path.join(self.m_video_folder, f))
        else:
            os.makedirs(self.m_video_folder)

    def run(self):
        logging.basicConfig(level=logging.INFO)  # Initialize logger, only show messages of level INFO or higher

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

            if self.m_bConnected:
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

                raw_input("\nRotate the drone so it faces the camera, press Enter when you're ready...\n")
                self.m_drone_target_yaw = self.m_yaw
                print "Target yaw set at %.2f." % self.m_yaw

                self.init_video_cam()

                # Also take an image from the background, for background substraction
                _, frame = self.m_video_capture.read()
                ### self.m_background_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                cv2.imshow(self.m_figure_name, frame)
                cv2.waitKey(10000)  # Give 10 sec to prepare for take-off

                # self.m_takeOff_start = datetime.now()
                # self.m_takeOff_end = self.m_takeOff_start + 10

                t = Timer(20, self.m_CrazyFlie.close_link)  # Start a timer to disconnect in 10s
                t.start()

            cnt = 0
            while cnt < 20:
                cnt += 1
                self.m_CrazyFlie.commander.send_setpoint(6, 8, 0, self.m_PIDs["thrust_offs"])
                if cv2.waitKey(100) > 0:
                    raise Exception("KeyboardInterrupt")

            while self.m_bConnected:
                self.m_bConnected = self.hover() & self.m_bConnected

        except Exception as e:
            logging.error("Shutting down due to an exception: %s." % str(e))
        self.m_CrazyFlie.close_link()
        self.m_video_capture.release()

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
            px = self.m_imshow_out[y, x]
            px_HSV = cv2.cvtColor(np.array([[px]]), cv2.COLOR_BGR2HSV).ravel()
            print "MOUSE_CLICK! At: (x=%d, y=%d). Pixel value: [H=%d, S=%d, V=%d] = [R=%d, G=%d, B=%d]" % (x, y, px_HSV[0], px_HSV[1], px_HSV[2], px[2], px[1], px[0])

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
        self.m_yaw = data['stabilizer.yaw']
        print "\rCurrent yaw: %.2fdeg" % self.m_yaw,

    def max_abs(self, x, m):
        if x >= 0:
            return min(x, m)
        else:
            return max(x, -m)

    def hover(self, sendCommand=True, saveImg=True):
        # self.m_video_capture.set(cv2.CAP_PROP_EXPOSURE, 100)
        # self.m_video_capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.m_video_capture.set(cv2.CAP_PROP_IRIS, 10)
        # self.m_video_capture.set(cv2.CAP_PROP_SATURATION, 10)
        # print str(self.m_video_capture.get(cv2.CAP_PROP_EXPOSURE)) + ";\t" + str(self.m_video_capture.get(cv2.CAP_PROP_AUTO_EXPOSURE)) + ";\t" + str(self.m_video_capture.get(cv2.CAP_PROP_IRIS)) + ";\t" + str(self.m_video_capture.get(cv2.CAP_PROP_SATURATION))
        ret, frame = self.m_video_capture.read()
        if not ret:
            logging.error("Unexpected error accessing the camera frame :(")
            return False
        else:
            frame = cv2.GaussianBlur(frame, (3, 3), 0)
            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ### mask_bkgnd = 1 - (np.sum(abs(self.m_background_image-frame_HSV) < 20, 2) == 3).astype(np.uint8)
            # mask = mask_bkgnd * cv2.inRange(frame_HSV, np.array([130, 100, 60], np.uint8), np.array([170, 160, 255], np.uint8))
            mask = cv2.inRange(frame_HSV, np.array([110, 60, 50], np.uint8), np.array([170, 255, 255], np.uint8))
            # mask = cv2.inRange(frame_HSV, np.array([115, 75, 50], np.uint8), np.array([140, 150, 160], np.uint8))
            # mask = cv2.inRange(frame_HSV, np.array([130, 90, 50], np.uint8), np.array([160, 150, 255], np.uint8))
            mask = cv2.erode(mask, None, iterations=3)
            mask = cv2.dilate(mask, None, iterations=10)
            mask3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            keypoints = cv2.SimpleBlobDetector_create(self.m_drone_detector).detect(mask)
            if keypoints:
                self.m_drone_pos_tracked = True
                keypoint = max(keypoints, key=attrgetter('size'))   # Keep only the biggest blob
                mask_with_keypoints = cv2.drawKeypoints(mask, [keypoint], np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                # self.m_drone_curr_pos = tuple(int(round(x)) for x in keypoint.pt)
                self.m_drone_curr_pos = np.array(keypoint.pt)
                cv2.circle(mask_with_keypoints, tuple(self.m_drone_curr_pos.astype(int)), int(round(keypoint.size/2)), self.m_col_ball_tracked, -1)
                cv2.line(mask_with_keypoints, tuple(self.m_drone_curr_pos.astype(int)), tuple(self.m_drone_target_pos), self.m_col_line_tracked, 10)
            else:
                self.m_drone_pos_tracked = False
                mask_with_keypoints = mask3
            cv2.circle(mask_with_keypoints, tuple(self.m_drone_target_pos), 25, self.m_col_target_tracked if self.m_drone_pos_tracked else self.m_col_target_untracked, -1)  # Plot circle at desired drone location

            self.m_imshow_out = cv2.resize(np.vstack((frame, mask_with_keypoints)), None, fx=0.5, fy=0.5)
            ### np.putmask(frame, np.stack((np.zeros_like(mask_bkgnd), np.zeros_like(mask_bkgnd), mask_bkgnd), 2)>0, [0, 0, 255])
            np.putmask(frame, mask3==255, [0, 255, 0])

            out = cv2.resize(np.vstack((frame, mask_with_keypoints)), None, fx=0.5, fy=0.5)
            cv2.imshow(self.m_figure_name, out)
            t = datetime.now()
            if saveImg:
                cv2.imwrite(os.path.join(self.m_video_folder, t.strftime("out_%H-%M-%S-%f.jpg")), out)

            send_roll = self.max_abs(self.m_PIDs["roll_offs"] + self.m_PIDs["roll_p"]*(self.m_drone_curr_pos-self.m_drone_target_pos)[0], 5)  # Positive roll = tilted right
            send_pitch = self.max_abs(self.m_PIDs["pitch_offs"] + self.m_PIDs["pitch_p"]*0, 5)
            send_yaw = (self.m_PIDs["yaw_offs"] + self.m_yaw-self.m_drone_target_yaw) % 360
            if send_yaw > 180: send_yaw -= 360  # Back to (-180, 180] range
            send_yaw = self.max_abs(send_yaw*self.m_PIDs["yaw_p"], 5)
            send_thrust = self.m_PIDs["thrust_offs"] + self.m_PIDs["thrust_p"]*(self.m_drone_curr_pos-self.m_drone_target_pos)[1]
            if sendCommand:
                self.m_CrazyFlie.commander.send_setpoint(send_roll, send_pitch, send_yaw, send_thrust)
                print "Sent:\troll=%.2f\tpitch=%.2f\tyaw=%.2f\tthrust=%.2f\tcurr_pos=%s\t@%s" % (send_roll, send_pitch, send_yaw, send_thrust, self.m_drone_curr_pos, t.strftime("%H:%M:%S.%f"))
            return cv2.waitKey(1) <= 0


if __name__ == '__main__':
    sendCommands = True
    sendCommands = False
    if sendCommands:
        Hover().run()
    else:
        h = Hover()
        h.init_video_cam()
        while h.hover(False, False):
            time.sleep(1/30.0)
