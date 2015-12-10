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
        self.m_video_capture = cv2.VideoCapture()
        self.m_video_size = (0, 0)
        self.m_video_folder = "img"
        self.m_background_image = None
        self.m_drone_detector = self.init_detector_params()
        self.m_drone_pos_tracked = False
        self.m_drone_curr_pos = np.array([0, 0])
        self.m_drone_target_pos = np.array([0, 0])
        self.m_drone_target_yaw = 0
        self.m_col_ball_tracked = (255, 0, 0)
        self.m_col_line_tracked = (255, 0, 0)
        self.m_col_target_tracked = (0, 255, 0)
        self.m_col_target_untracked = (0, 0, 255)

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
                # if cnt >= 15:
                #     raise Exception("Connection to Crazyflie timed out, unable to establish a useful link after 15sec.")
                # if cnt % 5 == 0:
                #     logging.warning("Unable to establish a connection with Crazyflie (%s) after 5 sec. Retrying..." % linkUri)
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
                    self.m_CrazyFlie.log.add_config(self.m_log_stab)
                    self.m_log_stab.data_received_cb.add_callback(self.log_stabilizer_data)
                    self.m_log_stab.error_cb.add_callback(self.log_stabilizer_error)
                    self.m_log_stab.start()
                    self.m_CrazyFlie.param.set_value("ring.effect", "0")  # Turn off LED ring
                except KeyError as e:
                    raise Exception("Couldn't start log configuration, %s not found in TOC!" % str(e))
                except AttributeError:
                    raise Exception("Couldn't add Stabilizer log config, bad configuration.")

                self.m_video_capture.open(0)
                if self.m_video_capture.isOpened():
                    logging.info("Camera opened! :)")
                    self.m_video_size = (int(self.m_video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.m_video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
                    # self.m_drone_target_pos = tuple(int(round(x/2)) for x in self.m_video_size)
                    self.m_drone_target_pos = (np.array(self.m_video_size)/2).astype(int)
                    self.m_drone_curr_pos = self.m_drone_target_pos
                else:
                    raise Exception("Couldn't open camera! :(")

                raw_input("\nRotate the drone so it faces the camera, press Enter when you're ready...\n")
                self.m_drone_target_yaw = self.m_yaw
                print "Target yaw set at %.2f." % self.m_yaw

                # Also take an image from the background, for background substraction
                _, frame = self.m_video_capture.read()
                self.m_background_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                cv2.imshow('Video', frame)
                cv2.waitKey(10000)

                # t = Timer(10, self.m_CrazyFlie.close_link)  # Start a timer to disconnect in 10s
                # t.start()

            cnt = 0
            while cnt < 10:
                cnt += 1
                self.m_CrazyFlie.commander.send_setpoint(5, 5, 0, 42000)
                if cv2.waitKey(100) > 0:
                    raise KeyboardInterrupt

            if os.path.exists(self.m_video_folder):
                for f in os.listdir(self.m_video_folder): os.remove(os.path.join(self.m_video_folder, f))
            else:
                os.makedirs(self.m_video_folder)

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

    def hover(self):
        ret, frame = self.m_video_capture.read()
        if not ret:
            logging.error("Unexpected error accessing the camera frame :(")
            return False
        else:
            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_bkgnd = 1 - (np.sum(abs(self.m_background_image-frame_HSV) < 20, 2) == 3).astype(np.uint8)
            mask = mask_bkgnd * cv2.inRange(frame_HSV, np.array([110, 90, 50], np.uint8), np.array([160, 255, 255], np.uint8))
            # mask = cv2.inRange(frame_HSV, np.array([0, 0, 200], np.uint8), np.array([180,255,255], np.uint8))
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

            np.putmask(frame, np.stack((np.zeros_like(mask_bkgnd), np.zeros_like(mask_bkgnd), mask_bkgnd), 2)>0, [0, 0, 255])
            np.putmask(frame, mask3==255, [0, 255, 0])

            out = cv2.resize(np.vstack((frame, mask_with_keypoints)), None, fx=0.5, fy=0.5)
            cv2.imshow('Video', out)
            cv2.imwrite(os.path.join(self.m_video_folder, datetime.now().strftime("out_%H-%M-%S-%f.png")), out)
            send_roll = (self.m_drone_target_pos-self.m_drone_curr_pos)[0]/30  # Positive roll = tilted right
            send_pitch = 0
            send_yaw = (self.m_drone_target_yaw-self.m_yaw) % 360
            if send_yaw > 180: send_yaw -= 360  # Back to (-180, 180] range
            send_yaw /= 3
            send_thrust = 38000 - 10*(self.m_drone_target_pos-self.m_drone_curr_pos)[1]
            #self.m_CrazyFlie.commander.send_setpoint(send_roll, send_pitch, send_yaw, send_thrust)
            print "Sending:\troll=%.2f\tpitch=%.2f\tyaw=%.2f\tthrust=%.2f\tcurr_pos=%s" % (send_roll, send_pitch, send_yaw, send_thrust, self.m_drone_curr_pos)
            return cv2.waitKey(1) <= 0


if __name__ == '__main__':
    Hover().run()
