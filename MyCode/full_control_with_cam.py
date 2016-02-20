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
import sys
import os
import cv2
import numpy as np
from operator import attrgetter
import PID
import plot_tools

sys.path.insert(0, "../Crazyflie client/build/lib")
from cflib.crazyflie import Crazyflie
from cflib import crtp
from cfclient.utils.logconfigreader import LogConfig  # noqa


class Hover:
    COLOR_BALL_TRACKED = (255, 0, 0)
    COLOR_BALL_UNTRACKED = (0, 0, 255)
    COLOR_LINE_TRACKED = (255, 0, 0)
    COLOR_LINE_UNTRACKED = (0, 0, 255)
    COLOR_TARGET_TRACKED = (0, 255, 0)
    COLOR_TARGET_UNTRACKED = (0, 0, 255)
    FIGURE_NAME = "Output"
    VIDEO_FOLDER = "img"
    ASK_FOR_TARGET_YAW = False
    TAKEOFF_THRUST = 47000

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
        self.m_desired_fps = 15
        self.m_video_capture = cv2.VideoCapture()
        self.m_video_size = (0, 0)
        self.m_imshow_out = None
        self.DRONE_DETECTOR_PARAMS = self.init_detector_params()
        self.m_drone_pos_tracked = False
        ## self.m_drone_last_pos = np.array([0, 0])
        ## self.m_drone_curr_pos = np.array([0, 0])
        ## self.m_drone_target_pos = np.array([0, 0])
        ## self.m_drone_curr_vel = np.array([0, 0])
        ## self.m_PID_roll = PID.PID(P=1, I=2, D=0.01, offs=3, out_max=20)
        self.m_PID_roll = PID.PIDposAndVel(posP=0.5, velP=0.007, velI=0.02, vel_offs=5, pos_out_max=300, vel_out_max=30, vel_invert_error=True)
        self.m_PID_pitch = PID.PID(P=1, I=2, D=0.01, offs=10, out_max=20, invert_error=True)
        self.m_PID_yaw = PID.PID(P=0.5, I=0.3, D=0, offs=0, out_max=20, invert_error=True, error_in_degrees=True)
        self.m_PID_thrust = PID.PIDposAndVel(posP=1, velP=45, velI=15, vel_offs=45000, pos_out_max=300, vel_out_max=5000, pos_invert_error=True, vel_invert_input=True)
        ## self.m_PID_thrust_pos = PID.PID(P=1, I=0, D=0, offs=0, out_max=500, invert_error=True)
        ## self.m_PID_thrust_vel = PID.PID(P=40, I=15, D=0, offs=45000, out_max=5000, I_max=5000)

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
            self.m_PID_roll.setSetPoint(self.m_video_size[0]/2)
            self.m_PID_thrust.setSetPoint(self.m_video_size[1]/2)
            ## self.m_drone_target_pos = (np.array(self.m_video_size)/2).astype(int)
            ## self.m_drone_curr_pos = self.m_drone_target_pos  # Initialize drone's position at target position so PID loop doesn't go crazy at the beginning
            ## self.m_drone_last_pos = self.m_drone_curr_pos
            cv2.namedWindow(self.FIGURE_NAME)
            cv2.setMouseCallback(self.FIGURE_NAME, self.on_mouse_callback)
            cv2.moveWindow(self.FIGURE_NAME, 200, 200)
        else:
            raise Exception("Couldn't open camera! :(")

        # Prepare the folder self.m_video_folder so we can log each frame processed (for debugging)
        if os.path.exists(self.VIDEO_FOLDER):
            for f in os.listdir(self.VIDEO_FOLDER): os.remove(os.path.join(self.VIDEO_FOLDER, f))
        else:
            os.makedirs(self.VIDEO_FOLDER)

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

        # try:
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

        # Also take an image from the background, for background substraction
        _, frame = self.m_video_capture.read()
        cv2.imshow(self.FIGURE_NAME, frame)
        cv2.waitKey(5000)  # Give 10 sec to prepare for take-off

        # t = Timer(20, self.m_CrazyFlie.close_link)  # Start a timer to disconnect in 10s
        # t.start()

        # cnt = 0
        # while cnt < 5:  # x 100ms
        #     cnt += 1
        #     self.m_CrazyFlie.commander.send_setpoint(8, 10, 0, 46000)
        #     if cv2.waitKey(100) > 0:
        #         raise Exception("KeyboardInterrupt")

        self.m_str_status = "TAKING OFF"
        self.m_t_start = datetime.now()
        self.m_PID_roll.clear()
        self.m_PID_pitch.clear()
        self.m_PID_yaw.clear()
        self.m_PID_thrust.clear()
        ## self.m_PID_thrust_pos.clear()
        ## self.m_PID_thrust_vel.clear()

        tStop = None
        while tStop is None:
            tStop = self.hover()

        print "AT t={}, A KEY WAS PRESSED -> STOPPING!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])
        self.m_str_status = "STOPPED"

        while datetime.now() < tStop:
            self.hover()

        self.m_CrazyFlie.commander.send_setpoint(0, 0, 0, 0)

        # except Exception as e:
        #     exc_type, exc_obj, exc_tb = sys.exc_info()
        #     logging.error("Shutting down due to an exception -> %s: %s (at %s:%d)." % (str(exc_type), str(e), os.path.split(exc_tb.tb_frame.f_code.co_filename)[1], exc_tb.tb_lineno))
        self.m_CrazyFlie.close_link()
        self.m_video_capture.release()
        cv2.destroyAllWindows()
        self.m_log_control_data.plot()

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
        self.m_roll = data['stabilizer.roll']
        self.m_pitch = data['stabilizer.pitch']
        self.m_yaw = data['stabilizer.yaw']
        print "\rCurrent yaw: %.2fdeg" % self.m_yaw,

    def hover(self, sendCommand=True, saveImg=True):
        ret, frame = self.m_video_capture.read()  # read() blocks execution until a new frame arrives! -> Obtain t AFTER grabbing the frame
        t = datetime.now()
        if not ret:
            logging.error("Unexpected error accessing the camera frame :(")
            return False

        # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(frame_HSV, np.array([110, 30, 150], np.uint8), np.array([130, 160, 255], np.uint8))  # Office
        mask = cv2.inRange(frame_HSV, np.array([120, 40, 50], np.uint8), np.array([150, 160, 255], np.uint8))  # Bedroom
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=10)
        mask3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        keypoints = cv2.SimpleBlobDetector_create(self.DRONE_DETECTOR_PARAMS).detect(mask)
        if keypoints:
            keypoint = max(keypoints, key=attrgetter('size'))   # Keep only the biggest blob
            mask_with_keypoints = cv2.drawKeypoints(mask, [keypoint], np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            mask_with_keypoints = mask3
            keypoint = type('', (), {'pt': np.array([self.m_PID_roll.getCurrPos(), self.m_PID_thrust.getCurrPos()]) +
                                           np.array([self.m_PID_roll.getCurrVel(), -self.m_PID_thrust.getCurrVel()])*(t-self.m_PID_thrust.PIDvel.curr_time).total_seconds(), 'size': 40})  # Since PIDs haven't been updated with current values yet, don't have to multiply velocity by (curr_time-last_time) but rather by (t-curr_time)
            ## keypoint = type('', (), {'pt': self.m_drone_curr_pos + self.m_drone_curr_vel*(t-self.m_t_last_frame).total_seconds(), 'size': 40})

        self.m_drone_pos_tracked = bool(keypoints)
        ##self.m_drone_last_pos = self.m_drone_curr_pos  # Save current position as "last position"
        drone_curr_pos = np.array(keypoint.pt)  # And update current position based on the CV algorithm output
        drone_target_pos = tuple(int(round(x)) for x in [self.m_PID_roll.getSetPoint(), self.m_PID_thrust.getSetPoint()])
        ## self.m_drone_curr_vel = np.array([max(min(x, 50), -50)/(t-self.m_t_last_frame).total_seconds() for x in (self.m_drone_curr_pos - self.m_drone_last_pos)])  # Limit max speed (we know it's not faster than 50 px/frame
        cv2.circle(mask_with_keypoints, tuple(drone_curr_pos.astype(int)), int(round(keypoint.size/2)), self.COLOR_BALL_TRACKED if self.m_drone_pos_tracked else self.COLOR_BALL_UNTRACKED, -1)
        cv2.line(mask_with_keypoints, tuple(drone_curr_pos.astype(int)), drone_target_pos, self.COLOR_LINE_TRACKED if self.m_drone_pos_tracked else self.COLOR_LINE_UNTRACKED, 10)
        cv2.circle(mask_with_keypoints, drone_target_pos, 25, self.COLOR_TARGET_TRACKED if self.m_drone_pos_tracked else self.COLOR_TARGET_UNTRACKED, -1)  # Plot circle at desired drone location

        ## self.m_PID_roll.update(self.m_roll)
        self.m_PID_roll.update(drone_curr_pos[0], t)
        self.m_PID_pitch.update(self.m_pitch, t)
        self.m_PID_yaw.update(self.m_yaw, t)
        self.m_PID_thrust.update(drone_curr_pos[1], t)
        ## self.m_PID_thrust_pos.SetPoint = self.m_drone_target_pos[1]
        ## self.m_PID_thrust_vel.SetPoint = self.m_PID_thrust_pos.update(self.m_drone_curr_pos[1])
        ## self.m_PID_thrust_vel.update(-self.m_drone_curr_vel[1])

        if sendCommand:
            if self.m_bConnected:
                if self.m_bTakingOff:
                    self.m_CrazyFlie.commander.send_setpoint(self.m_PID_roll.PIDvel.out_offs, self.m_PID_pitch.output, self.m_PID_yaw.output, self.TAKEOFF_THRUST)
                else:
                    self.m_CrazyFlie.commander.send_setpoint(self.m_PID_roll.getOutput(), self.m_PID_pitch.output, self.m_PID_yaw.output, self.m_PID_thrust.getOutput())
            else:
                self.m_CrazyFlie.commander.send_setpoint(0, 0, 0, 0)
                self.m_PID_roll.clear(); self.m_PID_pitch.clear(); self.m_PID_yaw.clear(); self.m_PID_thrust.clear()  ## self.m_PID_thrust_pos.clear(); self.m_PID_thrust_vel.clear()

        formatNum = "{:+6.2f}"
        strPrint = ("ROLLp. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
                    "ROLLv. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
                    "PITCH. ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
                    "YAW..  ={:+3.0f};" + formatNum + " [" + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
                    "THRUSp={:3.0f};{:6.0f} [{:+6.0f}, {:+6.0f}, {:+6.0f}]\t\t" +
                    "THRUSv={:3.0f};{:6.0f} [{:+6.0f}, {:+6.0f}, {:+6.0f}]\t\t" +
                    "pos=[x:{:4.0f}, y:{:4.0f}, vx:{:4.0f}, vy:{:4.0f}, rpy: " + formatNum + "," + formatNum + "," + formatNum + "]\t\t" +
                    "@{} (FPS: {:5.2f}) - " + self.m_str_status).format(
            self.m_PID_roll.PIDpos.SetPoint, self.m_PID_roll.PIDpos.output, self.m_PID_roll.PIDpos.PTerm, self.m_PID_roll.PIDpos.Ki*self.m_PID_roll.PIDpos.ITerm, self.m_PID_roll.PIDpos.Kd*self.m_PID_roll.PIDpos.DTerm,
            self.m_PID_roll.PIDvel.SetPoint, self.m_PID_roll.PIDvel.output, self.m_PID_roll.PIDvel.PTerm, self.m_PID_roll.PIDvel.Ki*self.m_PID_roll.PIDvel.ITerm, self.m_PID_roll.PIDvel.Kd*self.m_PID_roll.PIDvel.DTerm,
            self.m_PID_pitch.SetPoint, self.m_PID_pitch.output, self.m_PID_pitch.PTerm, self.m_PID_pitch.Ki * self.m_PID_pitch.ITerm, self.m_PID_pitch.Kd * self.m_PID_pitch.DTerm,
            self.m_PID_yaw.SetPoint, self.m_PID_yaw.output, self.m_PID_yaw.PTerm, self.m_PID_yaw.Ki * self.m_PID_yaw.ITerm, self.m_PID_yaw.Kd * self.m_PID_yaw.DTerm,
            self.m_PID_thrust.PIDpos.SetPoint, self.m_PID_thrust.PIDpos.output, self.m_PID_thrust.PIDpos.PTerm, self.m_PID_thrust.PIDpos.Ki*self.m_PID_thrust.PIDpos.ITerm, self.m_PID_thrust.PIDpos.Kd*self.m_PID_thrust.PIDpos.DTerm,
            self.m_PID_thrust.PIDvel.SetPoint, self.m_PID_thrust.PIDvel.output, self.m_PID_thrust.PIDvel.PTerm, self.m_PID_thrust.PIDvel.Ki*self.m_PID_thrust.PIDvel.ITerm, self.m_PID_thrust.PIDvel.Kd*self.m_PID_thrust.PIDvel.DTerm,
            ## self.m_PID_thrust_pos.SetPoint, self.m_PID_thrust_pos.output, self.m_PID_thrust_pos.PTerm, self.m_PID_thrust_pos.Ki * self.m_PID_thrust_pos.ITerm, self.m_PID_thrust_pos.Kd * self.m_PID_thrust_pos.DTerm,
            ## self.m_PID_thrust_vel.SetPoint, self.m_PID_thrust_vel.output, self.m_PID_thrust_vel.PTerm, self.m_PID_thrust_vel.Ki * self.m_PID_thrust_vel.ITerm, self.m_PID_thrust_vel.Kd * self.m_PID_thrust_vel.DTerm,
            ## self.m_drone_curr_pos[0], self.m_drone_curr_pos[1], self.m_drone_curr_vel[0], -self.m_drone_curr_vel[1], self.m_roll, self.m_pitch, self.m_yaw,
            self.m_PID_roll.getCurrPos(), self.m_PID_thrust.getCurrPos(), self.m_PID_roll.getCurrVel(), self.m_PID_thrust.getCurrVel(), self.m_roll, self.m_pitch, self.m_yaw,
            str(t-self.m_t_start)[3:-3], 1./(t-self.m_t_last_frame).total_seconds())
        # print "Sent: \t" + strPrint
        strPrint = "          SP | SENT  [   P   ,   I   ,   D  ]\t\t" + strPrint

        self.m_log_control_data.update(self.m_PID_roll, self.m_PID_pitch, self.m_PID_yaw, self.m_PID_thrust)

        frame2 = np.vstack((frame, mask_with_keypoints))
        self.m_imshow_out = cv2.resize(frame2, None, fx=0.5, fy=0.5)
        np.putmask(frame, mask3==255, [0, 255, 0])

        out = cv2.resize(np.vstack((frame, mask_with_keypoints)), None, fx=0.5, fy=0.5)
        cnt = 0
        for s in strPrint.split('\t\t'):
            cv2.putText(out, s.replace('\t', '; '), (25, out.shape[0]+25*(cnt-1-strPrint.count('\t\t'))), cv2.FONT_HERSHEY_DUPLEX, 0.7, (200,200,200), 1, cv2.LINE_AA)
            cnt += 1
        cv2.imshow(self.FIGURE_NAME, out)
        if saveImg:
            cv2.imwrite(os.path.join(self.VIDEO_FOLDER, t.strftime("out_%H-%M-%S-%f.jpg")), out)

        # tt = datetime.now()
        keyCode = cv2.waitKey(1)
        # print "DeltaT = {:5.4f}s -> FPS: {:7.2f}\tTotal FPS: {:5.2f}".format((datetime.now()-tt).total_seconds(), 1./(datetime.now()-tt).total_seconds(), 1./(t-self.m_t_last_frame).total_seconds())

        if keyCode > 0:
            keyCode = chr(keyCode).lower()
            if keyCode == 'a':
                self.m_PID_roll.setSetPoint(self.m_PID_roll.getSetPoint() + 10)
            elif keyCode == 's':
                self.m_PID_pitch.SetPoint += 1
            elif keyCode == 'd':
                self.m_PID_roll.setSetPoint(self.m_PID_roll.getSetPoint() - 10)
            elif keyCode == 'w':
                self.m_PID_pitch.SetPoint -= 1
            elif keyCode == 'e':
                self.m_bTakingOff = False
                self.m_str_status = "FLYING"
                self.m_PID_roll.setSetPoint(self.m_PID_roll.getCurrPos())
                self.m_PID_thrust.setSetPoint(self.m_PID_thrust.getCurrPos())
                self.m_PID_roll.clear()
                self.m_PID_thrust.clear()
                ## self.m_drone_target_pos = self.m_drone_curr_pos.astype(int)
                ## self.m_PID_thrust.setSetPoint(self.m_drone_target_pos[1])
                ## self.m_PID_thrust_pos.clear(); self.m_PID_thrust_vel.clear()
            else:
                self.m_bConnected = False
                return datetime.now() + timedelta(seconds=2)

        tt = datetime.now()
        time.sleep(max(0, (1./self.m_desired_fps) - (datetime.now() - t).total_seconds() - 0.0045))
        print "DeltaT = {:5.4f}s -> FPS: {:7.2f}\tTotal FPS: {:5.2f}".format((datetime.now()-tt).total_seconds(), 1./(datetime.now()-t).total_seconds(), 1./(t-self.m_t_last_frame).total_seconds())
        self.m_t_last_frame = t

        return None

if __name__ == '__main__':
    sendCommands = True
    # sendCommands = False
    if sendCommands:
        Hover().run()
    else:
        h = Hover()
        h.init_video_cam()
        h.m_bConnected = True
        while h.hover(False, False) is None:
            # time.sleep(1/30.0)
            pass
