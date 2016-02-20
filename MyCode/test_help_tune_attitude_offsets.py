"""
    Old code used to test different offset values for roll, pitch, yaw and base(=hover) thrust. At the same time,
    it is used to test that the camera functionality (connecting to the camera, grabbing and displaying frames,
    writing text on images or saving images to the hard drive) and comms to the drone are working as expected.
"""

import logging
import os
import sys
from threading import Thread
import time
import cv2
from datetime import datetime, timedelta

sys.path.insert(0, "../Crazyflie client/lib")
from cflib.crazyflie import Crazyflie
from cflib import crtp
from cfclient.utils.logconfigreader import LogConfig  # noqa


class Hover:

    def __init__(self):
        self.m_bShuttingDown = False
        self.m_CrazyFlie = None
        self.m_bConnected = False
        self.m_roll = 0
        self.m_pitch = 0
        self.m_yaw = 0

    def Run(self):
        logging.basicConfig()
        logging.getLogger().setLevel(logging.INFO)

        logging.info("Initializing drivers.")
        crtp.init_drivers(enable_debug_driver=False)
        availableLinks = crtp.scan_interfaces()
        logging.info("Available links: %s" % availableLinks)
        logging.info("Initializing Crazyflie.")
        self.m_CrazyFlie = Crazyflie(ro_cache="cachero", rw_cache="cacherw")

        logging.info("Setting radio link.")
        if(len(availableLinks) == 0):
            logging.error("Error, no links.  Aborting.")
            return

        linkUri = availableLinks[0][0]

        self.m_CrazyFlie.connected.add_callback(self.OnConnected)
        self.m_CrazyFlie.open_link(linkUri)

        while not self.m_bConnected:
            time.sleep(0.5)

        try:
            self.m_log_stab = LogConfig(name="Stabilizer", period_in_ms=10)
            self.m_log_stab.add_variable("stabilizer.roll", "float")
            self.m_log_stab.add_variable("stabilizer.pitch", "float")
            self.m_log_stab.add_variable("stabilizer.yaw", "float")
            self.m_CrazyFlie.log.add_config(self.m_log_stab)
            self.m_log_stab.data_received_cb.add_callback(self.log_stabilizer_data)
            self.m_log_stab.start()
        except KeyError as e:
            raise Exception("Couldn't start log configuration, %s not found in TOC!" % str(e))
        except AttributeError:
            raise Exception("Couldn't add Stabilizer log config, bad configuration.")

        try:
            # while not self.m_bShuttingDown:
            #     if self.m_bConnected:
            #         self.hover(True)
            #     else:
            #         time.sleep(0.1)
            thread1 = Thread(target=self.hover, args=())
            thread1.start()
            self.show_cam()
        except Exception as e:
            logging.error("Shutting down due to an exception: %s." % str(e))
            self.m_bShuttingDown = True

        cv2.destroyAllWindows()
        self.m_CrazyFlie.commander.send_setpoint(0, 0, 0, 0)
        self.m_CrazyFlie.close_link()

    def OnConnected(self, linkUri):
        logging.info("OnConnectSetupFinished")
        self.m_bConnected = True

    def log_stabilizer_data(self, timestamp, data, logconf):
        logging.debug("[%d][%s]: %s" % (timestamp, logconf.name, data))
        self.m_roll = data['stabilizer.roll']
        self.m_pitch = data['stabilizer.pitch']
        self.m_yaw = data['stabilizer.yaw']
        # print "\r@t={}\troll: {:6.2f}\tpitch: {:6.2f}\tyaw: {:6.2f}\n".format(datetime.now().strftime("%H:%M:%S.%f")[:-3], self.m_roll, self.m_pitch, self.m_yaw),

    def hover(self):
        time.sleep(2)

        roll = 7
        pitch = 8
        yawrate = 0
        thrust = 43500
        for cnt in range(0, 20):
            if self.m_bShuttingDown: break  # Allow aborting take-off too (otherwise a key press would not stop the drone until take-off phase was completed)
            self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)

        print "AT t={}, DONE TAKING OFF!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])

        while not self.m_bShuttingDown:
            roll = 7
            pitch = 8
            yawrate = 0
            thrust = 43500
            self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)

        self.m_CrazyFlie.commander.send_setpoint(0, 0, 0, 0)
        print "AT t={}, COMPLETELY STOPPED THE DRONE!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])

    def show_cam(self):
        video_capture = cv2.VideoCapture(0)
        video_folder = "img"
        if video_capture.isOpened():
            print "Camera opened! :)"
            cv2.namedWindow('Video')
            cv2.moveWindow('Video', 200, 200)
            cv2.waitKey(500)

            # Prepare the folder self.m_video_folder so we can log each frame processed (for debugging)
            if os.path.exists(video_folder):
                for f in os.listdir(video_folder): os.remove(os.path.join(video_folder, f))
            else:
                os.makedirs(video_folder)
        else:
            print "Couldn't open camera! :("
            self.m_bShuttingDown = False

        tStop = None
        t = datetime.now()
        while (tStop is None) or (t < tStop):
            t = datetime.now()
            ret, frame = video_capture.read()
            if ret:
                cv2.putText(frame, "roll: {:+6.2f}; pitch: {:+6.2f}; yaw: {:+6.2f}".format(self.m_roll, self.m_pitch, self.m_yaw), (25, frame.shape[0]-50), cv2.FONT_HERSHEY_DUPLEX, 0.7, (20,20,200), 1, cv2.LINE_AA)
                cv2.putText(frame, "@t={}".format(t.strftime("%H:%M:%S.%f")[:-3]), (25, frame.shape[0]-25), cv2.FONT_HERSHEY_DUPLEX, 0.7, (20,20,200), 1, cv2.LINE_AA)
                cv2.imshow('Video', frame)
                cv2.imwrite(os.path.join(video_folder, t.strftime("out_%H-%M-%S-%f.jpg")), frame)
                print "@t={}; roll: {:+6.2f}; pitch: {:+6.2f}; yaw: {:+6.2f}; New camera frame :D".format(t.strftime("%H:%M:%S.%f")[:-3], self.m_roll, self.m_pitch, self.m_yaw)
            if cv2.waitKey(1) > 0:
                tStop = t + timedelta(seconds=2)
                self.m_bShuttingDown = True
                print "AT t={}, A KEY WAS PRESSED -> STOPPING!".format(t.strftime("%H:%M:%S.%f")[:-3])

if __name__ == "__main__":
    Hover().Run()
