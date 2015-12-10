import logging
import sys
from threading import Thread
import time
import cv2

sys.path.insert(0, "../Crazyflie client/lib")
from cflib.crazyflie import Crazyflie
from cflib import crtp
from cfclient.utils.logconfigreader import LogConfig  # noqa


class Hover:

    def __init__(self):
        self.m_bShuttingDown = False
        self.m_CrazyFlie = None
        self.m_bConnected = False

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

        try:
            while not self.m_bShuttingDown:
                if self.m_bConnected:
                    self.Hover(True)
                else:
                    time.sleep(0.1)
        except:
            logging.error("Shutting down due to exception.")
            self.m_bShuttingDown = True
            self.m_CrazyFlie.close_link()

    def OnConnected(self, linkUri):
        logging.info("OnConnectSetupFinished")
        self.m_bConnected = True

    def Hover(self, showCam):
        if showCam:
            video_capture = cv2.VideoCapture(0)
            if video_capture.isOpened():
                print "Camera opened! :)"
            else:
                print "Couldn't open camera! :("
                showCam = False

        roll = 15
        pitch = 25
        yawrate = 0
        thrust = 38000
        self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(1)
        self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(1)
        while not self.m_bShuttingDown:
            roll = 6
            pitch = 5
            yawrate = 0
            thrust = 38000
            self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)

            if showCam:
                ret, frame = video_capture.read()
                if ret:
                    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    cv2.imshow('Video', frame_HSV[:,:,1])
                cv2.waitKey(1)
                print "New camera frame :D"

    # def ShowLivestream(self, numFrames):
    #     numFrames = 100
    #     video_capture = cv2.VideoCapture(0)
    #     if video_capture.isOpened():
    #         print "Camera opened! :)"
    #         for i in range(0,numFrames):
    #             ret, frame = video_capture.read()
    #             if ret: cv2.imshow('Video', frame)
    #             cv2.waitKey(25)
    #             print i+1
    #         video_capture.release()
    #     else:
    #         print "Couldn't open camera! :("


Hover().Run()