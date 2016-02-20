"""
    Simple script that provides constant thrust to the drone, and controls roll, pitch and yaw using independent
    PID loops, whose input is the drone's roll, pitch and yaw, measured from its accels&gyros and sent via radio.
    ASDW keys can be used to modify the PID set points, basically acting like a joystick/remote controller.
"""

import logging
import os
import sys
from threading import Thread
import time
import cv2
from datetime import datetime, timedelta
import PID

sys.path.insert(0, "../Crazyflie client/lib")
from cflib.crazyflie import Crazyflie
from cflib import crtp
from cfclient.utils.logconfigreader import LogConfig  # noqa


class Hover:

    def __init__(self):
        self.m_bShuttingDown = False
        self.m_CrazyFlie = None
        self.m_bConnected = False
        self.str_status = "TAKING OFF"
        self.m_roll = 0
        self.m_pitch = 0
        self.m_yaw = 0
        self.m_PID_roll = PID.PID(P=1, I=2, D=0.01, offs=0, out_max=20)
        self.m_PID_pitch = PID.PID(P=1, I=2, D=0.01, offs=0, out_max=20, invert_error=True)
        self.m_PID_yaw = PID.PID(P=0.5, I=0.3, D=0.01, offs=0, out_max=20, invert_error=True, error_in_degrees=True)

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
            return
        except AttributeError:
            raise Exception("Couldn't add Stabilizer log config, bad configuration.")
            return

        try:
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
        # self.m_PID_roll.update(self.m_roll)
        # self.m_PID_pitch.update(self.m_pitch)
        # self.m_PID_yaw.update(self.m_yaw)
        # print "\r@t={}\troll: {:6.2f}\tpitch: {:6.2f}\tyaw: {:6.2f}\n".format(datetime.now().strftime("%H:%M:%S.%f")[:-3], self.m_roll, self.m_pitch, self.m_yaw),

    def hover(self):
        time.sleep(2)

        roll = 8
        pitch = 10
        yawrate = 0
        thrust = 45000
        for cnt in range(0, 10):
            if self.m_bShuttingDown: break  # Allow aborting take-off too (otherwise a key press would not stop the drone until take-off phase was completed)
            self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)

        print "AT t={}, DONE TAKING OFF!".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])
        self.str_status = "FLYING (yaw = {:+6.2f})".format(self.m_yaw)
        self.m_PID_roll.clear()
        self.m_PID_pitch.clear()
        self.m_PID_yaw.clear()
        self.m_PID_roll.SetPoint = 0
        self.m_PID_pitch.SetPoint = 0
        self.m_PID_yaw.SetPoint = self.m_yaw

        while not self.m_bShuttingDown:
            self.m_PID_roll.update(self.m_roll)
            self.m_PID_pitch.update(self.m_pitch)
            self.m_PID_yaw.update(self.m_yaw)
            roll = self.m_PID_roll.output
            pitch = self.m_PID_pitch.output
            yawrate = self.m_PID_yaw.output
            thrust = 44750
            self.m_CrazyFlie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.05)

        self.m_PID_roll.output = 0
        self.m_PID_pitch.output = 0
        self.m_PID_yaw.output = 0
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
                strPrint = ("ROLL.. => SP:{:+3.0f}; Curr:{:+7.2f}; Sent:{:+6.2f} [P={:+6.2f}, I={:+6.2f}, D={:+6.2f}]\t\t" + \
                        "PITCH. => SP:{:+3.0f}; Curr:{:+7.2f}; Sent:{:+6.2f} [P={:+6.2f}, I={:+6.2f}, D={:+6.2f}]\t\t" + \
                        "YAW..  => SP:{:+3.0f}; Curr:{:+7.2f}; Sent:{:+6.2f} [P={:+6.2f}, I={:+6.2f}, D={:+6.2f}]\t\t" + \
                        "@{} - " + self.str_status).format(
                               self.m_PID_roll.SetPoint, self.m_roll, self.m_PID_roll.output, self.m_PID_roll.PTerm, self.m_PID_roll.Ki * self.m_PID_roll.ITerm, self.m_PID_roll.Kd * self.m_PID_roll.DTerm,
                               self.m_PID_pitch.SetPoint, self.m_pitch, self.m_PID_pitch.output, self.m_PID_pitch.PTerm, self.m_PID_pitch.Ki * self.m_PID_pitch.ITerm, self.m_PID_pitch.Kd * self.m_PID_pitch.DTerm,
                               self.m_PID_yaw.SetPoint, self.m_yaw, self.m_PID_yaw.output, self.m_PID_yaw.PTerm, self.m_PID_yaw.Ki * self.m_PID_yaw.ITerm, self.m_PID_yaw.Kd * self.m_PID_yaw.DTerm,
                               t.strftime("%H:%M:%S.%f")[:-3])
                cnt = 0
                for s in strPrint.split('\t\t'):
                    cv2.putText(frame, s, (25, frame.shape[0]+25*(cnt-1-strPrint.count('\t\t'))), cv2.FONT_HERSHEY_DUPLEX, 0.7, (20,20,200), 1, cv2.LINE_AA)
                    cnt += 1
                cv2.imshow('Video', frame)
                cv2.imwrite(os.path.join(video_folder, t.strftime("out_%H-%M-%S-%f.jpg")), frame)
                print strPrint
            keyCode = cv2.waitKey(1)
            if keyCode > 0:
                keyCode = chr(keyCode).lower()
                if keyCode == 'a':
                    self.m_PID_roll.SetPoint -= 1
                elif keyCode == 's':
                    self.m_PID_pitch.SetPoint += 1
                elif keyCode == 'd':
                    self.m_PID_roll.SetPoint += 1
                elif keyCode == 'w':
                    self.m_PID_pitch.SetPoint -= 1
                else:
                    tStop = t + timedelta(seconds=2)
                    self.m_bShuttingDown = True
                    print "AT t={}, A KEY WAS PRESSED -> STOPPING!".format(t.strftime("%H:%M:%S.%f")[:-3])
                    self.str_status = "STOPPED"

Hover().Run()