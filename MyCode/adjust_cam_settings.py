import uvc
import cv2
import numpy as np
from datetime import datetime
import os
import sys
from uvc_capture import UvcCapture
from PyQt5.QtWidgets import QApplication, QSpinBox, QSpacerItem, QSizePolicy, QCheckBox, QAbstractSpinBox, QPushButton, QFileDialog, \
    QMessageBox
from PyQt5.QtGui import QImage, QPixmap, QResizeEvent
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot, QSize, QObject, QEvent
from PyQt5.uic import loadUi
# import logging
# logging.basicConfig(level=logging.DEBUG)


class CamDeviceListRefresher(QThread):

    sig_new_dev = pyqtSignal()

    def __init__(self):
        QThread.__init__(self)
        self.done = False
        self.dev_list = []

    def __del__(self):
        self.wait()

    @pyqtSlot()
    def stop_thread(self):
        self.done = True
        if not self.wait(5000):  # Should never happen, but just in case ;)
            print "Uh-ooohh, CamDeviceListRefresher didn't terminate even after waiting for 5sec. Force quitting :S"
            self.terminate()
        else:
            print "CamDeviceListRefresher exited cleanly :)"

    def run(self):
        while not self.done:
            print "Scanning for new devices..."
            dev_list = uvc.device_list()  # Find available devices and print the list
            if self.dev_list != dev_list:
                self.dev_list = dev_list
                self.sig_new_dev.emit()
                print "Compatible devices found:\n\t" + str(self.dev_list)
            self.sleep(2)


class CamFrameGrabberThread(QThread):

    DEFAULT_CAM = "USB 2.0 Camera"
    # DESIRED_CAM = "FaceTime HD Camera (Built-in)"
    DEFAULT_FPS = 60
    FPS_ESTIMATION_ALPHA = 0.98
    FRAME_SIZE_STR_SPLITTER = " x "
    VIDEO_FOLDER = "img_test"

    sig_update_cam_ui = pyqtSignal()
    sig_new_dev_selected = pyqtSignal(str)
    sig_new_img = pyqtSignal()
    sig_error = pyqtSignal(str)

    def __init__(self):
        QThread.__init__(self)
        self.done = False
        self.dev_selected_name = self.DEFAULT_CAM
        self.cap = None
        self.qPix = QPixmap(1, 1)
        self.index = 0
        self.actual_fps = 0

    def __del__(self):
        self.wait()

    @pyqtSlot()
    def stop_thread(self):
        self.done = True  # Flag to indicate the outer while loop in run() to finish thread execution
        self.change_selected_device("EXIT! :P")  # Choose a void device, to force the inner while loop in run() to exit after grabbing at most one frame
        if self.wait(5000) != True or self.cap is not None:  # Worst-case scenario: force quit if self.cap is still not None after 5sec
            print "Uh-ooohh, CamFrameGrabberThread didn't terminate even after waiting for 5sec. Force quitting :S"
            self.cap = None
            self.terminate()
        else:
            print "CamFrameGrabberThread exited cleanly :)"

    @pyqtSlot(str)
    def change_selected_device(self, dev_selected):
        if self.dev_selected_name != dev_selected: print "New device selected: {}".format(dev_selected)
        self.dev_selected_name = dev_selected

    @pyqtSlot(str, bool)
    def change_cam_frame_size(self, new_frame_size, emit_signal):
        if self.cap is not None and window.drpSize.count() > 0:  # Sanity check (in case capture device was just closed or we're clearing drpSize to re-add available sizes)
            print "Changing frame size to {}".format(new_frame_size)
            self.cap.frame_size = tuple([int(x) for x in new_frame_size.split(self.FRAME_SIZE_STR_SPLITTER)])
            if emit_signal:  # If the user was the one who clicked on this combobox item (as opposed to me, manually from the code), update cam UI
                self.sig_update_cam_ui.emit()  # drpFPS might need to be updated (different frame sizes might have different fps available)

    @pyqtSlot(str)
    def change_cam_frame_rate(self, new_frame_rate):
        if self.cap is not None and window.drpFPS.count() > 0:  # Sanity check (in case capture device was just closed or we're clearing drpSize to re-add available sizes)
            print "Changing frame rate to {}".format(new_frame_rate)
            self.cap.frame_rate = int("".join(x for x in new_frame_rate if x.isdigit()))  # Keep only the numbers (= remove the " fps" part, if exists)

    @pyqtSlot(str, int, bool)
    def change_cam_control_setting(self, ctrl_name, new_value, is_bool):
        if self.cap is not None:  # Sanity check (in case capture device was just closed)
            for c in self.cap.controls:
                if c.display_name == ctrl_name:
                    try:
                        if is_bool:
                            c.set_value(c.min_val if (new_value == Qt.Unchecked) else c.max_val)
                        else:
                            c.set_value(new_value)
                        print "'{}' changed to {}".format(c.display_name, c.value)
                    except:
                        self.sig_error.emit("Unable to change '{}' property to '{}'! Make sure the value is valid (within bounds, not disabled by other setting like 'Auto' modes, etc).".format(ctrl_name, new_value))
                    return

    def run(self):
        while not self.done:
            self.sleep(1)

            self.cap = UvcCapture(self.dev_selected_name)
            if self.cap is None:  # If we didn't find the desired cam, don't continue
                self.qPix = QPixmap(1, 1)
                print "No compatible cameras found or chosen camera name not available :("
                self.sig_update_cam_ui.emit()
                continue

            for c in self.cap.controls:  # Adjust some cam settings and print cam info
                print "{} = {} {} (def:{}, min:{}, max:{})".format(c.display_name, str(c.value), str(c.unit), str(c.def_val), str(c.min_val), str(c.max_val))
            self.cap.select_best_frame_mode(self.DEFAULT_FPS)
            print self.cap.name + " has the following available modes:\n\t" + str([tuple(x) for x in self.cap.sorted_available_modes()]) + "\nSelected mode: " + str(self.cap.frame_mode)
            self.cap.print_info()
            # print "LOADING SETTINGS returned {}".format(self.cap.load_settings("/Users/Carlitos/Dropbox/UNI CARLOS/Grad school/Research/SensorFlySwarm/MyCode/UVCcam settings - USB 2.0 Camera.txt"))
            self.sig_update_cam_ui.emit()

            img = np.zeros(self.cap.frame_size)
            self.qPix = QPixmap.fromImage(QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888))

            if os.path.exists(self.VIDEO_FOLDER):
                for f in os.listdir(self.VIDEO_FOLDER): os.remove(os.path.join(self.VIDEO_FOLDER, f))
            else:
                os.makedirs(self.VIDEO_FOLDER)

            self.actual_fps = self.cap.frame_rate
            tt = datetime.now()
            while self.cap.name == self.dev_selected_name:
                ok = False
                for a in range(5):
                    try:
                        frame = self.cap.get_frame_robust()
                    except uvc.CaptureError as e:
                        print 'DEBUG - Could not get Frame. Error: "%s". Tried %s time(s).'%(e.message,a+1)
                    else:
                        # print 'DEBUG - Successfully got frame {:d} after {:d} tries'.format(frame.index, a+1)
                        ok = True
                        break
                if not ok:
                    self.sig_error.emit("Couldn't get camera frame after 5 tries, reconnecting...")
                    break

                t = datetime.now()
                self.index = frame.index
                img = cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2RGB)
                self.qPix.convertFromImage(QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888))
                self.sig_new_img.emit()
                t2 = datetime.now()
                # cv2.imwrite(os.path.join(VIDEO_FOLDER, "test_" + str(frame.index) + ".jpg"), cv2.resize(frame.img, None, fx=0.5, fy=0.5))

                self.actual_fps = self.FPS_ESTIMATION_ALPHA*self.actual_fps + (1-self.FPS_ESTIMATION_ALPHA)/(datetime.now()-tt).total_seconds()
                print "At t={} ({}) picture #{:03d} was taken; deltaTtotal={:6.2f}ms (toQPixmap->{:6.2f}ms + get_frame->{:6.2f}ms) -> Estimated FPS: {:.2f}".format(t, frame.timestamp, frame.index, (datetime.now()-tt).total_seconds()*1000, (t2-t).total_seconds()*1000, (t-tt).total_seconds()*1000, self.actual_fps)
                tt = datetime.now()

            self.cap = None


class FilterIgnoreScroll(QObject):

    sig_scroll = pyqtSignal()

    def eventFilter(self,  obj,  event):
        if event.type() == QEvent.Wheel:
            self.sig_scroll.emit()  # In case someone is interested in catching the ingnored scroll event, emit a signal
            event.ignore()
            return True
        return super(FilterIgnoreScroll, self).eventFilter(obj, event)


class FilterNotifyWindowCloses(QObject):

    sig_close = pyqtSignal()

    def __init__(self, parent):
        super(FilterNotifyWindowCloses, self).__init__(parent)
        self.sig_close.connect(threadFrameGrabber.stop_thread)
        self.sig_close.connect(threadDevLister.stop_thread)

    def eventFilter(self,  obj,  event):
        if event.type() == QEvent.Close:
            self.sig_close.emit()
            return True
        return super(FilterNotifyWindowCloses, self).eventFilter(obj, event)


def remove_all_layout_children(layout):
    if layout is None: return  # Nothing to remove!

    while layout.count():
        item = layout.takeAt(0)  # Get first item and remove it from the layout
        widget = item.widget()
        if widget is not None:  # Then, delete the actual widget
            widget.deleteLater()
        else:  # Unless it is a nested layout, in which case recursively delete all its children
            remove_all_layout_children(item.layout())

@pyqtSlot(str)
def show_error_message(str_msg):
    print "Showing error message: '{}'".format(str_msg)
    QMessageBox.critical(window, "ERROR! :(", str_msg)

@pyqtSlot()
def save_current_settings():
    fileName, extension_filter = QFileDialog.getSaveFileName(window, "Save current camera settings to a file", "UVCcam settings - {}".format(threadFrameGrabber.dev_selected_name), "UVC camera settings (*.txt)")
    if fileName:
        if threadFrameGrabber.cap is not None:
            if threadFrameGrabber.cap.save_settings(fileName):
                QMessageBox.about(window, "Current settings", "Congratulations, {}'s current settings have been successfully saved!".format(threadFrameGrabber.dev_selected_name))
            else:
                QMessageBox.about(window, "Current settings", "Unfortunately, there was an error while attempting to save {}'s current settings. Try again later or read the log for more details.".format(threadFrameGrabber.dev_selected_name))
    else:
        print "User canceled saving the settings"

@pyqtSlot(QResizeEvent)
def on_resize(event):
    # print "Window resized!! old size: {}; new size: {}".format((event.oldSize().width(),(event.oldSize().height())), (event.size().width(),(event.size().height())))
    window.lblCamImg.setPixmap(threadFrameGrabber.qPix.scaled(window.lblCamImg.size(), Qt.KeepAspectRatio))  # Resize pixmap inside label to fit new size

@pyqtSlot()
def repaint_cam_img():
    t = datetime.now()
    window.lblCamImg.setPixmap(threadFrameGrabber.qPix.scaled(window.lblCamImg.size(), Qt.KeepAspectRatio))
    # window.lblCamImg.update()
    # window.lblCamImg.repaint()
    window.setWindowTitle("Frame {} - FPS = {:.3f}".format(threadFrameGrabber.index, threadFrameGrabber.actual_fps))
    # print "\tUpdating image (#{:d}) at t={} ({}ms)".format(threadFrameGrabber.index, str(datetime.now().time())[:-3], (datetime.now() - t).total_seconds() * 1000)

@pyqtSlot()
def refresh_cam_device_list():
    old_dev_selected = window.drpInput.currentText()
    window.drpInput.clear()
    window.drpInput.addItem("None")  # Add a 'None' item so user can turn off camera
    for dev in threadDevLister.dev_list:
        window.drpInput.addItem(dev['name'])
        if dev['name'] == old_dev_selected:
            window.drpInput.setCurrentIndex(window.drpInput.count()-1)
    threadFrameGrabber.sig_new_dev_selected.emit(window.drpInput.currentText())

@pyqtSlot()
def update_cam_ui():
    window.drpSize.blockSignals(True)  # First, block frame size and FPS signals, so we don't receive changeIndex events while adding/removing items
    window.drpFPS.blockSignals(True)
    window.drpSize.clear()  # Then, clear frame size and FPS comboboxes, as well as remove all items from the settings tab
    window.drpFPS.clear()
    remove_all_layout_children(window.layoutGrpSettings)
    if threadFrameGrabber.cap is None:
        window.drpSize.addItem("N/A")  # No need to reenable signals, since it would be useless to get notified that user selected option "N/A"
        window.drpFPS.addItem("N/A")
        window.layoutGrpSettings.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding), 0, 0, 1, 1)  # Adding this spacer forces grpSettings to resize, otherwise it would look empty but a vertical scrollbar would show up
        return  # Only continue if a valid camera was selected

    frame_qsize = QSize(threadFrameGrabber.cap.frame_size[0], threadFrameGrabber.cap.frame_size[1])
    new_win_size = window.size() - window.lblCamImg.size() + frame_qsize
    window.setMaximumSize(new_win_size)
    window.setMinimumSize(new_win_size - frame_qsize + frame_qsize/2)
    window.resize(new_win_size)
    window.move((app.desktop().screenGeometry().width()-window.width())/2, (app.desktop().screenGeometry().height()-window.height())/2)
    print "Changed UI pic size to {}".format(threadFrameGrabber.cap.frame_size)

    for s in threadFrameGrabber.cap.frame_sizes:
        window.drpSize.addItem("{}{}{}".format(s[0], threadFrameGrabber.FRAME_SIZE_STR_SPLITTER, s[1]))
        if s == threadFrameGrabber.cap.frame_size:
            window.drpSize.setCurrentIndex(window.drpSize.count()-1)
    if window.drpSize.currentIndex() < 0:
        window.drpSize.setCurrentIndex(0)  # If no matching value found, choose the first one
        threadFrameGrabber.change_cam_frame_size(window.drpSize.currentText(), False)
    window.drpSize.blockSignals(False)  # Reenable signals for drpSize, as no more items will be added/removed nor currentIndex will be changed

    for f in threadFrameGrabber.cap.frame_rates:
        window.drpFPS.addItem("{} fps".format(f))
        if f == threadFrameGrabber.cap.frame_rate:
            window.drpFPS.setCurrentIndex(window.drpFPS.count()-1)
    window.drpFPS.blockSignals(False)  # Reenable signals for drpFPS too, as no more items will be added/removed nor currentIndex will be changed
    if window.drpFPS.currentIndex() < 0: window.drpSize.setCurrentIndex(0)  # If no matching value found, choose the first one

    i = 0
    for c in threadFrameGrabber.cap.controls:
        if c.max_val - c.min_val == 1:
            ctrl = QCheckBox(window.grpSettings)
            ctrl.setObjectName("chk_camSettings_{}".format(i+1))
            ctrl.setText(c.display_name)
            ctrl.setChecked(c.value == c.max_val)
            ctrl.stateChanged.connect(lambda v, n=c.display_name: threadFrameGrabber.change_cam_control_setting(n, v, True))
        else:
            ctrl = QSpinBox(window.grpSettings)
            ctrl.setObjectName("spin_camSettings_{}".format(i+1))
            ctrl.setAccelerated(True)
            ctrl.setCorrectionMode(QAbstractSpinBox.CorrectToNearestValue)
            ctrl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
            ctrl.setFocusPolicy(Qt.StrongFocus)
            ctrl.installEventFilter(FilterIgnoreScroll(ctrl))

            ctrl.setPrefix("{}: ".format(c.display_name))
            ctrl.setMaximum(c.max_val)
            ctrl.setMinimum(c.min_val)
            ctrl.setValue(c.value)
            ctrl.setSingleStep(max(1, (c.max_val-c.min_val)/25))
            ctrl.valueChanged.connect(lambda v, n=c.display_name: threadFrameGrabber.change_cam_control_setting(n, v, False))

        ctrl.setStatusTip("{}. Default value: {}. Valid range: [{}, {}]. Units: {}".format(c.display_name, c.def_val, c.min_val, c.max_val, c.unit))
        window.layoutGrpSettings.addWidget(ctrl, i, 0, 1, 1)
        i += 1
    btn_save_settings = QPushButton(window.grpSettings)
    btn_save_settings.setObjectName("btn_save_settings")
    btn_save_settings.setText("Save current settings")
    btn_save_settings.clicked.connect(save_current_settings)
    window.layoutGrpSettings.addWidget(btn_save_settings, i, 0, 1, 1)
    window.layoutGrpSettings.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding), i+1, 0, 1, 1)

app = QApplication(sys.argv)
threadDevLister = CamDeviceListRefresher()  # Initialize all variables first, in case other methods need to access them before .start() is called
threadFrameGrabber = CamFrameGrabberThread()
window = loadUi('adjust_cam_settings.ui')
window.resizeEvent = on_resize
window.installEventFilter(FilterNotifyWindowCloses(window))
window.drpInput.currentIndexChanged[str].connect(threadFrameGrabber.change_selected_device)
window.drpSize.currentIndexChanged[str].connect(lambda x: threadFrameGrabber.change_cam_frame_size(x, True))
window.drpFPS.currentIndexChanged[str].connect(threadFrameGrabber.change_cam_frame_rate)
window.drpInput.installEventFilter(FilterIgnoreScroll(window.drpInput))
window.drpSize.installEventFilter(FilterIgnoreScroll(window.drpSize))
window.drpFPS.installEventFilter(FilterIgnoreScroll(window.drpFPS))
refresh_cam_device_list()  # Add item "None" to drpInput and select it...
update_cam_ui()  # Also remove all items from grpSettings

threadDevLister.sig_new_dev.connect(refresh_cam_device_list)
# threadFrameGrabber.finished.connect(done)
threadFrameGrabber.sig_update_cam_ui.connect(update_cam_ui)
threadFrameGrabber.sig_new_dev_selected.connect(threadFrameGrabber.change_selected_device)
# threadFrameGrabber.sig_new_img.connect(change_img)
threadFrameGrabber.sig_error.connect(show_error_message)
threadDevLister.start()
threadFrameGrabber.start()

timer_repaint = QTimer()
timer_repaint.timeout.connect(repaint_cam_img)
timer_repaint.start()  # Delay of 0ms = Execute as soon as the event loop is done processing

window.show()
window.raise_()

sys.exit(app.exec_())
