import uvc
import numpy as np
import logging


class UvcCapture(uvc.Capture):

    SETTINGS_SEPARATOR = '\t'  # String used to separate setting name from setting value (don't use ',' as some setting names might contain commas)
    logging.basicConfig(level=logging.DEBUG)

    def __new__(cls, cam_name, *more):
        cam_uid = ""
        if isinstance(cam_name, str):
            for d in uvc.device_list():  # From the available devices list, find the one we want
                if d['name'] == cam_name:
                    cam_uid = d['uid']
                    break
        elif isinstance(cam_name, int):
            if cam_name < len(uvc.device_list()):
                cam_uid = uvc.device_list()[cam_name]['uid']

        if cam_uid == "":
            logging.warning("Couldn't find camera '{}' in the list of available UVC devices, aborting creation of {} object.".format(cam_name, UvcCapture))
            return None

        obj = super(UvcCapture, cls).__new__(cls, more)
        super(UvcCapture, obj).__init__(cam_uid)
        return obj

    def __init__(self, *more):  # Initialization of underlying uvc.Capture object was already done in __new__
        pass  # We still need to explicitly add this method or else the super.__init__ method will be called (and it'll try to open a camera with uid=cam_name, so it'll obviously fail)

    def save_settings(self, file_name):
        try:
            logging.debug("Saving current UVC camera settings to file '{}'".format(file_name))
            with open(file_name, 'w') as f:
                f.write("{},{}{}{}\n".format(self.frame_size[0], self.frame_size[1], self.SETTINGS_SEPARATOR, self.frame_rate))
                for c in self.controls:
                    f.write("{}{}{}\n".format(c.display_name, self.SETTINGS_SEPARATOR, c.value))
        except:
            logging.exception("Something went wrong while saving current UVCcam settings to '{}'.".format(file_name))
            return False
        return True

    def load_settings(self, file_name):
        try:
            logging.debug("Loading UVC camera settings from file '{}'".format(file_name))
            with open(file_name, 'r') as f:
                size, fps = f.readline().split(self.SETTINGS_SEPARATOR)
                self.frame_size = tuple([int(x) for x in size.split(',')])
                self.frame_rate = int(fps)
                logging.debug("\tLoaded camera settings: frame_size = {}x{}, frame_rate = {} fps".format(self.frame_size[0], self.frame_size[1], self.frame_rate))
                for line in f:
                    name, value = line.split(self.SETTINGS_SEPARATOR)
                    for c in self.controls:
                        if c.display_name == name:
                            c.value = int(value)
                            logging.debug("\tLoaded control setting: '{}' = {}".format(c.display_name, c.value))
                            break
        except:
            logging.exception("Something went wrong while loading UVCcam settings from '{}'.".format(file_name))
            return False
        return True

    def sorted_available_modes(self):
        return np.array(sorted(self.avaible_modes, reverse=True))  # Sort available modes by decreasing frame width, height and fps (in that order, in case there's a tie)

    def select_best_frame_mode(self, min_fps=1, min_width=1, min_height=1):
        avail_modes = self.sorted_available_modes()
        condition = (avail_modes[:,0] >= min_width) & (avail_modes[:,1] >= min_height) & (avail_modes[:,2] >= min_fps)
        try:
            self.frame_mode = tuple(avail_modes[np.argwhere(condition).ravel()[0]])  # Choose biggest size (because avail_modes is sorted) meeting criteria 'condition'
        except:  # In case no modes meet the criteria, choose the biggest size & highest frame rate
            self.frame_mode = tuple(avail_modes[0])
            logging.debug("No modes met the criteria of frame rate >= {}fps, width >= {}px, height >= {}px. Selected mode with biggest size and highest frame rate: {}".format(min_fps, min_width, min_height, self.frame_mode))
