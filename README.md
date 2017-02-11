# SensorFly Swarm README

> This repository contains all the code used in our implementation of our Collaborative Localization approach for increasing the localization accuracy in drone swarms (especially useful in GPS-denied scenarios or when using resource-constrained drones).

## Requirements
I may have forgotten to include some of the requirements in this list, will update if necessary. Basic requirements are:
 - [Python 2.7](#python)
 - [OpenCV3](#opencv)
 - [Qt5](#qt5)
 - [UVC](#uvc)
 - [CrazyFlie Python library](#crazyflie-python-library)
 - [CrazyFlie Custom Firmware](#crazyflie-custom-firmware)

## Install
### Python
For Mac users, I find it easiest to install Python through [Homebrew](http://brew.sh/)
Just download the latest [Python 2.7 release](https://www.python.org/downloads/) and follow the instructions. As of Feb 4, 2017 I'm using Python 2.7.13 on Mac.

### OpenCV
Our vision algorithms rely on the well-known open-source vision library [OpenCV](http://docs.opencv.org/3.2.0/). As of Feb 4, 2017 I'm using OpenCV 3.2, and I installed it on my Mac using Homebrew with the following command (the only actual requirements are `--with-contrib --with-qt5`):
```sh
brew install opencv3 --c++11 --with-contrib --with-examples --with-ffmpeg --with-gphoto2 --with-gstreamer --with-jasper --with-java --with-jpeg-turbo --with-libdc1394 --with-openni2 --with-opengl --with-python3 --with-qt5 --with-tbb --with-vtk --with-nonfree
```
It may take up to 1h to compile and install depending on your computer. Google instructions if not on Mac, as I've never followed the procedure for a different OS.

### Qt5
Our UIs (user interfaces) are built on top of [Qt5](http://doc.qt.io/qt-5/qt5-intro.html), which is a cross-platform GUI tool. As of Dec 4, 2016 I'm using Qt 5.7. It can also be installed through Homebrew on Mac by typing:
```sh
brew install qt5
```
Note that this step may also take a long time.

### UVC
In order to have better control over our cameras, we use a [UVC](https://en.wikipedia.org/wiki/USB_video_device_class) library. In particular, we followed [pupil-labs/pyuvc](https://github.com/pupil-labs/pyuvc)'s implementation. Feel free to follow their instructions, although this is a summary extracted from several of their READMEs:
 - Install pupil-labs/libuvc:
```sh
git clone https://github.com/pupil-labs/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
```
 - Install libjpeg-turbo:
```sh
brew install libjpeg-turbo
```
 - Install cython and numpy:
```sh
pip install cython
brew install numpy
```
 - Finally, build and install *my own fork* of pupil-labs/pyuvc (CarlosRDomin/pyuvc):
```sh
git clone https://github.com/CarlosRDomin/pyuvc
cd pyuvc
python setup.py install
```

### CrazyFlie Python library
We use [CrazyFlie 2](https://www.bitcraze.io/crazyflie-2/) drones for our workers. The spotter uses CrazyFlie's open-source [Python library](https://github.com/bitcraze/crazyflie-lib-python) to communicate with them. Follow these steps (extracted from their README) to install `cflib`:
```sh
git clone https://github.com/bitcraze/crazyflie-lib-python
pip install -e path/to/cflib
```

### CrazyFlie Custom Firmware
In order to control the worker drones the way we want, we use our own custom firmware, which was forked from the original [CrazyFlie firmware](https://github.com/bitcraze/crazyflie-firmware).
 - Simply clone my fork: ```git clone --recursive https://github.com/CarlosRDomin/crazyflie-firmware```
 - Compile: ```make```
 - And if you have a CrazyRadio connected (this step might require you to install [CrazyFlie's Python Client](https://github.com/bitcraze/crazyflie-clients-python)), flash it (you first need to enter the CrazyFlie into bootloader, which is done by holding the Power button for 5sec, or until the 2 blue LEDs in the back of the CrazyFlie start toggling/blinking): ```make cload```

### Additional tools
I just realized `MyCode/full_control_with_cam.py` uses the `SDL2` library to handle user input through the keyboard. This means `SDL2` needs to be installed:
```sh
brew install sdl2
pip install PySDL2
```

## How-to
### Adjust camera settings
Run `MyCode/adjust_cam_settings.py`, and select the camera that wants to be tuned. The GUI will let you specify parameters such as exposure time, white balance, etc.
In addition, color thresholding and blob detector parameters can also be adjusted to find the vision marker (ie: ping pong ball) on the workers. Then, settings can be saved to a file once the right values have been set.
PS: For more info, read the description at the top of the file, the comments throughout the code, or just [email me](emailto:carlosrd@cmu.edu).

### Calibrate camera
You first need to print out a calibration pattern (`MyCode/cam_calibration/circles_pattern.pdf`) and tape it to a solid surface (furniture, floor, walls are ideal). Then run `MyCode/calibrate_cam_params.py`, which will open a live feed of the camera. Press Space to take images (make sure the calibration pattern is visible) from different angles and in different positions (up-left corner, down-right corner, center of the image, etc.). I usually collect 20-30 images for camera calibration. Once you're done taking images, press any key that's not Space (eg: Esc), and the code will generate calibration values for you and save them to a file (by default, it saves to `MyCode/cam_calibration/cam_calibration_output.npz`).
PS: For more info, read the description at the top of the file, the comments throughout the code, or just [email me](emailto:carlosrd@cmu.edu).

### Test vision algorithm before flying
Run `MyCode/calibrate_2d_to_3d.py`. It will look for the vision marker (ie: ping pong ball) using the HSV color thresholds obtained at the [Adjust camera settings](#adjust-camera-settings) step, and print on the live camera feed the estimated distance between the camera and the ball. If these results aren't accurate or consistent, try [retuning the camera settings](#adjust-camera-settings) or [recalibrating the camera](#calibrate-camera).
PS: For more info, read the description at the top of the file, the comments throughout the code, or just [email me](emailto:carlosrd@cmu.edu).

### Fly!
Once the vision algorithm has been successfully tested and we have a CrazyFlie 2 with our [custom firmware](#crazyflie-custom-firmware) installed, you can fly the worker by running `MyCode/full_control_with_cam.py`. This will fly the CrazyFlie up until keyboard letter `e` is pressed. Then, the worker will hover in place. Keyboard keys ASDW can be used to control left, down, right and up setpoints respectively. Keys `u` and `h` control closer and further to/from the camera, respectively. Any other key ends the flight.
PS: For more info, read the description at the top of the file, the comments throughout the code, or just [email me](emailto:carlosrd@cmu.edu).
