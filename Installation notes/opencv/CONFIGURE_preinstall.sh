#!/bin/bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/opt/local \
-D PYTHON2_PACKAGES_PATH=~/.virtualenvs/sensorflyswarm/lib/python2.7/site-packages \
-D PYTHON2_LIBRARY=/opt/local/Library/Frameworks/Python.framework/Versions/2.7/bin \
-D PYTHON2_INCLUDE_DIR=/opt/local/Library/Frameworks/Python.framework/Headers \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D BUILD_EXAMPLES=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
..

# INSTRUCTIONS: SIMPLY CHANGE PYTHON2_PACKAGES_PATH (LINE 4) BASED ON WHERE YOU WANT TO INSTALL OPENCV