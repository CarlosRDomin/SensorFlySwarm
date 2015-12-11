#!/bin/bash
source virtualenvwrapper.sh
#workon sensorflyswarm
QMAKE_CMAKE_LIB_PATH="/Library/MyBuilds/Qt5/5.5/clang_64/lib/cmake"
cmake \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/opt/local \
-D PYTHON2_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; aux=get_python_lib(); print(aux)") \
-D PYTHON2_LIBRARY=$(python -c "from distutils.sysconfig import get_config_var; aux=get_config_var('BINDIR'); print(aux)") \
-D PYTHON2_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; aux=get_python_inc(); print(aux)") \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_TESTS=ON \
-D BUILD_EXAMPLES=ON \
-D WITH_QT=ON \
-D Qt5Concurrent_DIR="$QMAKE_CMAKE_LIB_PATH/Qt5Concurrent" \
-D Qt5Core_DIR="$QMAKE_CMAKE_LIB_PATH/Qt5Core" \
-D Qt5Gui_DIR="$QMAKE_CMAKE_LIB_PATH/Qt5Gui" \
-D Qt5OpenGL_DIR="$QMAKE_CMAKE_LIB_PATH/Qt5OpenGL" \
-D Qt5Test_DIR="$QMAKE_CMAKE_LIB_PATH/Qt5Test" \
-D Qt5Widgets_DIR="$QMAKE_CMAKE_LIB_PATH/Qt5Widgets" \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D BUILD_opencv_ximgproc=OFF \
..