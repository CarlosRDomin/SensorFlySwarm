#!/bin/bash
workon sensorflyswarm
python configure.py --qmake=/Users/Carlitos/Qt5.5.1/5.5/clang_64/bin/qmake
make -j8
sudo make install