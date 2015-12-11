#!/bin/bash
workon sensorflyswarm
python configure.py --qmake=/Library/MyBuilds/Qt5/5.5/clang_64/bin/qmake
sudo make -j8 install