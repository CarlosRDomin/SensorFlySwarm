#!/bin/bash
workon sensorflyswarm
python configure.py
make -j8
sudo make install