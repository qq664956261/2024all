#!/bin/bash

catkin_make_isolated --install --use-ninja

#catkin_make_isolated --install --use-ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes

catkin_make_isolated --install --use-ninja  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install_isolated/setup.bash
