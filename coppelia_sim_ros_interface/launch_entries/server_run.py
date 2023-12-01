#!/usr/bin/env python
''' >>> python coppelia_sim_ros_server.py '''
import os
path_from_ws = f'src/coppelia_sim_ros_interface/coppelia_sim_ros_interface'
file = 'coppelia_sim_ros_server.py'

from entry_template import run; run(path_from_ws, file)
