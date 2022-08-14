# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/max/navi_ws/devel;/home/max/path_plan_ws/devel;/home/max/workspace/catkin_ws_livom/devel;/home/max/workspace/catkin_ws_sc_lio/devel;/home/max/workspace/catkin_ws_lio/devel;/home/max/workspace/catkin_ws_lins/devel;/home/max/workspace/catkin_ws_ov/devel;/home/max/eskf_ws/devel;/home/max/msckf_ws/devel;/home/max/vins_ws/devel;/home/max/imu_allan_ws/devel;/home/max/kalibr_allan_ws/devel;/home/max/vio_sim_ws/devel;/home/max/catkin_ws/devel;/home/max/kalibr_workspace/devel;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/max/path_plan_ws/devel/env.sh')

output_filename = '/home/max/path_plan_ws/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
