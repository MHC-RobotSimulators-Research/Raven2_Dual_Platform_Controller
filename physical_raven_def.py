"""
Raven II Dual Platform Controller: control software for the Raven II robot. Copyright © 2023-2024 Yun-Hsuan Su,
Natalie Chalfant, Mai Bui, Sean Fabrega, and the Mount Holyoke Intelligent Medical Robotics Laboratory.

This file is a part of Raven II Dual Platform Controller.

Raven II Dual Platform Controller is free software: you can redistribute it and/or modify it under the terms of the
GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

Raven II Dual Platform Controller is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with Raven II Dual Platform Controller.
If not, see <http://www.gnu.org/licenses/>.

physical_raven_def.py

date: May 13, 2024
author: Natalie Chalfant, Mai Bui, Sean Fabrega
"""

import math as m
import numpy as np
import sys
# import quaternion

# Copied from https://github.com/WPI-AIM/ambf/blob/restructure/ambf_controller/raven2/src/ambf_defines.cpp

# Defines variables used in raven_fk, raven_ik, etc.
RAVEN_TYPE = True

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

V            = -1
CAMERA_COUNT = 3
RAVEN_JOINTS = 7
RAVEN_ARMS   = 2
RAVEN_IKSOLS = 8
PUBLISH_RATE = 500
PUBLISH_TIME = 1 / PUBLISH_RATE
Eps          = sys.float_info.epsilon

MAX_JR             = np.array([5*Deg2Rad, 5*Deg2Rad, 0.02, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad], dtype="float") / PUBLISH_RATE
MAX_JOINTS         = np.array([90 * Deg2Rad, 135 * Deg2Rad, 0.56, m.pi, 2, (m.pi * 3)/4, (m.pi * 3)/4],  dtype = 'float')
MIN_JOINTS         = np.array([0, 45 * Deg2Rad, 0.23, -m.pi, -2, 0, 0],  dtype = 'float')
HOME_JOINTS        = np.array([m.pi / 6, m.pi / 2, 0.4, -m.pi/4, 0, m.pi / 4, m.pi / 4], dtype='float')
# HOME_JOINTS        = np.array([0.5217216610908508, 1.5799641609191895, 0.40040287375450134, 0.06240818649530411, -0.006050971802324057, 0.7633852362632751, 0.7971110939979553],dtype = 'float')
HOME_DH            = np.array([[0.52359878, 1.57079633, 0.4, -0.78539816, 0., 0., 0.78539816],
                               [0.52359878, 1.57079633, 0.4, -0.78539816, 0., -0., 0.78539816]], dtype="float")
DANCE_SCALE_JOINTS = np.array([0.3, 0.3, 0.06, 0.3, 1.2, m.pi/6, m.pi/6],  dtype='float')

RAVEN_JOINT_LIMITS = np.array([[0.0, m.pi/4, 0.23, -m.pi*2, -2, -2, -2],
                               [m.pi/2, (m.pi*3)/4, 0.56, m.pi*2, 2, 2, 2]])
RAVEN_DH_ALPHA     = np.array([[0, np.deg2rad(-75), np.deg2rad(128), 0, m.pi/2, m.pi/2, 0],
                               [m.pi, np.deg2rad(75), np.deg2rad(52), 0, m.pi/2, m.pi/2, 0]], dtype = 'float')
RAVEN_DH_THETA     = np.array([[V, V, m.pi/2, V, V, V, 0],
                               [V, V, -m.pi/2, V, V, V, 0]], dtype='float')
RAVEN_DH_A         = np.array([[0, 0, 0, 0, 0, 0.013, 0],
                               [0, 0, 0, 0, 0, 0.013, 0]], dtype='float')
RAVEN_DH_D         = np.array([[0, 0, V, -0.47, 0, 0, 0],
                               [0, 0, V, -0.47, 0, 0, 0]], dtype='float')

RAVEN_IKIN_PARAM   = np.array([float(m.sin(RAVEN_DH_ALPHA[0][1])),
                                float(m.cos(RAVEN_DH_ALPHA[0][1])),
                                float(m.sin(RAVEN_DH_ALPHA[1][2])),
                                float(m.cos(RAVEN_DH_ALPHA[1][2])),
                                RAVEN_DH_D[0][3],
                                RAVEN_DH_A[0][5]], dtype = 'float')
RAVEN_T_B0         = np.array([np.matrix([[0, 0,  1,  0.30071], [0, -1, 0, 0.061], [1, 0, 0, -0.007], [0, 0, 0, 1]], dtype = 'float'),
                               np.matrix([[0, 0, -1, -0.30071], [0,  1, 0, 0.061], [1, 0, 0, -0.007], [0, 0, 0, 1]], dtype = 'float')])
RAVEN_T_CB         = np.matrix([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0], [0, 0, 0, 1]], dtype='float') #using quaternion to rotational translator

Z_ROT = np.array([np.matrix([[m.cos(np.deg2rad(25)), -m.sin(np.deg2rad(25)), 0, 0], [m.sin(np.deg2rad(25)), m.cos(np.deg2rad(25)), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype='float'),
                  np.matrix([[m.cos(np.deg2rad(-25)), -m.sin(np.deg2rad(-25)), 0, 0], [m.sin(np.deg2rad(-25)), m.cos(np.deg2rad(-25)), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype='float')])

x_rot_val = 30
X_ROT = np.matrix([[1, 0, 0, 0],
                    [0, m.cos(np.deg2rad(x_rot_val)), m.sin(np.deg2rad(x_rot_val)), 0],
                    [0, -m.sin(np.deg2rad(x_rot_val)), m.cos(np.deg2rad(x_rot_val)), 0],
                    [0, 0, 0, 1]], dtype='float')

IDENTITY = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype='float')
# Raven joints:
# joint -: 0_link-base_link_L:             fixed
# joint 0: base_link_L-link1_L:            revolute        (shoulder)              range: -pi~pi
# joint 1: link1_L-link2_L:                revolute        (elbow)                 range: -pi~pi
# joint 2: link2_L-link3_L:                prismatic       (tool plate up/down)    range: -0.17~0.1
# joint 3: link3_L-instrument_shaft_L:     continuous      (tool shaft roll)       range: no limit
# joint 4: instrument_shaft_L-wrist_L:     revolute        (tool wrist)            range: -2~2
# joint 5: wrist_L-grasper1_L:             revolute        (grasper left)          range: -2~2
# joint 6: wrist_L-grasper2_L:             revolute        (grasper right)         range: -2~2

# Parameters for the file mode
COL_IN_FILE = 16              # the number of columns in the FROM FILE is 16
FROM_FILE = "ravenpath.csv"   # path to the csv file with desired raven trajectory
TO_FILE = "raven_state_record.csv"     # path to the csv file where the actual raven trajectory is saved to
RECORD_FLAG = False           # this flag determines whether ambf raven states are recorded