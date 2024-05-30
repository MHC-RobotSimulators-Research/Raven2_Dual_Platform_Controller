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

raven_fk.py

date: May 13, 2024
author: Natalie Chalfant, Mai Bui, Sean Fabrega
"""

# from physical_raven_def import *
import math as m
import numpy as np
import ambf_raven_def as ard
import physical_raven_def as prd

def joint_to_dhvalue(joint, arm, raven_def):
    success = False
    dhvalue = np.zeros(7, dtype = 'float')
    if arm < 0 or arm >= raven_def.RAVEN_ARMS or joint.size != raven_def.RAVEN_JOINTS:
        return success
    for i in range(raven_def.RAVEN_JOINTS):
        if i != 2:
            if i == 5:
                if arm == 0:
                    dhvalue[i] =  (joint[i] - joint[i+1])

                else:
                    dhvalue[i] = -(joint[i] - joint[i+1])
            else:
                dhvalue[i] = joint[i]
                while dhvalue[i] > m.pi:
                    dhvalue[i] -= 2*m.pi
                while dhvalue[i] < -m.pi:
                    dhvalue[i] += 2*m.pi
        else:
            dhvalue[i] = joint[i]
    success = True

    return success, dhvalue

def fwd_trans(a, b, dh_alpha, dh_theta, dh_a, dh_d):
    if ((b <= a) or b == 0):
        ROS_ERROR("Invalid start/end indices")

    xx = float(m.cos(dh_theta[a]))
    xy = float(-m.sin(dh_theta[a]))
    xz = float(0)

    yx = float(m.sin(dh_theta[a])) * float(m.cos(dh_alpha[a]))
    yy = float(m.cos(dh_theta[a])) * float(m.cos(dh_alpha[a]))
    yz = float(-m.sin(dh_alpha[a]))

    zx = float(m.sin(dh_theta[a])) * float(m.sin(dh_alpha[a]))
    zy = float(m.cos(dh_theta[a])) * float(m.sin(dh_alpha[a]))
    zz = float(m.cos(dh_alpha[a]))

    px = float(dh_a[a])
    py = float(-m.sin(dh_alpha[a])) * float(dh_d[a])
    pz = float(m.cos(dh_alpha[a])) * float(dh_d[a])

    xf = np.matrix([[xx, xy, xz, px],
                    [yx, yy, yz, py],
                    [zx, zy, zz, pz],
                    [0, 0, 0, 1]])

    if b > a + 1:
        xf = np.matmul(xf, fwd_trans(a + 1, b, dh_alpha, dh_theta, dh_a, dh_d))

    return xf


def fwd_kinematics(arm, input_joint_pos, raven_def):
    success = False

    dh_alpha = np.zeros(7, dtype = 'float')
    dh_theta = np.zeros(7, dtype = 'float')
    dh_a = np.zeros(7, dtype = 'float')
    dh_d = np.zeros(7, dtype = 'float')

    j2d = joint_to_dhvalue(input_joint_pos, arm, raven_def)
    worked = j2d[0]
    jp_dh = j2d[1]

    if not worked:
        ROS_ERROR("Something went wrong with joint to dh conversion")
        return success
    for i in range(raven_def.RAVEN_JOINTS):
        if i == 2:
            dh_d[i] = jp_dh[i]
            dh_theta[i] = raven_def.RAVEN_DH_THETA[arm][i]
        else:
            dh_d[i] = raven_def.RAVEN_DH_D[arm][i]
            dh_theta[i] = jp_dh[i]
        dh_alpha[i] = raven_def.RAVEN_DH_ALPHA[arm][i]
        dh_a[i] = raven_def.RAVEN_DH_A[arm][i]

    if raven_def.RAVEN_TYPE:
        # output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_CB, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d)
        # output_transformation = np.matmul(np.matmul(raven_def.X_ROT, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(raven_def.Z_ROT[arm], fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_B0[arm], raven_def.Z_ROT[arm]), fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
    else:
        # output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_CB, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(raven_def.RAVEN_T_B0[arm], fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(np.matmul(raven_def.X_ROT, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(raven_def.RAVEN_T_B0[arm], fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_B0[arm], raven_def.Z_ROT[arm]), fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
    return output_transformation


def fwd_kinematics_p5(arm, input_joint_pos, raven_def):
    """
    Gets the position of joint 5 origin
    """
    success = False

    dh_alpha = np.zeros(7, dtype = 'float')
    dh_theta = np.zeros(7, dtype = 'float')
    dh_a = np.zeros(7, dtype = 'float')
    dh_d = np.zeros(7, dtype = 'float')

    j2d = joint_to_dhvalue(input_joint_pos, arm, raven_def)
    worked = j2d[0]
    jp_dh = j2d[1]

    if not worked:
        ROS_ERROR("Something went wrong with joint to dh conversion")
        return success
    for i in range(raven_def.RAVEN_JOINTS):
        if i == 2:
            dh_d[i] = jp_dh[i]
            dh_theta[i] = raven_def.RAVEN_DH_THETA[arm][i]
        else:
            dh_d[i] = raven_def.RAVEN_DH_D[arm][i]
            dh_theta[i] = jp_dh[i]
        dh_alpha[i] = raven_def.RAVEN_DH_ALPHA[arm][i]
        dh_a[i] = raven_def.RAVEN_DH_A[arm][i]

    if raven_def.RAVEN_TYPE:
        # output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_CB, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(raven_def.RAVEN_T_B0[arm], fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(np.matmul(raven_def.X_ROT, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(raven_def.Z_ROT[arm], fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_B0[arm], raven_def.Z_ROT[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))


    else:
        # output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_CB, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(raven_def.RAVEN_T_B0[arm], fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = np.matmul(np.matmul(raven_def.X_ROT, raven_def.RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
        # output_transformation = fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d)
        # output_transformation = np.matmul(raven_def.RAVEN_T_B0[arm], fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
        output_transformation = np.matmul(np.matmul(raven_def.RAVEN_T_B0[arm], raven_def.Z_ROT[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))

    return output_transformation

def main():


    # dh_vals_l = joint_to_dhvalue(ard.HOME_JOINTS, 1, ard)
    # print(dh_vals_l)
    # print(fwd_kinematics(0, prd.HOME_JOINTS, prd))
    # print(fwd_kinematics(1, prd.HOME_JOINTS, prd))
    print(fwd_kinematics_p5(0, prd.HOME_JOINTS, prd))
    print(fwd_kinematics_p5(1, prd.HOME_JOINTS, prd))
    # print(fwd_kinematics(0, ard.HOME_JOINTS, ard))
    # print(fwd_kinematics(1, ard.HOME_JOINTS, ard))
    print(fwd_kinematics_p5(0, ard.HOME_JOINTS, ard))
    print(fwd_kinematics_p5(1, ard.HOME_JOINTS, ard))

main()