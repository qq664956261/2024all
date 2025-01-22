'''
    The main file for eskf estimation
'''
#!/usr/bin/env python3
import argparse
import os, sys
import math
import numpy as np
import pandas as pd
from pyquaternion import Quaternion
from scipy import interpolate            
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
cwd = os.path.dirname(__file__)
sys.path.append(os.path.join(cwd, './../'))
from plot_util import plot_pos, plot_pos_err, plot_traj

import matplotlib.pyplot as plt

def deleteNAN(array):
    nan_array = np.isnan(array)
    not_nan = ~ nan_array
    new_array = array[not_nan]
    return new_array

def extract_gt(df):
    '''
        helper functions for csv files
        extract ground truth
    '''
    gt_t = deleteNAN(np.array(df['t_pose'])).reshape(-1,1)
    gt_x = deleteNAN(np.array(df['pose_x'])).reshape(-1,1)
    gt_y = deleteNAN(np.array(df['pose_y'])).reshape(-1,1)
    gt_z = deleteNAN(np.array(df['pose_z'])).reshape(-1,1)
    gt_qx = deleteNAN(np.array(df['pose_qx'])).reshape(-1,1)
    gt_qy = deleteNAN(np.array(df['pose_qy'])).reshape(-1,1)
    gt_qz = deleteNAN(np.array(df['pose_qz'])).reshape(-1,1)
    gt_qw = deleteNAN(np.array(df['pose_qw'])).reshape(-1,1)
    gt_pose = np.hstack((gt_t, gt_x, gt_y, gt_z, gt_qx, gt_qy, gt_qz, gt_qw))
    return gt_pose
if __name__ == "__main__":
    csv_file_path = './eskf_positions_stream.csv'
    anchor_npz = "/home/zc/data/dataset/flight-dataset/survey-results/anchor_const1.npz"
    anchor_survey = np.load(anchor_npz)
    # select anchor constellations
    anchor_position = anchor_survey['an_pos']
    # load data
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=2)
    args = parser.parse_args()
    

    # access csv
    csv_file = args.i[0]
    pose_file = args.i[1]
    df = pd.read_csv(csv_file)
    df_pose = pd.read_csv(pose_file)
    print("df_pose", df_pose)
    csv_name = os.path.split(sys.argv[-1])[1] 
    print("ESKF estimation with: " + str(csv_name) + "\n")

    # --------------- extract csv file --------------- #
    gt_pose = extract_gt(df)
    #
    t_vicon = gt_pose[:,0];    pos_vicon = gt_pose[:,1:4]
    # for t, pos in zip(t_vicon, pos_vicon):
    #     print("time:", t)
    #     print("Position:", pos)
    pose_time = deleteNAN(np.array(df_pose['uwbtime'])).reshape(-1,1)
    pose_x = deleteNAN(np.array(df_pose['uwbx'])).reshape(-1,1)
    pose_y = deleteNAN(np.array(df_pose['uwby'])).reshape(-1,1)
    pose_z = deleteNAN(np.array(df_pose['uwbz'])).reshape(-1,1)
    estimate_pose = np.hstack((pose_x, pose_y, pose_z))
    # for t in zip(estimate_pose):
    #     print("1time:", t)

   

    ## compute the error    
    # interpolate Vicon measurements
    f_x = interpolate.splrep(t_vicon, pos_vicon[:,0], s = 0.5)
    f_y = interpolate.splrep(t_vicon, pos_vicon[:,1], s = 0.5)
    f_z = interpolate.splrep(t_vicon, pos_vicon[:,2], s = 0.5)
    x_interp = interpolate.splev(pose_time, f_x, der = 0)
    y_interp = interpolate.splev(pose_time, f_y, der = 0)
    z_interp = interpolate.splev(pose_time, f_z, der = 0)

    x_error = pose_x - x_interp
    y_error = pose_y - y_interp
    z_error = pose_z - z_interp

    pos_error = np.concatenate((x_error.reshape(-1,1), y_error.reshape(-1,1), z_error.reshape(-1,1)), axis = 1)

    rms_x = math.sqrt(mean_squared_error(x_interp, pose_x))
    rms_y = math.sqrt(mean_squared_error(y_interp, pose_y))
    rms_z = math.sqrt(mean_squared_error(z_interp, pose_z))
    print('The RMS error for position x is %f [m]' % rms_x)
    print('The RMS error for position y is %f [m]' % rms_y)
    print('The RMS error for position z is %f [m]' % rms_z)

    RMS_all = math.sqrt(rms_x**2 + rms_y**2 + rms_z**2)          
    print('The overall RMS error of position estimation is %f [m]\n' % RMS_all)

    # visualization
    plot_pos(pose_time, estimate_pose, t_vicon, pos_vicon)
    Ppo=np.zeros((pose_time.size, 9, 9))
    print("pose_time.size:", pose_time.size)
    plot_pos_err(pose_time, pos_error,Ppo)
    plot_traj(pos_vicon, estimate_pose, anchor_position)
    plt.show()