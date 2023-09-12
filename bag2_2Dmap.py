# conda install -c conda-forge ros-rospy
# pip install rosbags
# pip install open3d
# pip install mrob==0.0.12

import argparse
import os
from pathlib import Path

import open3d as o3d
import numpy as np
from project_map import project_map, save_figs, create_video_from_pngs


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--poses_path", type=str,help="path to localization solution, odom.npy",)
    parser.add_argument("--map_path", type=str,help="path to map.pcd",)
    parser.add_argument("--save_path", type=str, default='./assets_test/',help="path to save meta-data",)
    args = parser.parse_args()

    os.makedirs(args.save_path, exist_ok=True)
    
    map_pcd = o3d.io.read_point_cloud(args.map_path)
    
    # rectify, filter and save map and T_rect
    project_map(args, map_pcd)
    
    # save localization sequance
    dT = np.load(Path(args.save_path) / 'dT.npy')
    poses = np.load(args.poses_path)
    plots_dir = os.path.join(Path(args.save_path), "plots")
    os.makedirs(plots_dir, exist_ok=True)

    save_figs(args, poses, plots_dir)
    
    # make video from plots
    output_video_filename = os.path.join(Path(args.save_path), "localization_video.mp4")  # Specify the output video filename
    frame_rate = 10  # Set the frame rate for the video
    print('creating video ...')
    create_video_from_pngs(plots_dir, output_video_filename, frame_rate)