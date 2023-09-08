# conda install -c conda-forge ros-rospy
# pip install rosbags
# pip install open3d
# pip install mrob==0.0.12


import argparse
import os
import struct
from pathlib import Path
from rosbags.highlevel import AnyReader
import open3d as o3d

from sensor_msgs.msg import PointCloud2,  PointField
import numpy as np
from project_map import project_map, save_figs, create_video_from_pngs


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument("--bag_path", type=str, help="path to saved bag",)
parser.add_argument("--poses", type=str,help="path to localization solution ",)
parser.add_argument("--save_path", type=str, default='./assets/',help="path to save meta-data",)
args = parser.parse_args()


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):

    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)[:3]
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)[:3]
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt






if __name__ == '__main__':
    
    _DATATYPES = {}
    _DATATYPES[PointField.INT8]    = ('b', 1)
    _DATATYPES[PointField.UINT8]   = ('B', 1)
    _DATATYPES[PointField.INT16]   = ('h', 2)
    _DATATYPES[PointField.UINT16]  = ('H', 2)
    _DATATYPES[PointField.INT32]   = ('i', 4)
    _DATATYPES[PointField.UINT32]  = ('I', 4)
    _DATATYPES[PointField.FLOAT32] = ('f', 4)
    _DATATYPES[PointField.FLOAT64] = ('d', 8)
    
    os.makedirs(args.save_path, exist_ok=True)
    
    # Change the path here to you data path
    with AnyReader([Path(args.bag_path)]) as reader:
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/laser_cloud_map':
                msg = reader.deserialize(rawdata, connection.msgtype)
                points_as_array = np.array(list(read_points(msg, skip_nans=True)))
                
    map_pcd = o3d.geometry.PointCloud()
    map_pcd.points = o3d.utility.Vector3dVector(points_as_array)
    
    # rectify, filter and save map and T_rect
    project_map(args, map_pcd)
    
    # save localization sequance
    dT = np.load(Path(args.save_path) / 'dT.npy')
    poses = np.load(Path(args.poses) / 'localization_solution.npy')
    plots_dir = os.path.join(Path(args.save_path), "plots")
    save_figs(dT, poses, plots_dir)
    
    # make video from plots
    output_video_filename = os.path.join(Path(args.save_path), "localization_video.mp4")  # Specify the output video filename
    frame_rate = 10  # Set the frame rate for the video
    print('creating video ...')
    create_video_from_pngs(plots_dir, output_video_filename, frame_rate)