from sklearn.decomposition import PCA
import argparse
import os, copy, math
import matplotlib.pyplot as plt
import mrob
import numpy as np
import open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
from bagpy import bagreader
import pandas as pd
from tqdm import tqdm
import cv2

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument("--map_path", type=str, help="path to saved map",)
parser.add_argument("--bag_path", type=str, help="path to saved bag",)
parser.add_argument("--save_path", type=str, default='./assets/',help="path to save meta-data",)
args = parser.parse_args()



def T_rect_check(solution):
    
    eye = np.array([0., 1., 0.])
    dot_product = np.dot(solution, eye)
    norm = np.linalg.norm(solution)
    
    cosine_similarity = dot_product / (norm)
    angle_radians = np.arccos(cosine_similarity)
    angle_degrees = np.degrees(angle_radians)
       
    if abs(angle_degrees - 180) < 45:
        return True
    elif abs(angle_degrees) < 45:
        return False



def run_FGraph(x):
    
    x = x / np.linalg.norm(x)
    y = np.array([0., 0., 1.])

    graph = mrob.FGraph()
    W = np.eye(3)
    n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
    graph.add_factor_1pose_point2point(
        z_point_x=x, z_point_y=y, nodePoseId=n1, obsInf=W
    )

    W_0 = np.identity(6) * 1e6
    W_0[:3, :3] = np.identity(3) * 1e-4
    graph.add_factor_1pose_3d(mrob.geometry.SE3(), n1, W_0)

    graph.solve(mrob.LM)
    T = graph.get_estimated_state()

    return T[0]

def pose2D(pose):
    rect_origin = pose.T() @ np.array([0, 0, 0, 1])
    rect_z_axis = pose.T() @ np.array([0, 1, 0, 1])
    
    x_coord = rect_origin[0]
    y_coord = rect_origin[1]
    
    pose_dx = rect_z_axis[0] - rect_origin[0]
    pose_dy = rect_z_axis[1] - rect_origin[1]

    return x_coord, y_coord, pose_dx, pose_dy

def ax(pose, size = 10):
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    return axis.transform(pose)

def plot_map_pose(X, Y, init_pose, dT):
    
    dT = mrob.geometry.SE3(dT)
    
    pose = dT * init_pose
    x, y, dx, dy = pose2D(pose)
    
    fig, ax = plt.subplots(figsize=(12,12))

    ax.scatter(X, Y, marker='.', s=1)

    ax.arrow(
        x, y, dx, dy,
        width=0.1,
        alpha=1,
        color="red",
        head_width=3,
        head_length=math.sqrt((dx*5) ** 2 + (dy*5) ** 2),
        length_includes_head=True,
        head_starts_at_zero=False,
        )

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('some title :)')

    plt.axis('equal')
    plt.show()
    
def quat_to_SE3(quat_pose):
    
    rot_4x1 = quat_pose[-4:]
    tra_3x1 = quat_pose[:3]
    rot = mrob.geometry.quat_to_so3(rot_4x1)
    pose = mrob.geometry.SE3(mrob.geometry.SO3(rot),tra_3x1)
    return pose

def pcd_3D_to_2d_hat(map_pcd):
    pcd = copy.deepcopy(map_pcd)
    pcd_points = np.asarray(pcd.points)
    twod_points = np.zeros_like(pcd_points)
    twod_points[:, :2] = pcd_points[:, :2]
    pcd_2d = o3d.geometry.PointCloud()
    pcd_2d.points = o3d.utility.Vector3dVector(twod_points)
    return pcd_2d

def filter_point_cloud(pcd, threshold, K, color):
    
    pcd_2d = pcd_3D_to_2d_hat(pcd)
    point_cloud = np.asarray(pcd_2d.points)
    
    kdtree = cKDTree(point_cloud)

    idxs = []

    for idx, point in enumerate(point_cloud):
        neighbors = kdtree.query_ball_point(point, threshold)

        num_neighbors = len(neighbors)

        if num_neighbors > K:
            idxs.append(idx)
            
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[idxs])
    pcd_filtered.paint_uniform_color(color) 
    
    pcd_2d.points = o3d.utility.Vector3dVector(np.asarray(pcd_2d.points)[idxs])

    return pcd_filtered, pcd_2d

def read_domData(path_2_odom, dT):
    

    odom_poses = pd.read_csv(path_2_odom)[
                                    [
                                    'pose.pose.position.x',
                                    'pose.pose.position.y',
                                    'pose.pose.position.z',
                                    'pose.pose.orientation.x',
                                    'pose.pose.orientation.y',
                                    'pose.pose.orientation.z',
                                    'pose.pose.orientation.w',
                                    ]
                                    ]
    odom_poses = np.asarray(odom_poses)

    SE3 = np.zeros((odom_poses.shape[0],4,4))
    for i in range(len(odom_poses)):
        SE3[i,:,:] = (quat_to_SE3(odom_poses[i,:])).T() @ dT
        
    return SE3

def rectify_map(map_pcd):
        
    pca = PCA(n_components=3)
    pca.fit(np.asarray(map_pcd.points))
    dT = run_FGraph(pca.components_[2])
    map_rect = copy.deepcopy(map_pcd).transform(dT)
    
    map_points = np.asarray(map_rect.points)
    _2D_points = np.zeros_like(map_points)
    _2D_points[:, :2] = map_points[:, :2]
    
    pcd_2d = o3d.geometry.PointCloud()
    pcd_2d.points = o3d.utility.Vector3dVector(_2D_points)
        
    return dT, map_rect
    
    
def project_map(args):
    
    
    #read saved map from LOAM
    print('read map.pcd ...')
    map_pcd = o3d.io.read_point_cloud(args.map_path)
    
    #rectify map
    print('rectify map ...')
    dT, map_pcd = rectify_map(map_pcd)
    
    map_pcd_filtered, map_pcd_2d = filter_point_cloud(map_pcd, threshold= 0.3, K= 40, color=[0,0,0])
    map_pcd_2d = map_pcd_2d.voxel_down_sample(0.5)
    
    cl, ind = map_pcd_2d.remove_statistical_outlier(nb_neighbors=40, std_ratio=0.5)
    map_pcd_2d = map_pcd_2d.select_by_index(ind)
    
    # will be used to plot scatter map
    _2d_map = np.asarray(map_pcd_2d.points)[:, :2]
    np.save(os.path.join(Path(args.save_path), "map_rect_2d"), _2d_map)
    
    # can be used for 3d visualization 
    o3d.io.write_point_cloud(os.path.join(Path(args.save_path), "map_rect.pcd"), map_pcd_filtered)
    
    
    T_coord = np.array([
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    dT = T_coord @ dT
    print('saving rectification matrix ...')
    np.save(Path(args.save_path) / 'dT.npy', dT)
    
def save_figs(dT, poses, save_path):
    
    fig, ax = plt.subplots(figsize=(12,12))
    plt.rcParams.update({'font.size': 22})

    plt.axis('equal')
    
    X, Y = np.load(os.path.join(Path(args.save_path), 'map_rect_2d.npy')).T
    
    dT = mrob.geometry.SE3(dT)
    for idx, t in enumerate(tqdm(poses)):
        idx_str = f'{idx:04}'
        save_dir = Path(save_path)
        save_dir.mkdir(parents=True, exist_ok=True)
        
          
        pose = mrob.geometry.SE3(t)
        x, y, dx, dy = pose2D(pose)
    
        ax.scatter(X, Y, marker='.', s=1)
        ax.arrow(
            x, y, dx, dy,
            width=0.1,
            alpha=1,
            color="red",
            head_width=3,
            head_length=math.sqrt((dx*5) ** 2 + (dy*5) ** 2),
            length_includes_head=True,
            head_starts_at_zero=False,
            )
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Akula Recording Sequence')
        filename = os.path.join(save_dir, f'{idx_str}.png')
        fig.savefig(filename, format='png')

        ax.clear()


def create_video_from_pngs(png_folder, output_video_filename, frame_rate):
    # Get a list of all PNG files in the folder
    png_files = [f for f in os.listdir(png_folder) if f.endswith('.png')]
    
    if not png_files:
        print("No PNG files found in the folder.")
        return
    
    # Sort the PNG files by filename
    png_files.sort()
    
    # Read the first PNG file to get dimensions
    first_frame = cv2.imread(os.path.join(png_folder, png_files[0]))
    height, width, layers = first_frame.shape
    
    # Define the codec for the video (e.g., XVID)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    # Create a VideoWriter object
    out = cv2.VideoWriter(output_video_filename, fourcc, frame_rate, (width, height))
    
    # Write each PNG frame to the video
    for png_file in png_files:
        frame = cv2.imread(os.path.join(png_folder, png_file))
        out.write(frame)
    
    # Release the VideoWriter
    out.release()
    print(f"Video saved as {output_video_filename}")

if __name__ == '__main__':
    
    os.makedirs(args.save_path, exist_ok=True)
    
    #save odom data
    b = bagreader(args.bag_path)  
    topic = b.message_by_topic('/aft_mapped_to_init')
    odom = pd.read_csv(topic)
    odom.to_csv(os.path.join(Path(args.save_path), "odom.csv"))  
    
    project_map(args)
    
    dT = np.load(Path(args.save_path) / 'dT.npy')
    
    SE3 = read_domData(os.path.join(Path(args.save_path), "odom.csv"), dT)
    
    plots_dir = os.path.join(Path(args.save_path), "plots")
    print('saving plots ...')
    save_figs(dT, list(SE3[::5]), plots_dir)
    
    
    # make video from plots
    output_video_filename = os.path.join(Path(args.save_path), "output_video.mp4")  # Specify the output video filename
    frame_rate = 10  # Set the frame rate for the video
    print('creating video ...')
    create_video_from_pngs(plots_dir, output_video_filename, frame_rate)

    