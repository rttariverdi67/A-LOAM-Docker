from sklearn.decomposition import PCA
import argparse
import os, copy, math
import matplotlib.pyplot as plt
import mrob
import numpy as np
import open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument("--map_path", type=str, default='./map.pcd',help="path to saved map",)
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
    
    # if not T_rect_check(x):
    #     print('T_rect checked.')
    #     x = -1 * x
    
    # How to align a vector with gravity, ensuring that only the rotation is changed
    # For this we need two factors, one looking at the rotation and the second looking at the pose to remain at the origin.

    # Current vector, i
    # x = np.random.randn(3) # Inpout max vector from PCA
    x = x / np.linalg.norm(x)

    # Desired vector, in this case Z axis
    y = np.zeros(3)
    y[2] = 1

    # Solve for one point, it is rank defincient, but the solution should be the geodesic (shortest) path
    graph = mrob.FGraph()
    W = np.eye(3)
    n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
    graph.add_factor_1pose_point2point(
        z_point_x=x, z_point_y=y, nodePoseId=n1, obsInf=W
    )

    # Add anchor factor for position
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
    
    

def pcd_3D_to_2d(map_pcd):
    pcd = copy.deepcopy(map_pcd)
    pcd_points = np.asarray(pcd.points)
    twod_points = np.zeros_like(pcd_points)
    twod_points[:, :2] = pcd_points[:, :2]
    pcd_2d = o3d.geometry.PointCloud()
    pcd_2d.points = o3d.utility.Vector3dVector(twod_points)
    return pcd_2d

def filter_point_cloud(pcd, threshold, K, color):
    
    pcd_2d = pcd_3D_to_2d(pcd)
    point_cloud = np.asarray(pcd_2d.points)
    
    # Build a kd-tree from the point cloud data
    kdtree = cKDTree(point_cloud)

    # List to store points with more than K neighbors
    idxs = []

    # Iterate through all points
    for idx, point in enumerate(point_cloud):
        # Query the kd-tree for neighbors within distance d
        neighbors = kdtree.query_ball_point(point, threshold)

        # Count the number of neighbors
        num_neighbors = len(neighbors)

        # Check if the point has more than K neighbors
        if num_neighbors > K:
            idxs.append(idx)
            
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[idxs])
    pcd_filtered.paint_uniform_color(color) 
    
    pcd_2d.points = o3d.utility.Vector3dVector(np.asarray(pcd_2d.points)[idxs])

    return pcd_filtered, pcd_2d


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
    os.makedirs(args.save_path, exist_ok=True)
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
    
    o3d.io.write_point_cloud(os.path.join(Path(args.save_path), "map_rect.pcd"), map_pcd_filtered)
    o3d.io.write_point_cloud(os.path.join(Path(args.save_path), "map_rect_2d.pcd"), map_pcd_2d)
    
    T_coord = np.array([
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    dT = T_coord @ dT
    print('saving rectification matrix ...')
    np.save(Path(args.save_path) / 'dT.npy', dT)

    # print('saving map.png ...')
    # plt.imsave(Path(args.save_path) / "map.png", image)

if __name__ == '__main__':
    project_map(args)