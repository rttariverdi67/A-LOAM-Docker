from sklearn.decomposition import PCA
import argparse
import os, copy
import matplotlib.pyplot as plt
import mrob
import numpy as np
import open3d as o3d
from pathlib import Path

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
    
    if not T_rect_check(x):
        x = -1 * x
    
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


def rectify_map(map_pcd):
    cl, ind = map_pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=0.001)
    map_pcd = map_pcd.select_by_index(ind)
    
    map_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.5, max_nn=30))
    normals = np.asarray(map_pcd.normals)
    colors = 0.5 * (normals + 1)  # Normalize and shift the range to [0, 1]
    map_pcd.colors = o3d.utility.Vector3dVector(colors)
    idxs = (colors[:, 2] < 0.3)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(map_pcd.points)[idxs])

    # Rotate map to 2D projection
    pca = PCA(n_components=3)
    pca.fit(np.asarray(pcd.points))
    dT = run_FGraph(pca.components_[2])
    map_rect = copy.deepcopy(pcd).transform(dT)
    
    return dT, map_rect
    
    
def project_map(args):
    os.makedirs(args.save_path, exist_ok=True)
    #read saved map from LOAM
    print('read map.pcd ...')
    map_pcd = o3d.io.read_point_cloud(args.map_path)
    
    #rectify map
    print('rectify map ...')
    dT, map_pcd = rectify_map(map_pcd)
    o3d.io.write_point_cloud(os.path.join(Path(args.save_path), "map_rect.pcd"), map_pcd)
    
    #paint/initialize points colors
    map_pcd.paint_uniform_color([0,0,0])

    # Project points to 2D map image
    min_coord = np.min(np.asarray(map_pcd.points)[:, :2], axis=0)
    max_coord = np.max(np.asarray(map_pcd.points)[:, :2], axis=0)

    meters_resoltion = max_coord - min_coord
    meter2pix = 100
    k = 256
    image_height = (np.ceil(meters_resoltion[0] * meter2pix) // k + 1) * k
    image_width = (np.ceil(meters_resoltion[1] * meter2pix) // k + 1) * k
    image = np.ones((int(image_height), int(image_width), 3))

    points = np.asarray(map_pcd.points)
    colors = np.asarray(map_pcd.colors)

    for i in range(points.shape[0]):
        coords = ((points[i][:2] - min_coord) * meter2pix).astype(int)
        image[coords[0], coords[1]] = colors[i]
        

    dT[:2, 3] = dT[:2, 3] - np.array([min_coord[0], min_coord[1]])
    T_coord = np.array([
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    dT = T_coord @ dT
    print('saving rectification matrix ...')
    np.save(Path(args.save_path) / 'dT.npy', dT)

    print('saving map.png ...')
    # plt.imsave(Path(args.save_path) / "map.png", image)

if __name__ == '__main__':
    project_map(args)