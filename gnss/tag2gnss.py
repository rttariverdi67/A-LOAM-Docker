import argparse, os, copy, math
import mrob
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from scipy.spatial import cKDTree
from functools import partial


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument("--map_3d", type=str, help="path to pcd map")
parser.add_argument("--tags_num", type=int, help="number of tags")
parser.add_argument("--gnss_coords", type=str, help="path to tag coordinates")

def pose2D(pose):
    rect_origin = pose.T() @ np.array([0, 0, 0, 1])
    rect_z_axis = pose.T() @ np.array([1, 0, 0, 1])
    
    x_coord = rect_origin[0]
    y_coord = rect_origin[1]
    
    pose_dx = rect_z_axis[0] - rect_origin[0]
    pose_dy = rect_z_axis[1] - rect_origin[1]

    return x_coord, y_coord, pose_dx, pose_dy


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

class MAP2GNSS:
    def __init__(self, args):
        self.args = args
        self.fig = None
        self.akula_pose = np.eye(4)

        self.gnss_coords = np.loadtxt(args.gnss_coords, delimiter=',')
        # Nx2 to Nx3
        self.gnss_coords = np.concatenate((self.gnss_coords, np.zeros((self.gnss_coords.shape[0], 1))), axis=1)

        self.tags_coords = []
        
    def onclick(self, event, coord_list):
        if event.button == 3:
            x, y = event.xdata, event.ydata
            print(f"Clicked at coordinates: ({x}, {y})")
            coord_list.append([x, y, 0])
            
            if len(coord_list) == self.args.tags_num:
                plt.close(self.fig)
        
    def get_coords(self, points, coord_list):
        self.fig, ax = plt.subplots(figsize=(15,5))
        ax.plot(points[:, 0], points[:, 1], 'r,')

        akula_pose = mrob.geometry.SE3(self.akula_pose)
        x, y, dx, dy = pose2D(akula_pose)
        ax.arrow(
                x, y, dx, dy,
                width=0.1,
                alpha=1,
                color="blue",
                head_width=3,
                head_length=math.sqrt((dx*5) ** 2 + (dy*5) ** 2),
                length_includes_head=True,
                head_starts_at_zero=False,
                )
        
        partial_onclick = partial(self.onclick, coord_list=coord_list)
        cid = self.fig.canvas.mpl_connect('button_press_event', partial_onclick)
        plt.show()
        
        return coord_list
    
    def main(self):
        Map = o3d.io.read_point_cloud(args.map_3d)
        map_filtered, map_pcd_2d = filter_point_cloud(Map, threshold= 0.3, K= 40, color=[0,0,0])
        map_2d = np.asarray(map_pcd_2d.points)[:, :2]

        #get tag coordinated in map
        self.get_coords(map_2d, self.tags_coords)

        #map to gnss transformation SE3
        T_map_2_gnss = mrob.registration.scaled_arun(self.tags_coords,self.gnss_coords)
        map_2_gnss = map_pcd_2d.transform(T_map_2_gnss)

        #save map2gnss 
        np.save('gnss/map2gnss.npy', T_map_2_gnss)

        #plot map in gnss coordinates
        map_2_gnss = np.asarray(map_2_gnss.points)

        fig, ax = plt.subplots(figsize=(15,5))
        ax.plot(map_2_gnss[:, 0], map_2_gnss[:, 1], 'r,')
        ax.plot(self.gnss_coords[:, 0], self.gnss_coords[:, 1], 'bo', label='Tags')

        plt.title("Akula Map in GNSS coordinates ")
        plt.xlabel("N/S -> y")
        plt.ylabel(" x <- E/W")
        plt.legend()

        plt.show()


if __name__ == '__main__':
   
   args = parser.parse_args()
    
   map_2_gnss = MAP2GNSS(args)
   map_2_gnss.main()

