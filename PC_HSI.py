import numpy as np 
import pymeshlab as pym
import open3d as o3d
import matplotlib as mpl
from spectral import *


# Load HSI
hdr = open_image(r"G:\Markus_Folder\Kolbu Door\Data\HS-DATASET_2023-04-19_006\results\REFLECTANCE_HS-DATASET_2023-04-19_006.hdr")
hdr.__class__
print(hdr) 



#print(hdr)

# Get the size of a pixel for voxelization


# Load PLY

pc = o3d.io.read_point_cloud("G:\Markus_Folder\Kolbu Door\Data\Door2.ply")

o3d.visualization.draw_geometries([pc])               



N = 2000
pcd = obj.sample_points_poisson_disk(N)
# fit to unit cube
pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
          center=pcd.get_center())
pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
o3d.visualization.draw_geometries([pcd])

print('voxelization')
# Voxel size is set to each pixel size
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                            voxel_size=0.05)
o3d.visualization.draw_geometries([voxel_grid])