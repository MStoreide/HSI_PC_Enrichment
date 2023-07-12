import numpy as np 
import open3d as o3d
import matplotlib as mpl
from spectral import *


# Load HSI
hdr = open_image(r"G:\Markus_Folder\Kolbu Door\Data\HS-DATASET_2023-04-19_006\results\REFLECTANCE_HS-DATASET_2023-04-19_006.hdr")
hdr.__class__
print(hdr) 

# Load RGB version of HSI
hdrrgb = o3d.io.read_image((r"G:\Markus_Folder\Kolbu Door\Data\HS-DATASET_2023-04-19_006\results\REFLECTANCE_HS-DATASET_2023-04-19_006.png"))
print(np.asarray(hdrrgb))

# Add spectral values to array?
# or link the 

#spc_normals = 


##### Load PLY with RGB values

pc = o3d.io.read_point_cloud(r"/media/markus/Business/Datasets/Kolbu/Data/Data/DoorFit.ply")
pc.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
pc_normals = np.asarray(pc.normals)[:]
print("PC normals: ", pc_normals)
o3d.visualization.draw_geometries([pc])               



N = 2000
pcd = pc.sample_points_poisson_disk(N)
# fit to unit cube
pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
          center=pcd.get_center())
pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
o3d.visualization.draw_geometries([pcd])

print('voxelization')
###### Voxel size is set to each pixel size
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                            voxel_size=0.05)  # The voxel size here must be the same as the pixel.
o3d.visualization.draw_geometries([voxel_grid])



###### Get the pixel size of the image in registered 3D space units.


######  Simplify the PC with the pixel size
#voxel_size = #max(NUMVC.get_max_bound() - NUMVC.get_min_bound()) / 800     # This should be set to the pixel size of the image. 
print(f'Voxel_size = {voxel_size:e}')                                
hspc = pc.simplify_vertex_clustering(                        
    voxel_size=voxel_size,                        
    contraction=o3d.geometry.SimplificationContraction.Average)
print(f'Vertex clustering Complete! \n PC has {len(hspc.vertices)} vertices')