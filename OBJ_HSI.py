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




# Load OBJ

obj = o3d.io.read_triangle_mesh("G:\Markus_Folder\Kolbu Door\Data\Door.obj")
obj.compute_vertex_normals()
#o3d.visualization.draw_geometries([obj])