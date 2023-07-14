import numpy as np 
import open3d as o3d
import matplotlib as mpl
import pymeshlab as pym
from spectral import *


##### Load HSI #####
hdr = open_image(r"G:\Markus_Folder\Kolbu Door\Data\HS-DATASET_2023-04-19_006\results\REFLECTANCE_HS-DATASET_2023-04-19_006.hdr")
hdr.__class__
print("HDR Info :", hdr) 





##### Load RGB version of HSI and PLY with RGB values (PyMeshLab)
ms = pym.MeshSet()
ms.load_new_mesh("ColoredKolbuReg.ply")
ms.load_new_raster("Reflectance.png")





##### Align Raster and PLY ##### 

# Apply the raster_global_refinement_mutual_information filter
ms.apply_filter("raster_global_refinement_mutual_information")

# Retrieve the transformation matrix
cur = ms.current_mesh()
#transformation_matrix = cur.vertex_matrix("transformation_matrix").to_numpy()
transformation_matrix = cur.vertex_matrix()

# Extract camera intrinsics from the transformation matrix
focal_length_x = transformation_matrix[0, 0]
focal_length_y = transformation_matrix[1, 1]
principal_point_x = transformation_matrix[0, 2]
principal_point_y = transformation_matrix[1, 2]
skew = transformation_matrix[0, 1]

# Print the camera intrinsics
print("Registered Camera Intrinsics:")
print("Focal Length (fx, fy):", focal_length_x, focal_length_y)
print("Principal Point (cx, cy):", principal_point_x, principal_point_y)
print("Skew (s):", skew)

# Save the aligned point cloud
ms.save_current_mesh("AlignedCloud.ply")  # This is the cloud we are working on from now on






##### Load RGB version of HSI and PLY with RGB values (Open3D) #####
hdrrgb = o3d.io.read_image("Reflectance.png")
pc = o3d.io.read_point_cloud("AlignedCloud.ply")
pc.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
pc_normals = np.asarray(pc.normals)[:]
print("Aligned PC normals: ", pc_normals)
print("Visualizing Registered PC Normals: ")
o3d.visualization.draw_geometries([pc],
                                  point_show_normal=True)               



##### Voxel Grid #####

resolution = 1.568   #This should be computed from the pixel size
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pc, voxel_size=resolution)

# Visualize the voxel grid
print("Visualizing Voxel Grid: ")
o3d.visualization.draw_geometries([voxel_grid], point_show_normal = True)

voxinfcent = voxel_grid.get_center()
print("VoxelInfo Get Center:", voxinfcent)
voxinfvox = voxel_grid.get_voxel([1,1,1])
print("VoxelInfo Get Voxel 111:", voxinfvox)


##### Voxel Sampling #####

# Load the camera intrinsics
intrinsics = o3d.camera.PinholeCameraIntrinsic()
intrinsics.intrinsic_matrix = np.array([[focal_length_x, 0, principal_point_x],   #(cx, cy) = Camera center in pixels
                                         [0, focal_length_y, principal_point_y],   # (fx, fy) = Focal length in pixels
                                         [0, 0, 1]])    # This is copied from the pym code in line 35, 36

# Convert the image to a numpy array
image_np = np.asarray(hdrrgb)

 # Get the image width and height
width = np.shape(image_np)[1]
height = np.shape(image_np)[0]

 # Create an Open3D color image from the numpy array
color_image = o3d.geometry.Image(image_np)

 # Create an Open3D RGBD image from the color and depth images
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
     color=color_image,
     depth=o3d.cpu.pybind.geometry.Image(width, height),                   # Placeholder depth image
     depth_scale=1.0,
     depth_trunc=3.0,
     convert_rgb_to_intensity=False
)

# Create a PinholeCameraTrajectory object with a single camera pose
camera_pose = np.eye(4)
camera_pose[:3, :3] = rotation_matrix
camera_pose[:3, 3] = translation_vector
camera_trajectory = o3d.camera.PinholeCameraTrajectory()
camera_trajectory.parameters = [o3d.camera.PinholeCameraParameters()]
camera_trajectory.parameters[0].extrinsic = camera_pose
camera_trajectory.parameters[0].intrinsic = intrinsics

 # Perform ray casting to obtain the 3D points
points = o3d.geometry.PointCloud.create_from_rgbd_image(
     rgbd_image,
     intrinsics,
     camera_trajectory)

 # Visualize the sampled points
o3d.visualization.draw_geometries([point_cloud, points])


##### Dot Products #####




######  Simplify the PC with the pixel size
""" voxel_size = resolution 
print(f'Voxel_size = {voxel_size:e}')                                
hspc = pc.simplify_vertex_clustering(                        
    voxel_size=voxel_size,                        
    contraction=o3d.geometry.SimplificationContraction.Average)
print(f'Vertex clustering Complete! \n PC has {len(hspc.vertices)} vertices') """