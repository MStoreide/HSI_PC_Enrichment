import numpy as np 
import open3d as o3d
import matplotlib.pyplot as plt
import pymeshlab as pym
from spectral import *                                                                                                                          




### PyMeshlab

# Loads the mesh and raster, aligns them, and reports camera intrinsics.
ms = pym.MeshSet()

ms.load_new_mesh("ColoredKolbuReg.ply")
ms.load_new_raster("Reflectance.png")   # Using the colored RGB image here. Has the same spatial info as the HSI. 

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

# Extract camera pose from the transformation matrix
rotation_matrix = transformation_matrix[:3, :3]
translation_vector = transformation_matrix[:3, -1]  # Use the last column as translation

# Print the camera intrinsics
print("Camera Intrinsics:")
print("Focal Length (fx, fy):", focal_length_x, focal_length_y)
print("Principal Point (cx, cy):", principal_point_x, principal_point_y)
print("Skew (s):", skew)

# Print the camera pose
print("Registered Camera Pose:")
print("Rotation Matrix:")
print(rotation_matrix)
print("Translation Vector:")
print(translation_vector)

# Apply the get_depth_complexity filter
depth_map = ms.get_depth_complexity(onprimitive = 0, 
                                    numberrays = 128,
                                    depthtexturesize = 512,
                                    peelingiteration = 10,
                                    peelingtolerance = 0.0000000000001)

# Retrieve the depth map
#depth_map = np.array(ms.vertex_property("depth_complexity"))
#depth = np.array(depth_map)

#print(depth_map)

# Visualize the depth map
#plt.imshow(depth_map, cmap='gray')
#plt.axis('off')
#plt.show()


# Save the aligned point cloud
ms.save_current_mesh("alignedcloud.ply")


##############################################################################################################################################

### Open3D

 # Load the point cloud
point_cloud = o3d.io.read_point_cloud("alignedcloud.ply")
pcnor = point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([point_cloud],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)

 # Load the camera intrinsics
intrinsics = o3d.camera.PinholeCameraIntrinsic()
intrinsics.intrinsic_matrix = np.array([[focal_length_x, 0, principal_point_x],   #(cx, cy) = Camera center in pixels
                                         [0, focal_length_y, principal_point_y],   # (fx, fy) = Focal length in pixels
                                         [0, 0, 1]])    # This is copied from the pym code in line 35, 36

 # Load the image
image = o3d.io.read_image("Reflectance.png")

 # Convert the image to a numpy array
image_np = np.asarray(image)

 # Get the image width and height
width = np.shape(image)[1]
height = np.shape(image)[0]

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
