import numpy as np
import open3d as o3d

# Load the point cloud
point_cloud = o3d.io.read_point_cloud(r"G:\Markus_Folder\Kolbu Door\Data\Door2.ply")

# Load the camera intrinsics
intrinsics = o3d.camera.PinholeCameraIntrinsic()
intrinsics.intrinsic_matrix = np.array([[fx, 0, cx],
                                        [0, fy, cy],
                                        [0, 0, 1]])

# Load the image
image = o3d.io.read_image(r"G:\Markus_Folder\Kolbu Door\Data\HS-DATASET_2023-04-19_006\results\REFLECTANCE_HS-DATASET_2023-04-19_006.png")

# Convert the image to a numpy array
image_np = np.asarray(image)

# Get the image width and height
width = image_np.shape[1]
height = image_np.shape[0]

# Create an Open3D color image from the numpy array
color_image = o3d.geometry.Image(image_np)

# Create an Open3D RGBD image from the color and depth images
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color=color_image,
    depth=o3d.geometry.Image(width, height),  # Placeholder depth image
    depth_scale=1.0,
    depth_trunc=3.0,
    convert_rgb_to_intensity=False
)

# Create a PinholeCameraTrajectory object with a single camera pose
camera_pose = np.eye(4)  # Placeholder camera pose
camera_trajectory = o3d.camera.PinholeCameraTrajectory()
camera_trajectory.parameters = [o3d.camera.PinholeCameraParameters()]
camera_trajectory.parameters[0].extrinsic = camera_pose
camera_trajectory.parameters[0].intrinsic = intrinsics

# Perform ray casting to obtain the 3D points
points = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    intrinsics,
    camera_trajectory
)

# Visualize the sampled points
o3d.visualization.draw_geometries([point_cloud, points])
