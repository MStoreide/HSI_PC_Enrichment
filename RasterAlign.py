import pymeshlab as pym

# Load the raster image and point cloud

ms = pym.MeshSet()

img = ms.load_new_raster(r"/home/markus/Whispers Conference 2023/Reflectance.jpg")
pc = ms.load_new_mesh(r"/home/markus/Whispers Conference 2023/DoorFit.obj")

# Apply the raster_global_refinement_mutual_information filter
ms.apply_filter("raster_global_refinement_mutual_information")

# Retrieve the camera intrinsics
camera_intrinsics = ms.current_mesh().camera_intrinsics()

# Print the camera intrinsics
print("Camera Intrinsics:")
print("Focal Length (fx, fy):", camera_intrinsics[0], camera_intrinsics[1])
print("Principal Point (cx, cy):", camera_intrinsics[2], camera_intrinsics[3])
print("Skew (s):", camera_intrinsics[4])

# Save the aligned point cloud
aligned_point_cloud_path = "Whispers Conference 2023/cloud.ply"
ms.save_current_mesh(aligned_point_cloud_path)
