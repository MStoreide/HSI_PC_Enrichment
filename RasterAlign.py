import pymeshlab as pym
import numpy as np

# Load the raster image and point cloud
ms = pym.MeshSet()

ms.load_new_mesh("ColoredKolbu.ply")
ms.load_new_raster("Reflectance.png")

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
print("Camera Intrinsics:")
print("Focal Length (fx, fy):", focal_length_x, focal_length_y)
print("Principal Point (cx, cy):", principal_point_x, principal_point_y)
print("Skew (s):", skew)

# Save the aligned point cloud
ms.save_current_mesh("alignedcloud.ply")

