import open3d as o3d
import numpy as np

print("Testing IO for images ...")
img = o3d.io.read_image("Reflectance.png")
print(img)

# Load the image
image_path = "Reflectance.png"

image = o3d.io.read_image(image_path)
    
# Convert the image to a binary array
threshold = 128
image_array = np.asarray(image)
grayscale_image = np.mean(image_array, axis=2)
binary_array = grayscale_image > threshold

# Create a voxel grid from the binary array
voxel_size = 0.1  # Adjust the voxel size according to your needs
#voxel_grid = o3d.cpu.pybind.geometry.VoxelGrid(grayscale_image, voxel_size)
voxel_grid = o3d.geometry.VoxelGrid.create_dense(origin = 0,
                                                 color = [255,255,255],
                                                voxel_size = 1,
                                                 width = 512,
                                                 height = 512 ,
                                                 depth = 5)

# Visualize the voxel grid
o3d.visualization.draw_geometries([voxel_grid])
