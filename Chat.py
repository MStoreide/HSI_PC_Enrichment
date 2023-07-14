import numpy as np
import open3d as o3d
import pymeshlab as ml


def load_point_cloud(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd


def load_image(file_path):
    img = o3d.io.read_image(file_path)
    return img


def align_point_cloud_and_image(pcd, img):
        ml.load_mesh_from_trimesh(pcd)
        ml.load_texture(img)
        ml.apply_filter('raster_global_refinement_mutual_information')
        aligned_pcd = ml.extract_trimesh()
        return aligned_pcd


def create_voxel_grid(pcd, resolution):
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=resolution)
    return voxel_grid


def calculate_dot_product(pcd, voxel_grid):
    pcd_normals = pcd.compute_vertex_normals()
    voxel_grid_normals = voxel_grid.compute_normals()
    dot_products = np.dot(pcd_normals, voxel_grid_normals.T)
    return dot_products


def visualize_dot_product_differences(dot_products):
    dot_product_diff = np.abs(dot_products - 1)  # Calculate the absolute difference from 1 (perfect alignment)
    dot_product_diff_img = o3d.geometry.Image(dot_product_diff)
    o3d.visualization.draw_geometries([dot_product_diff_img])


# Example usage
point_cloud_file = 'ColoredKolbuReg.ply'
image_file = 'Reflectance.png'
resolution = 0.1

# Load point cloud and image
pcd = load_point_cloud(point_cloud_file)
img = load_image(image_file)

# Align point cloud and image
aligned_pcd = align_point_cloud_and_image(pcd, img)

# Create voxel grid
voxel_grid = create_voxel_grid(aligned_pcd, resolution)

# Calculate dot product
dot_products = calculate_dot_product(aligned_pcd, voxel_grid)

# Visualize dot product differences
visualize_dot_product_differences(dot_products)
