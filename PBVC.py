import numpy as np
import open3d as o3d
import pymeshlab as pym
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import spectral as sp

def load_point_cloud_data(pc_filepath):
    """
    Loads a point cloud both in Open3D and a Pandas Dataframe.
    Pandas has 3 dataframes: pdpcd = Points, pdpcdn = Normals, pc = Points and Normals with correct index.
    """
    print("Loading Point Cloud...")
    pcd = o3d.io.read_point_cloud(pc_filepath)
    assert (pcd.has_normals())
    pdpcd = pd.DataFrame(pcd.points, columns=["x", "y", "z"])
    pdpcdn = pd.DataFrame(pcd.normals, columns=["nx", "ny", "nz"])
    pc = pd.concat([pdpcd, pdpcdn], axis=1, join="inner")
    print("Printing Point Cloud as Dataframe:")
    print(pc)

    return pcd, pdpcd, pdpcdn, pc

def extract_img_resolution(img_filepath):
    """
    Extracts the resolution of the image to be projected, which is used to define the size of the voxels in the voxel grid. 
    """
    print("Convert an image to numpy...")
    img = o3d.io.read_image(r"/home/markus/MDPI Paper/Samples/Cultural Heritage/KolbuDoor.JPG")
    mplimg = mpimg.imread(r"/home/markus/MDPI Paper/Samples/Cultural Heritage/KolbuDoor.JPG")
    img_resolution = imgdim.shape
    print("Image Resolution is: ", img_resolution)

    return img_resolution


def align_point_cloud_and_image(pc_filepath, img_filepath):
    """
    Aligns the selected image and the point cloud in 3D space.
    Uses PyMeshLab
    """
    print("Loading Point Cloud and Image...")
    ms = pym.MeshSet()
    pympc = ms.load_new_mesh(pc_filepath)
    pymimg = ms.load_new_raster(img_filepath)
    print("Aligning Point Cloud and Image...")
    ms.apply_filter("raster_global_refinement_mutual_information")
    cur = ms.current_mesh()
    transformation_matrix = cur.vertex_matrix()

    """
    Extract camera intrinsics from the transformation matrix
    """
    focal_length_x = transformation_matrix[0, 0]
    focal_length_y = transformation_matrix[1, 1]
    principal_point_x = transformation_matrix[0, 2]
    principal_point_y = transformation_matrix[1, 2]
    skew = transformation_matrix[0, 1]

    """
    Print the camera intrinsics
    """
    print("Printing Camera Intrinsics:")
    print("Focal Length (fx, fy):", focal_length_x, focal_length_y)
    print("Principal Point (cx, cy):", principal_point_x, principal_point_y)
    print("Skew (s):", skew)

    """
    Save the aligned point cloud
    """
    aligned_pcd = ms.save_current_mesh("alignedcloud.ply")
    return aligned_pcd


### Stops here at the moment

def create_voxel_grid(pcd, img_resolution):
    """
    Creates a voxel grid along the planar point cloud, with the same resolution as the input image. 
    """
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(aligned_pcd, voxel_size=img_resolution)
    return voxel_grid


def calculate_dot_product(pcd, voxel_grid, img):
    """
    Computes the vertex normals of the point cloud, and groups them within each voxel index.
    Calculates the dot product of each vertex within the voxel to the corresponding image pixel normal. 
    """
    pcd_normals = pcd.compute_vertex_normals()
    voxel_grid_normals = voxel_grid.compute_normals()  # This most be converted to the normals of the vectors within each voxel. 
    # pixel_normals = compute_pixel_normals(img)
    for normal in voxel_grid:
        np.mean(normal)
    avg_dot_products =  asd
    dot_products = np.dot(pcd_normals, voxel_grid_normals.T)
    return dot_products, avg_dot_products

def vector_filtering(dot_products, avg_dot_products):
    """
    Here we filter out the points with vectors that receive a dot product with values higher than (-0.9).
    """
    if normal > (-0.9):
        continue
   # else np.delete():

    return accepted_vectors, rejected_vectors


def visualize_dot_product_differences(dot_products):
    """
    Returns an image visualizing which vertices were rejected in the previous steps. 
    """
    dot_product_diff = np.abs(dot_products - 1)  # Calculate the absolute difference from 1 (perfect alignment)
    dot_product_diff_img = o3d.geometry.Image(dot_product_diff)
    o3d.visualization.draw_geometries([dot_product_diff_img])


def PBVC(point_cloud, accepted_points, voxel_size):
    """
    Simplifies the 3D poin cloud based on the enriched spectral data and the rejected vertices. 
    """
    simp_pc = point_cloud.simplify_vertex_clustering(
        voxel_size = voxel_size,
        contraction = o3d.geometry.SimplificationContraction.Average)
    return simp_pc


# Example usage
# point_cloud_file = "/media/markus/Business/Datasets/Kolbu/Data/DoorFit.ply"
# image_file = "/media/markus/Business/Datasets/Kolbu/Data/HS-DATASET_2023-04-19_006/results/REFLECTANCE_HS-DATASET_2023-04-19_006.png"

# # Load point cloud and image
# pcd = load_point_cloud_data(point_cloud_file)
# align_point_cloud_and_image(point_cloud_file, image_file)
# o3d.visualization.draw_geometries([aligned_pcd],
#                                    zoom=0.3412,
#                                    front=[0.4257, -0.2125, -0.8795],
#                                    lookat=[2.6172, 2.0475, 1.532],
#                                    up=[-0.0694, -0.9768, 0.2024])





