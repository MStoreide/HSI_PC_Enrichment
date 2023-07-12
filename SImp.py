import open3d as o3d
import pymeshlab as pym
import pandas as pd


# pc = o3d.io.read_triangle_mesh( "DoorFit.obj")
# print(pc)

# voxel_size = max(pc.get_max_bound() - pc.get_min_bound()) / 450     # This should be set to the pixel size of the image.
# print(f'Voxel_size = {voxel_size:e}')                               # Should provide 262,144 vertices- 
# hspc = pc.simplify_vertex_clustering(                        
#     voxel_size=voxel_size,                        
#     contraction=o3d.geometry.SimplificationContraction.Average)
# print(f'Vertex clustering Complete! \n PC has {len(hspc.vertices)} vertices')

# o3d.io.write_triangle_mesh("DoorFitSimp.obj", hspc)

ms = pym.MeshSet()
ms.load_new_mesh(r"DoorFitSimp.obj")
ms.load_new_mesh(r"DoorFit.obj")

baseline = ms.current_mesh()
print("The baseline mesh is: ", ms.current_mesh_id())
samples = baseline.vertex_number()

haus_dist = ms.get_hausdorff_distance(sampledmesh = 1, 
                                     targetmesh = 0, 
                                     savesample = True,
                                     samplevert = True,
                                     samplenum = (samples),
                                     maxdist = pym.Percentage(50))


hausdf = pd.DataFrame.from_dict(haus_dist, 
                                orient = 'index',
                                columns = ["PBVC Hausdorff Error"])
pd.options.display.float_format = '{:.8f}'.format

hausdf2 = hausdf.T
latex = pd.DataFrame.style.to_latex(hausdf2)
print(latex)