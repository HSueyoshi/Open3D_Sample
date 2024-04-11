import open3d as o3d
import numpy as np

DIR_PATH = "src/open3d_sample/data/"

pcd_load = o3d.io.read_point_cloud(DIR_PATH + "bathtub.xyz")
# pcd_load = o3d.io.read_triangle_mesh(DIR_PATH + "bathtub_0154.ply")
o3d.visualization.draw_geometries([pcd_load])

# Normal estimation
pcd_load.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Orientation of normal vector is consistent with tangent plane 
pcd_load.orient_normals_consistent_tangent_plane(10)

# Surface reconstruction by ball pivoting algorithm
distances = pcd_load.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 2*avg_dist
radii = [radius, radius * 2]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd_load, o3d.utility.DoubleVector(radii))


# alpha = 0.15
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#     pcd_load, alpha
# )
# mesh.compute_vertex_normals()
# Visualization in window (BPA)
o3d.visualization.draw_geometries([mesh])

# o3d.io.write_point_cloud(DIR_PATH + "bathtub.stl", pcd_load)
o3d.io.write_triangle_mesh(DIR_PATH + "bathtub.stl", mesh)
