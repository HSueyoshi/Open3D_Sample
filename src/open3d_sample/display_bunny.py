import open3d as o3d
import numpy as np

DIR_PATH = "src/open3d_sample/data/"

mesh = o3d.io.read_triangle_mesh(o3d.data.BunnyMesh().path)
mesh.compute_vertex_normals()
pcd = mesh.sample_points_poisson_disk(3000)
o3d.visualization.draw_geometries([pcd])

# alpha = 0.01
# print(f"alpha={alpha:.3f}")
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
# mesh.compute_vertex_normals()

# # Normal estimation
# pcd.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# # Orientation of normal vector is consistent with tangent plane 
# pcd.orient_normals_consistent_tangent_plane(10)

# # Surface reconstruction by ball pivoting algorithm
# distances = pcd.compute_nearest_neighbor_distance()
# avg_dist = np.mean(distances)
# radius = 2*avg_dist
# radii = [radius, radius * 2]
radii = [0.005, 0.01, 0.02, 0.04]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))

o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
o3d.io.write_triangle_mesh(DIR_PATH + "bunny.obj", mesh)
