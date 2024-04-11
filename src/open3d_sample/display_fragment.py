import open3d as o3d

DIR_PATH = "src/open3d_sample/data/"

pcd_load = o3d.io.read_point_cloud(DIR_PATH + "fragment.xyz")
o3d.visualization.draw_geometries([pcd_load])
# o3d.io.write_point_cloud(DIR_PATH + "fragment.xyz", pcd_load)
