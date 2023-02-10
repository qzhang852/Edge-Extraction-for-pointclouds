import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

pcd = o3d.io.read_point_cloud("1.pcd")

#Voxel down 
def voxel_down(pcd):
	pcd = o3d.geometry.PointCloud.voxel_down_sample(pcd, 0.05)
	return pcd


# curvature based down
def vector_angle(x, y):
	Lx = np.sqrt(x.dot(x))
	Ly = (np.sum(y ** 2, axis=1)) ** (0.5)
	cos_angle = np.sum(x * y, axis=1) / (Lx * Ly)
	angle = np.arccos(cos_angle)
	angle2 = angle * 360 / 2 / np.pi
	return angle2

def curvature_based(pcd):
	knn_num = 10  # number of points
	angle_thre = 30  
	N = 5  
	C = 10  #should be >N
	point = np.asarray(pcd.points)
	point_size = point.shape[0]
	tree = o3d.geometry.KDTreeFlann(pcd)
	o3d.geometry.PointCloud.estimate_normals(
		pcd, search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn_num))
	normal = np.asarray(pcd.normals)
	normal_angle = np.zeros((point_size))
	for i in range(point_size):
		[_, idx, dis] = tree.search_knn_vector_3d(point[i], knn_num + 1)
		current_normal = normal[i]
		knn_normal = normal[idx[1:]]
		normal_angle[i] = np.mean(vector_angle(current_normal, knn_normal))
		
	point_high = point[np.where(normal_angle >= angle_thre)]
	point_low = point[np.where(normal_angle < angle_thre)]
	pcd_high = o3d.geometry.PointCloud()
	pcd_high.points = o3d.utility.Vector3dVector(point_high)
	pcd_low = o3d.geometry.PointCloud()
	pcd_low.points = o3d.utility.Vector3dVector(point_low)
	pcd_high_down = o3d.geometry.PointCloud.uniform_down_sample(pcd_high, N)
	pcd_low_down = o3d.geometry.PointCloud.uniform_down_sample(pcd_low, C)
	pcd_finl = o3d.geometry.PointCloud()
	pcd_finl.points = o3d.utility.Vector3dVector(np.concatenate((np.asarray(pcd_high_down.points),
								np.asarray(pcd_low_down.points))))
	return pcd_finl

#uniform down 
def voxel_down(pcd):
	pcd = o3d.geometry.PointCloud.uniform_down_sample(pcd, 8)
	return pcd


def outlier_remover(pcd):
	# nb_neighbors:nearest k points    std_ratio:smaller more points disappear
	cl, ind = pcd.remove_statistical_outlier(nb_neighbors=5,std_ratio=1)
	pcd = pcd.select_by_index(ind)
	return cl


def radius_remover(pcd):
	# nb_neighbors:nearest k points    std_ratio:smaller more points disappear
	cl,ind = pcd.remove_radius_outlier(nb_points=5, radius=0.05)
	pcd = pcd.select_by_index(ind)
	return cl

def alpha_shape_mesh(pcd):
	alpha = 1
	print(f"alpha={alpha:.3f}")
	mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
	mesh.compute_vertex_normals()
	return mesh

def ball_pivoting_mesh(pcd):
	radii = [0.1, 0.2, 0.4, 0.8]
	mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
		pcd, o3d.utility.DoubleVector(radii))
	return mesh

def poisson_mesh(pcd):
	with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
		mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)
	vertices_to_remove = densities < np.quantile(densities, 0.05)
	mesh.remove_vertices_by_mask(vertices_to_remove)
	return mesh

