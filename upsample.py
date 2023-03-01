import numpy as np
import open3d as o3d


# read csv data as pointcloud ","as delimiter is not acceptable
pcd = o3d.io.read_point_cloud("1.csv", format='xyz')

# Create an empty line set
line_set = o3d.geometry.LineSet()

# Set the points of the line set to the points of the point cloud
line_set.points = pcd.points

# Create lines by connecting the consecutive points
lines = []
for i in range(0, len(pcd.points)-1, 2):
	lines.append([i, i+1])	

line_set.lines = o3d.utility.Vector2iVector(lines)

"""
# vis lineset
o3d.visualization.draw_geometries([line_set])
"""

# sample lines
sampled_points = []

#get x,y,z as single arrays
x = np.asarray(line_set.points)[:,0]
y = np.asarray(line_set.points)[:,1]
z = np.asarray(line_set.points)[:,2]

"""#Method to put points depending on threshold
temp:float = 0
tr:float = 0.6 #set threshold
for i in range(0,len(np.asarray(line_set.points))-2,2):
	temp = np.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 + (z[i+1]-z[i])**2) 
	if(temp > tr):
	    	p0, p1 = line_set.points[i], line_set.points[i+1]
	    	sampled_points.append(np.linspace(p0, p1, num=100, axis=0))
	else:
	    	p0, p1 = line_set.points[i], line_set.points[i+1]
	    	sampled_points.append(np.linspace(p0, p1, num=10, axis=0))
"""
#method to put points depending on distance between egdes
length:float = 0
for i in range(0,len(np.asarray(line_set.points))-2,2):
	length = np.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 + (z[i+1]-z[i])**2) 
	p0, p1 = line_set.points[i], line_set.points[i+1]
	sampled_points.append(np.linspace(p0, p1, num=int(length*100), axis=0))

sampled_points = np.concatenate(sampled_points)


sampled_lines = o3d.geometry.PointCloud()
sampled_lines.points = o3d.utility.Vector3dVector(sampled_points)

# vis sampled lines
o3d.visualization.draw_geometries([sampled_lines])

# save as pcd/ply
o3d.io.write_point_cloud("prov.pcd", sampled_lines)
