import numpy as np
import open3d as o3d
   
# Params class containing the counter and the main LineSet object holding all line subsets
class params():

    counter = 0
    full_line_set = o3d.geometry.LineSet()

# Callback function for generating the LineSets from each neighborhood
#def build_edges(vis):
def build_edges():
    # Run this part for each point in the point cloud
    print("Build Edges!")    	
    for params.counter in range( len(points)):
        # Find the K-nearest neighbors in Radius r for the current point.
        [k, idx, _] = pcd_tree.search_hybrid_vector_3d(points[params.counter,:], 0.4, 3)
        # Get the neighbor points from the indices
        points_temp = points[idx,:]
        
        # Create the neighbours indices for the edge array
        neighbours_num = np.arange(len(points_temp))
        # Create a temp array for the center point indices
        point_temp_num = np.zeros(len(points_temp))
        # Create the edges array as a stack from the current point index array and the neighbor indices array
        edges = np.vstack((point_temp_num,neighbours_num)).T

        # Create a LineSet object and give it the points as nodes together with the edges
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points_temp)
        line_set.lines = o3d.utility.Vector2iVector(edges)
        # Color the lines by either using red color for easier visualization or with the colors from the point cloud
        line_set.paint_uniform_color([1,0, 0])
        # line_set.paint_uniform_color(colors[params.counter,:])
        
        # Add the current LineSet to the main LineSet
        params.full_line_set+=line_set
        params.counter +=1
    else:   
        params.counter=0
        return
    
# Load point cloud .ply into Open3D point cloud object
point_cloud = o3d.io.read_point_cloud("fast.ply")

# crop pointcloud
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-100, -100, -100), max_bound=(100, 100, 85.2))
point_cloud = point_cloud.crop(bbox)
#o3d.visualization.draw_geometries([point_cloud])

"""
# downsample pointcloud
point_cloud = point_cloud.voxel_down_sample(voxel_size=0.1)
#o3d.visualization.draw_geometries([point_cloud])
"""

# get the points and colors as separate numpy arrays
points = np.asarray(point_cloud.points)
colors = np.asarray(point_cloud.colors)

# Calculate the KDTree from the point cloud
pcd_tree = o3d.geometry.KDTreeFlann(point_cloud)


# Recolor the point cloud in blue, just to get a better contrast with the distance lines
point_cloud.paint_uniform_color([0, 0, 1])

build_edges()

# o3d.visualization.draw_geometries([params.full_line_set, point_cloud])
o3d.io.write_point_cloud("kdtrees.ply", (point_cloud))

