import os
import sys

import numpy as np

import pandas as pd
from pyntcloud import PyntCloud

pcd1 = PyntCloud.from_file("f.pcd")

output_dir = "/detected_edge/"


k_n = 50
thresh = 0.03

pcd_np = np.zeros((len(pcd1.points),6))

# find neighbors
kdtree_id = pcd1.add_structure("kdtree")
k_neighbors = pcd1.get_neighbors(k=k_n, kdtree=kdtree_id) 

ev = pcd1.add_scalar_field("eigen_values", k_neighbors=k_neighbors)

x = pcd1.points['x'].values 
y = pcd1.points['y'].values 
z = pcd1.points['z'].values 

e1 = pcd1.points['e3('+str(k_n+1)+')'].values
e2 = pcd1.points['e2('+str(k_n+1)+')'].values
e3 = pcd1.points['e1('+str(k_n+1)+')'].values

sum_eg = np.add(np.add(e1,e2),e3)
sigma = np.divide(e1,sum_eg)
sigma_value = sigma


# Save the edges and point cloud
thresh_min = sigma_value < thresh
sigma_value[thresh_min] = 0
thresh_max = sigma_value > thresh
sigma_value[thresh_max] = 255

pcd_np[:,0] = x
pcd_np[:,1] = y
pcd_np[:,2] = z
pcd_np[:,3] = sigma_value

edge_np = np.delete(pcd_np, np.where(pcd_np[:,3] == 0), axis=0) 

clmns = ['x','y','z','red','green','blue']
pcd_pd = pd.DataFrame(data=pcd_np,columns=clmns)
pcd_pd['red'] = sigma_value.astype(np.uint8)

pcd_points = PyntCloud(pcd_pd)
edge_points = PyntCloud(pd.DataFrame(data=edge_np,columns=clmns))



edge_points.to_file('2.ply')             # Save just the edge points
