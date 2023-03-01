import open3d as o3d
import numpy as np
import pandas as pd
import csv
import math

pcd_generate = o3d.io.read_point_cloud("projection.pcd")
pcd_ground_truth = o3d.io.read_point_cloud("upsampled.pcd")

# csv_generate = np.genfromtxt('3D_vector.csv', delimiter=',')
# csv_ground_truth = np.genfromtxt('edges.csv', delimiter=',')

def point_point_comparison(pcd_generate, pcd_ground_truth) :
    dist1 = np.asarray(pcd_generate.compute_point_cloud_distance(pcd_ground_truth))
    rmse = np.mean(dist1**2)**0.5
    ind = np.where(dist1 <0.1)[0]
    pcd3 = pcd_generate.select_by_index(ind)
    dist2 = np.asarray(pcd_ground_truth.compute_point_cloud_distance(pcd_generate))
    ind = np.where(dist2 <0.1)[0]
    pcd4 = pcd_ground_truth.select_by_index(ind)
    print(" The points number of edge map is:",len(pcd_generate.points))
    print(" The RMS deviation is:",rmse)
    print(" The valid points number in edge map is:",len(pcd3.points))
    print(" The precison is: {:.2%}".format(len(pcd3.points)/len(pcd_generate.points)))
    print(" The detected points number in ground truth is:",len(pcd4.points))
    print(" The recall is: {:.2%}".format(len(pcd4.points)/len(pcd_ground_truth.points)))
    precison = len(pcd3.points)/len(pcd_generate.points)
    recall = len(pcd4.points)/len(pcd_ground_truth.points)
    print(" The F1 score is: {:.2%}".format(2*precison*recall/(precison+recall)))


def collinear(p1, p2, p3):
    # Calculate vectors between points
    v1 = p2 - p1
    v2 = p3 - p1
    # Calculate cross product
    cross_product = np.cross(v1, v2)
    # Check if cross product is zero vector
    if np.linalg.norm(cross_product) <= 1 and p1[0]>=min(p2[0],p3[0])and p1[0]<=max(p2[0],p3[0]):
        return True
    else:
        return False

def point_vector_comparison(csv_generate, csv_ground_truth) :
    count = 0
    for i in range(csv_generate.shape[0]):
        p1 = csv_generate[i, 0:3]
        for j in range(csv_ground_truth.shape[0]):
            p2 = csv_ground_truth[j, 0:3]
            p3 = csv_ground_truth[j, 3:6]
            if collinear(p1, p2, p3):
                count += 1
                break
    print(" The points number of edge map is:",csv_generate.shape[0])
    print(" The valid points number in edge map is:", count)
    print(" The precison is: {:.2%}".format(count/csv_generate.shape[0])) 


def vector_vector_comparison(csv_generate, csv_ground_truth) :
    count = 0
    for i in range(csv_generate.shape[0]):
        p1 = csv_generate[i, 0:3]
        for j in range(csv_ground_truth.shape[0]):
            p2 = csv_ground_truth[j, 0:3]
            p3 = csv_ground_truth[j, 3:6]
            if collinear(p1, p2, p3):
                count += 1
                break
    print(" The vectors number of edge map is:",csv_generate.shape[0])                
    print(" The valid vectors number in edge map is:", count)
    print(" The precison is: {:.2%}".format(count/csv_generate.shape[0])) 

point_point_comparison(pcd_generate, pcd_ground_truth)