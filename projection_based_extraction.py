# coding:utf-8
import open3d as o3d
import numpy as np
import cv2

def img_edge(point_cloud_np):

    vol_bnds = np.zeros((3, 2))

    vol_bnds[:, 0] = np.minimum(vol_bnds[:, 0], np.amin(point_cloud_np, axis=0))
    vol_bnds[:, 1] = np.maximum(vol_bnds[:, 1], np.amax(point_cloud_np, axis=0))

    print(vol_bnds)
    # leafsize = 0.1
    leafsize = 0.1
    vol_dim = np.ceil((vol_bnds[:, 1] - vol_bnds[:, 0]) / leafsize).copy(order='C').astype(int)
    print(vol_dim)

    shift = 100
    bev_map = np.zeros((vol_dim[1]+shift, vol_dim[0]+shift))

    for p in point_cloud_np:

        cx = np.floor((p[0] - vol_bnds[0, 0]) / leafsize)
        cy = np.floor((p[1] - vol_bnds[1, 0]) / leafsize)

        if cx < 0 or cx >= vol_dim[0]:
            continue
        if cy < 0 or cy >= vol_dim[1]:
            continue

        bev_map[int(cy), int(cx)] = 1


    bev_map = (bev_map * 255).astype(np.uint8)  # converter

    v1 = cv2.Canny(bev_map, 80, 150, (3, 3))



    # find edge points
    obj_points = []
    for p in point_cloud_np:

        cx = np.floor((p[0] - vol_bnds[0, 0]) / leafsize)
        cy = np.floor((p[1] - vol_bnds[1, 0]) / leafsize)

        if cx < 0 or cx >= vol_dim[0]:
            continue
        if cy < 0 or cy >= vol_dim[1]:
            continue

        if v1[int(cy), int(cx)] > 0:
            obj_points.append(p)



    cv2.waitKey(100)

    return np.vstack(obj_points)



pcd = o3d.io.read_point_cloud("final.pcd")    
pc_as_np = np.asarray(pcd.points)
print(pcd)
edge_p = img_edge(pc_as_np) 
np.savetxt('ori.txt',(edge_p))
