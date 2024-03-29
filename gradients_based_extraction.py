# -*-coding:utf-8 -*-

import numpy as np
import open3d as o3d
import time




class Edge3DCentroid:
    def __init__(self):
        """
           Init parameters
        """
        self.pcd        = None  # input point clouds
        self.NPt        = 0     # input point cloud number
        # self.Rnn        = 0.12  # r-neighbour sqrt(0.015), R-neighbour
        self.Rnn        = 1  # r-neighbour sqrt(0.015), R-neighbour
        self.EI         = None  # Save point-wise edge-index
        self.GOrt       = None  # Grid-wise max-orientation
        self.Neighbours = None  # Neighbour system
        self.MaxNN      = 350   # Too many points, not good(edge index  over-smoothing)
        self.NoMaxT     = 0.95  # Decide the direction for No_MaxSuppression.
        self.GradientK  = 100   # Used in caculating gradients
        self.EICopy     = None
        np.seterr(divide='ignore', invalid='ignore')

    def SetPts(self, pc):
        self.pcd = pc
        self.NPt = len(self.pcd.points)
        self.EI  = np.zeros(self.NPt)


    # Caculate edge index by cpp-wrappers, much faster
    # Time cost : from over 400s -> 69.07949757575989s  ->  15.45028567314148s after improving codes
    def CaculateEI_CPP(self):

        # self.Neighbours = cpp_neighbors.batch_query(np.asarray(self.pcd.points).astype(np.float32),
        #                                             np.asarray(self.pcd.points).astype(np.float32),
        #                                             [len(self.pcd.points)],
        #                                             [len(self.pcd.points)],
        #                                             radius = self.Rnn)

        pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        point_cloud_np = np.asarray(self.pcd.points)

        new_point = []
        # new_point_no = []
        for pi in point_cloud_np:
            # [k, idx, _] = pcd_tree.search_radius_vector_3d(pi, self.Rnn)
            [k, idx, _] = pcd_tree.search_knn_vector_3d(pi, self.MaxNN)
            # if k < min_p:
            # temp = point_cloud_np[idx[1:], :]
            new_point.append(idx)

        new_point_np = np.vstack(new_point)
        print(new_point_np.shape)
        self.Neighbours = new_point_np



        self.Neighbours = self.Neighbours[:, :self.MaxNN]
        for i in range(self.NPt):
            # pi = point_cloud_np[i, :]
            # [k, idx, _] = pcd_tree.search_radius_vector_3d(pi, self.Rnn)
            # if k > self.MaxNN:
            #     idx = idx[:self.MaxNN]

            idx = self.Neighbours[i, :]
            idx = idx[idx < self.NPt]
            CenterP = np.mean(np.asarray(self.pcd.points)[idx], axis = 0)  # Get center points
            l_dist = np.sqrt(
                    (np.asarray(self.pcd.points)[idx[len(idx) - 1]][0] - self.pcd.points[i][0]) ** 2 +
                    (np.asarray(self.pcd.points)[idx[len(idx) - 1]][1] - self.pcd.points[i][1]) ** 2 +
                    (np.asarray(self.pcd.points)[idx[len(idx) - 1]][2] - self.pcd.points[i][2]) ** 2)
            self.EI[i] = np.sqrt(np.sum((CenterP - self.pcd.points[i]) ** 2)) / l_dist

    # Get point-wise gradient
    def EdgeGradient_Simple(self):
        self.Neighbours = self.Neighbours[:, :self.GradientK]
        self.GOrt = np.zeros((self.NPt, 3))
        for i in range(self.NPt):
            idx = self.Neighbours[i, :]
            idx = idx[idx < self.NPt]
            gis = abs(self.EI[i] - self.EI[idx]) #gradient values
            maxDiffID = np.argmax(gis)
            # self.GI[i] = gis[maxDiffID] # if the gradient value is needed
            self.GOrt[i][0] = self.pcd.points[idx[maxDiffID]][0] - self.pcd.points[i][0]
            self.GOrt[i][1] = self.pcd.points[idx[maxDiffID]][1] - self.pcd.points[i][1]
            self.GOrt[i][2] = self.pcd.points[idx[maxDiffID]][2] - self.pcd.points[i][2]
            ds = np.sqrt(self.GOrt[i][0]**2 + self.GOrt[i][1]**2 + self.GOrt[i][2]**2)
            if ds < 0.00000001:
                ds = 0.00000001
            self.GOrt[i][0] = self.GOrt[i][0] / ds
            self.GOrt[i][1] = self.GOrt[i][1] / ds
            self.GOrt[i][2] = self.GOrt[i][2] / ds # Gradient direction

    #Non-Max Suppression based on gradients
    def No_MaxSuppression(self):
        self.EICopy = self.EI
        for i in range(self.NPt):
            idx = self.Neighbours[i, :]
            idx = idx[idx < self.NPt]
            dpt = np.asarray(self.pcd.points)[idx] - self.pcd.points[i]
            sum_of_rows = np.sqrt(np.sum(dpt**2, axis=1))
            NeigOrts    = dpt / sum_of_rows[:, np.newaxis]
            ######################################
            NeigOrts   = np.nan_to_num(NeigOrts)
            tileOrt    = np.tile(self.GOrt[i], (len(NeigOrts), 1))
            dotProduct = np.abs(tileOrt * NeigOrts).sum(axis=1)
            local_idx  = np.where(dotProduct > self.NoMaxT)
            local_idx  = idx[local_idx]
            if(self.EICopy[i] < self.EICopy[local_idx]).any():
                self.EI[i] = 0



if __name__ == "__main__":
    # pcd = o3d.io.read_point_cloud(".\\TestData\\fragment.ply") # It takes about 45s
    pcd = o3d.io.read_point_cloud("voxel_preprocessed.pcd") # It takes about 45s
    print(pcd)

    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    print("avg_dist ", avg_dist)


    # EdgePts = ECG.Edge3DCentroid()
    EdgePts = Edge3DCentroid()
    EdgePts.SetPts(pcd)
    # 开始计算
    time_start = time.time()
    EdgePts.CaculateEI_CPP()      # Edge index
    EdgePts.EdgeGradient_Simple() # Gradients
    EdgePts.No_MaxSuppression()   # No_MaxSuppression
    time_end = time.time()
    #
    print('time cost',time_end-time_start,'s')
    print(np.max(EdgePts.EI))
    print(np.min(EdgePts.EI))
    
    # Hard thresholding 
    edgeIdx = np.where(EdgePts.EI > 0.22)
    le  = [list(i) for i in edgeIdx]
    le  = le[0]
    pcdEdge = pcd.select_by_index(le)
    colors = np.zeros((len(le), 3))
    pcdEdge.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.io.write_point_cloud("22.pcd", pcdEdge)
