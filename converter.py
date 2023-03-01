import open3d as o3d
import numpy as np
import csv

pcd = o3d.io.read_point_cloud("pro3D.pcd")

def pcd2csv(pcd):
    points = np.asarray(pcd.points)
    points = np.round(points, 5)
    np.savetxt("1.csv", points, delimiter=" ", fmt='%.5f')

def vector2point():
    with open('fast_3D_vector.csv', 'r') as input_file:
        reader = csv.reader(input_file)
        data = list(reader)

    output_data = []
    for row in data:
        output_data.append(row[3:6])
        output_data.append([x for i, x in enumerate(row) if i not in [3, 4, 5]])

    with open('fast_3D.csv', 'w', newline='') as output_file:
        writer = csv.writer(output_file)
        writer.writerows(output_data)

def obj2vector():
    vertices = []
    lines = []

    with open("pro3D.obj", "r") as file:
        for line in file:
            if line.startswith("v"):
                x, y, z = [float(e) for e in line.split()[1:]]
                vertices.append((x, y, z))
            elif line.startswith("l"):
                indices = [int(e) for e in line.split()[1:]]
                lines.extend(zip(indices, indices[1:]))

    with open("pro3D_vector.csv", "w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["start_x", "start_y", "start_z", "end_x", "end_y", "end_z"])
        for start, end in lines:
            start = vertices[start - 1]
            end = vertices[end - 1]
            writer.writerow(list(start) + list(end))

def vector2obj():
    with open('edges.csv') as f:
        lines = f.readlines()

    vertices = []
    edges = []
    for line in lines:
        x1, y1, z1, x2, y2, z2 = [float(x) for x in line.strip().split(',')]
        vertices.append((x1, y1, z1))
        vertices.append((x2, y2, z2))
        edges.append((len(vertices) - 2, len(vertices) - 1))

    with open('output.obj', 'w') as f:
        for vertex in vertices:
            f.write(f'v {vertex[0]} {vertex[1]} {vertex[2]}\n')
        for edge in edges:
            f.write(f'l {edge[0] + 1} {edge[1] + 1}\n')



pcd2csv(pcd)