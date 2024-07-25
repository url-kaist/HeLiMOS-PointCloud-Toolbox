import open3d as o3d
import numpy as np
import struct
import os

def read_bin_file(filename, typeLiDAR):
    points = []
    with open(filename, "rb") as file:
        while True:
            if typeLiDAR == "Velodyne":
                data = file.read(22)
                if len(data) < 22:
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])
                ring = struct.unpack('H', data[16:18])
                time = struct.unpack('f', data[18:22])
            elif typeLiDAR == "Ouster":
                data = file.read(26)
                if len(data) < 26:
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])
                t = struct.unpack('I', data[16:20]) 
                reflectivity, ring, ambient = struct.unpack('HHH', data[20:26])
            elif typeLiDAR == "Aeva" and int(filename.split("/")[-1].split('.')[0]) > 1691936557946849179: 
                data = file.read(29)
                if len(data) < 29:
                    break
                x, y, z, reflectivity, velocity = struct.unpack('fffff', data[:20])
                time_offset_ns = struct.unpack('I', data[20:24])
                line_index = struct.unpack('B', data[24:25])
                intensity = struct.unpack('f', data[25:29])
            elif typeLiDAR == "Aeva" and int(filename.split("/")[-1].split('.')[0]) <= 1691936557946849179: 
                data = file.read(25)
                if len(data) < 25:
                    break
                x, y, z, reflectivity, velocity = struct.unpack('fffff', data[:20])
                time_offset_ns = struct.unpack('I', data[20:24])
                line_index = struct.unpack('B', data[24:25])
            elif typeLiDAR == "Livox":
                data = file.read(19)
                if len(data) < 19:
                    break
                x, y, z = struct.unpack('fff', data[:12])
                reflectivity, tag, line = struct.unpack('BBB', data[12:15])
                offset_time = struct.unpack('f', data[15:19])
            else:
                raise ValueError("Unsupported LiDAR type")
            points.append([x, y, z])
    return points

def visualize_point_cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    o3d.visualization.draw_geometries([pcd])

# Example Usage
filename = input("Enter the path of bin file: ")
typeLiDAR = input("Enter the LiDAR type (Livox, Aeva, Ouster, Velodyne): ")  # Change as per your LiDAR type: "Velodyne", "Ouster", "Aeva", or "Livox"
pointcloud = read_bin_file(filename, typeLiDAR)
visualize_point_cloud(pointcloud)
