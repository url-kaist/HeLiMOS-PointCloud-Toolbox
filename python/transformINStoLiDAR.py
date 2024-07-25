import pandas as pd
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from pyproj import Proj, transform


def latlon2local(lat, lon, alt, origin):
    # Define the projection for geographic coordinates
    wgs84 = Proj(proj='latlong', datum='WGS84')

    # Define the local projection: east, north, up
    local = Proj(proj='aeqd', lat_0=origin[0], lon_0=origin[1], datum='WGS84')

    # Transform geographic to local coordinates
    xEast, yNorth = transform(wgs84, local, lon, lat)

    # For altitude, we simply subtract the origin altitude
    zUp = alt - origin[2]

    return xEast, yNorth, zUp

def processTrajectory(inputPath, outputPath, typeLiDAR):
    # Read the CSV file
    latlonheight = pd.read_csv(inputPath, header=None)
    latlonheight = latlonheight.iloc[:, :10].to_numpy()

    # Convert latitude, longitude, height to local coordinates
    x, y, z = latlon2local(latlonheight[:, 1], latlonheight[:, 2], latlonheight[:, 3], [latlonheight[0, 1], latlonheight[0, 2], latlonheight[0, 3]])

    # Convert left-handed angles to right-handed angles
    rpy = latlonheight[:, 7:10]
    rpy[:, 2] = -rpy[:, 2]
    rpy = np.roll(rpy, 1, axis=1)
    rpyRad = np.deg2rad(rpy)

    # Convert roll, pitch, yaw to quaternions
    # Assuming 'ZYX' convention (yaw-pitch-roll)
    quat = R.from_euler('zyx', rpyRad).as_quat()

    latlonheight = np.column_stack((latlonheight[:, 0], x, y, z, quat))

    # # Define LiDAR matrices (R_IL, R_IA, R_IO, R_IV) and R_IN
    R_IL = np.array([[0.999260281307309, 0.0383884500761334, 0.00228409771914936, 0.205400000000000],
                     [-0.0383981303207532, 0.999253021123844, 0.00435699010113986, -0.352800000000000],
                     [-0.00211513344942227, -0.00444147223600340, 0.999987899694225, 0.0706000000000000],
                     [0, 0, 0, 1]])
    R_IA = np.array([[0.998593404866819, 0.0471428768856460, -0.0242643960451897, 0.133700000000000],
                     [-0.0476188752386917, 0.998676624578879, -0.0194278727795602, 0.355700000000000],
                     [0.0233163993252583, 0.0205559888762785, 0.999516781671935, 0.221900000000000],
                     [0, 0, 0, 1]])  # Replace with actual R_IA matrix

    R_IO = np.array([[0.999715495593027, 0.0223448061210468, -0.00834490926264448, -0.417000000000000],
                     [-0.0224514077723064, 0.999664614804883, -0.0129070599303583, -0.00300000000000000],
                     [0.00805370475188661, 0.0130907427756056, 0.999881878170293, 0.299600000000000],
                     [0, 0, 0, 1]])  # Replace with actual R_IO matrix

    R_IV = np.array([[0.999995760375673, 0.000773891425480788, -0.00280719125478291, -1.37200000000000],
                     [-0.000825966811080870, 0.999826715354752, -0.0185972320992637, 0.0668000000000000],
                     [0.00279231257318289, 0.0185994719007949, 0.999823115673720, 0.0297000000000000],
                     [0, 0, 0, 1]])  # Replace with actual R_IV matrix

    # Select the appropriate LiDAR matrix
    if typeLiDAR == "Livox":
        LiDAR = R_IL
    elif typeLiDAR == "Aeva":
        LiDAR = R_IA
    elif typeLiDAR == "Ouster":
        LiDAR = R_IO
    elif typeLiDAR == "Velodyne":
        LiDAR = R_IV
    else:
        raise ValueError("Invalid LiDAR type")

    R_IN = np.array([[0.0110181061714567, -0.999760525916693, -0.0189213279101856, -0.0136388586688681],
                    [0.999915259583709, 0.0108843018002586, 0.00715997514944445, 0.163678520730871],
                    [-0.00695231049069956, -0.0189986948218794, 0.999795674892162, -0.00743152306072198],
                    [0, 0, 0, 1]])

    trajLiDAR = latlonheight
    meshArrayTrajLiDAR = []

    for i in range(len(trajLiDAR)):
        # Select the quaternion components qx, qy, qz, qw
        quat = trajLiDAR[i, 4:8]
        rot = R.from_quat(quat).as_matrix()

        # Apply transformations
        trajLiDAR[i, 1:4] += (rot @ (R_IN[:3, :3] @ LiDAR[:3, 3] + R_IN[:3, 3])).T
        rotLiDAR = rot @ R_IN[:3, :3] @ LiDAR[:3, :3]
        trajLiDAR[i, 4:8] = R.from_matrix(rotLiDAR).as_quat()

        if(i % 20 == 0):
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 2)
            pose = np.identity(4)
            pose[:3, 3] = trajLiDAR[i, 1:4]
            pose[:3, :3] = rotLiDAR
            meshArrayTrajLiDAR.append(mesh.transform(pose))

    # Save to output file
    fmt = ['%.0f', '%.20f', '%.20f', '%.20f', '%.20f', '%.20f', '%.20f', '%.20f']
    np.savetxt(outputPath, trajLiDAR, fmt=fmt, delimiter=' ')
    
    print("Saved to " + outputPath)
    print("Visualizing the trajectory...")
    print("Press 'q' to exit")
    o3d.visualization.draw_geometries(meshArrayTrajLiDAR)
    print("Done")

if __name__ == "__main__":
# User inputs
    inputCSV = input("Enter the path of the input CSV file: ")
    outputTXT = input("Enter the path of the output file: ")
    typeLiDAR = input("Enter the LiDAR type (Livox, Aeva, Ouster, Velodyne): ")

    processTrajectory(inputCSV, outputTXT, typeLiDAR)