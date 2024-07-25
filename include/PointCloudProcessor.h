#ifndef PointCloudProcessor_H
#define PointCloudProcessor_H

#include "BsplineSE3.h"
#include "visualizer.h"

using namespace ov_core;

class PointCloudProcessor
{
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    BsplineSE3 bsplineSE3;
    Visualizer visualizer;
    std::vector<Eigen::Quaterniond> scanQuat;
    std::vector<Eigen::Vector3d> scanTrans;
    std::vector<std::string> scanTimestamps;
    std::vector<Point3D> scanPoints;

    std::vector<pcl::PointCloud<OusterPointXYZIRT>> vecOuster;
    std::vector<pcl::PointCloud<PointXYZIRT>> vecVelodyne;
    std::vector<pcl::PointCloud<LivoxPointXYZI>> vecLivox;
    std::vector<pcl::PointCloud<AevaPointXYZIRT>> vecAeva;

    std::vector<Eigen::VectorXd> trajPoints;
    Point3D lastPoint;
    int keyIndex = 0;
    int numBins = 0;

    std::string binPath;
    std::string savePath;
    std::string trajPath;
    std::string saveFile;

    LiDARType LiDAR = OUSTER;    // Ouster, Velodyne, Livox, Aeva (Same as the folder name)
    int distanceThreshold = 10;  // saved pointcloud distanceThreshold (m) (default: 10, >= 0)
    int numIntervals = 1000;     // number of intervals for interpolation (default: 1000, > 1)
    int accumulatedSize = 20;    // number of pointclouds to accumulate before processing (default: 20, > 1)
    int accumulatedStep = 1;     // number of pointclouds to skip before accumulating (default: 1, >= 1)
    bool downSampleFlag = true;  // downsample pointclouds before processing (default: true)
    float downSampleVoxelSize = 0.4f; // downsample size (m) (default: 0.4f)
    int downsamplePointSize: 8192;   // downsample point size (default: 8192, > 0)
    bool normalizeFlag = true;   // normalize pointclouds  (default: true)
    std::string saveAs = "bin";       // save as bin or pcd (default: bin)
    std::string saveName = "Index";   // save name (default: Index)
    bool undistortFlag = true;   // undistort pointclouds before processing (default: true)
    bool cropFlag = true;        // crop pointclouds before processing (default: true)
    float cropSize = 100.0f;      // crop size (m) (default: 100.0f)

    void gatherInput();
    void displayBanner();
    void displayInput();

    void interpolate(double timestamp, double timeStart, double dt,
                     const std::vector<Eigen::Quaterniond> &quaternions,
                     const std::vector<Eigen::Vector3d> &positions,
                     int numIntervals, Eigen::Quaterniond &qOut,
                     Eigen::Vector3d &pOut);

    template <class T>
    void processPoint(T &point, double timestamp, double timeStart, double dt, Eigen::Quaterniond &qStart, Eigen::Vector3d &pStart,
                      const std::vector<Eigen::Quaterniond> &quaternions,
                      const std::vector<Eigen::Vector3d> &positions,
                      int numIntervals, Eigen::Quaterniond &qOut,
                      Eigen::Vector3d &pOut);

    template <class T>
    void accumulateScans(std::vector<pcl::PointCloud<T>> &vecCloud, Point3D &lastPoint);
    
    void readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud);
    void readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud);
    void readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud);
    void readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double timeStart);

    void processFile(const std::string &filename);
    void loadAndProcessBinFiles();
    void loadTrajectory();
};

#endif