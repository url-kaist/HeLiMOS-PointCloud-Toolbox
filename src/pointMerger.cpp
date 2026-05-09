#include "PointCloudProcessor.h"

int main(int argc, char **argv)
{
    YAML::Node config = YAML::LoadFile("../config/config-helimos.yaml");

    PointCloudProcessor OusterProcessor(config, 0);
    OusterProcessor.loadTrajectoryAndPoses();

    PointCloudProcessor VelodyneProcessor(config, 1);
    VelodyneProcessor.loadTrajectoryAndPoses();

    PointCloudProcessor AviaProcessor(config, 2);
    AviaProcessor.loadTrajectoryAndPoses();

    PointCloudProcessor AevaProcessor(config, 3);
    AevaProcessor.loadTrajectoryAndPoses();

  const auto [minSize, maxSize] = validateAndAlignTimestamps(
      OusterProcessor.timestamp_lists_,
      VelodyneProcessor.timestamp_lists_,
      AviaProcessor.timestamp_lists_,
      AevaProcessor.timestamp_lists_);

  std::string saveAs = config["Save"]["saveAs"].as<std::string>();
  std::string mergedSavePath = config["Path"]["savePath"].as<std::string>() + "Merged/";
  std::string mergedSavePathLiDAR = mergedSavePath + "velodyne/";
  std::string mergedSavePathPose = mergedSavePath + "poses.txt";
  std::string mergedSavePathCalib = mergedSavePath + "calib.txt";
  std::ofstream foutMergedPose;
  std::ofstream foutMergedCalib;
  pcl::PCDWriter pcdWriter;

  if (!std::filesystem::exists(mergedSavePath)) {
    std::cout << "Creating a directory: " << mergedSavePath << std::endl;
    std::filesystem::create_directory(mergedSavePath);
  }
  if (!std::filesystem::exists(mergedSavePathLiDAR)) {
    std::cout << "Creating a directory: " << mergedSavePathLiDAR << std::endl;
    std::filesystem::create_directory(mergedSavePathLiDAR);
  }

  // 1. Write Calibration file
  writeCalibrationFile(mergedSavePathCalib);

  // 2. Write Pose file
  foutMergedPose.open(mergedSavePathPose);

  for (int i = 0; i < maxSize; ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ousterCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulatedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 1. Transform each point cloud to the Ouster frame
    const Eigen::Matrix4f T_o = OusterProcessor.gt_poses_.at(i);
    foutMergedPose << T_o(0, 0) << " " << T_o(0, 1) << " " << T_o(0, 2) << " " << T_o(0, 3) << " "
                   << T_o(1, 0) << " " << T_o(1, 1) << " " << T_o(1, 2) << " " << T_o(1, 3) << " "
                   << T_o(2, 0) << " " << T_o(2, 1) << " " << T_o(2, 2) << " " << T_o(2, 3) << std::endl;

    pcl::PointXYZI minPt, maxPt;

    // 3-1. Merge Ouster
    loadCloud(i, OusterProcessor.savePathLiDAR, "bin", *ousterCloud);
    *accumulatedCloud = *ousterCloud;

    // 3-2. Merge Velodyne
    VelodyneProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    // 3-3. Merge Avia (Livox)
    AviaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    // 3-4. Merge Aeva
    AevaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZI>);

    sor.setInputCloud(accumulatedCloud);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*sampledCloud);

    std::cout << accumulatedCloud->points.size() << " -----> " << sampledCloud->size() << std::endl;
    if (saveAs == "pcd") {
      pcdWriter.writeBinary(mergedSavePathLiDAR + padZeros(i, 6) + ".pcd", *accumulatedCloud);
    } else if (saveAs == "bin") {
      saveToBinFile(mergedSavePathLiDAR + padZeros(i, 6) + ".bin", *accumulatedCloud);
    }
    std::cout << "Saved " << i << "th synced frame" << std::endl;
  }


  return 0;
}
