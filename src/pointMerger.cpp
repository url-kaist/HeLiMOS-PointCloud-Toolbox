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

    const auto &ts_o = OusterProcessor.timestamp_lists_;
    const auto &ts_v = VelodyneProcessor.timestamp_lists_;
    const auto &ts_a  = AviaProcessor.timestamp_lists_;
    const auto &ts_e = AevaProcessor.timestamp_lists_;

    std::cout << std::fixed << OusterProcessor.timestamp_lists_[0] << std::endl;
    std::cout << VelodyneProcessor.timestamp_lists_[0] << std::endl;
    std::cout << AviaProcessor.timestamp_lists_[0] << std::endl;
    std::cout << AevaProcessor.timestamp_lists_[0] << std::endl;


    auto isDifferenceWithin = [](long long a, long long b, double diff_in_sec) {
      auto diff = static_cast<double>(std::abs(a - b)) / 1e9;
      return std::abs(diff) <= diff_in_sec;
    };

    auto findMinMaxSize = [](const std::vector<long long> &ts_o,
                           const std::vector<long long> &ts_v,
                           const std::vector<long long> &ts_a,
                           const std::vector<long long> &ts_e) {
      size_t minSize = std::min({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()});
      size_t maxSize = std::max({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()});

      return std::make_tuple(minSize, maxSize);
  };

  if (ts_o[0] < ts_v[0] && ts_o[0] < ts_a[0] && ts_o[0] < ts_e[0]) {
    std::cout << std::fixed << "Check Ouster comes first!" << std::endl;
  } else { std::runtime_error("Currently, only KAIST05 is supported while strongly assuming that the Ouster comes first"); }

  double diff_in_sec = 0.08; 

  const auto &[minSize, maxSize] = findMinMaxSize(ts_o, ts_v, ts_a, ts_e);

  for (int i = 0; i < minSize; ++i) {
    if (isDifferenceWithin(ts_o[i], ts_v[i], diff_in_sec) && isDifferenceWithin(ts_o[i], ts_a[i], diff_in_sec)
      && isDifferenceWithin(ts_o[i], ts_e[i], diff_in_sec)) {
      continue;
    } else {
      std::runtime_error("Something's wrong. Timestamp is not matched :(");
    }
  }

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
