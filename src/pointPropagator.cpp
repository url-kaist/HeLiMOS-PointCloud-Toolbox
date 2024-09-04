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

  std::string mergedSavePath = config["Path"]["savePath"].as<std::string>() + "Merged/";

  std::string mergedSavePathLiDAR = mergedSavePath + "velodyne/";
  std::string mergedSavePathLabel = mergedSavePath + "labels/";
  std::string mergedSavePathPose = mergedSavePath + "poses.txt";
  std::string mergedSavePathCalib = mergedSavePath + "calib.txt";
  
  for (int i = 0; i < maxSize; ++i) {

    std::string mergedLabelName = mergedSavePathLabel + padZeros(i, 6) + ".label";
    if(!std::filesystem::exists(mergedLabelName)) {
      continue;
    }
    std::cout << green << "Processing " << i << "th frame" << reset << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr aevaCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr aviaCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ousterCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr veloCloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::vector<uint32_t> mergedLabel;
    std::vector<uint32_t> aevaLabel;
    std::vector<uint32_t> aviaLabel;
    std::vector<uint32_t> ousterLabel;
    std::vector<uint32_t> veloLabel;

    // 1. Transform each point cloud to the Ouster frame
    const Eigen::Matrix4f T_o = OusterProcessor.gt_poses_.at(i);

    // 2-1. Load Labeled Merged Point Cloud
    loadCloud(i, mergedSavePathLiDAR, "bin", *mergedCloud);
    loadLabel(mergedLabelName, mergedLabel);
    assignLabels(mergedLabel, *mergedCloud);

    // 2-2. Propagate to Ouster
    loadCloud(i, OusterProcessor.savePathLiDAR, "bin", *ousterCloud);
    findDynamicCorrespondences(*ousterCloud, *mergedCloud, ousterLabel);
    std::string saveOusterLabelPath = OusterProcessor.savePathLabel + padZeros(i, 6) + ".label";
    saveLabels(saveOusterLabelPath, ousterLabel);
    std::cout << "Saved " << i << "th propagated Ouster label" << std::endl;
    
    // 2-3. Propagate to Velodyne
    VelodyneProcessor.getTransformedCloud(i, T_o, *veloCloud, "propagated");
    findDynamicCorrespondences(*veloCloud, *mergedCloud, veloLabel);
    std::string saveVeloLabelPath = VelodyneProcessor.savePathLabel + padZeros(i, 6) + ".label";
    saveLabels(saveVeloLabelPath, veloLabel);
    std::cout << "Saved " << i << "th propagated Velodyne label" << std::endl;

    // 2-4. Propagate to Avia
    AviaProcessor.getTransformedCloud(i, T_o, *aviaCloud, "propagated");
    findDynamicCorrespondences(*aviaCloud, *mergedCloud, aviaLabel);
    std::string saveAviaLabelPath = AviaProcessor.savePathLabel + padZeros(i, 6) + ".label";
    saveLabels(saveAviaLabelPath, aviaLabel);
    std::cout << "Saved " << i << "th propagated Avia label" << std::endl;

    // 2-5. Propagate to Aeva
    AevaProcessor.getTransformedCloud(i, T_o, *aevaCloud, "propagated");
    findDynamicCorrespondences(*aevaCloud, *mergedCloud, aevaLabel);
    std::string saveAevaLabelPath = AevaProcessor.savePathLabel + padZeros(i, 6) + ".label";
    saveLabels(saveAevaLabelPath, aevaLabel);
    std::cout << "Saved " << i << "th propagated Aeva label" << std::endl;
    

  }
  return 0;
}
