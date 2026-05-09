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
