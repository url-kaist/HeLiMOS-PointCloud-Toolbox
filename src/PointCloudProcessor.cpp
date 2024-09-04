#include "PointCloudProcessor.h"

PointCloudProcessor::PointCloudProcessor(YAML::Node &config, int sensorType)
{
  displayBanner();
  gatherInput(config, sensorType);
  displayInput();
  visualizer.progressBar(0, 1, "", false);
  // loadTrajectoryAndPoses();
  
}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::displayBanner()
{
// Clear the screen
#ifdef _WIN32
  system("cls");
#else
  system("clear");
#endif

  // Set a fixed width for the banner
  const int bannerWidth = 80; // Adjust this width as needed

  // Define the banner text
  const std::string title = "HeLiMOS Point Cloud Undistortion and Merging Tool";
  const std::string description = "This file is a utility designed to undistort the point clouds, "
                                  "from the HeLiPR dataset in order to create the HeLiMOS dataset. "
                                  "By running this file, you will generate point cloud files "
                                  "that conform to the SemanticKITTI format, as well as the poses.txt and calib.txt files.";

  const std::string inputInfo = "Input: {timestamp}.bin files, TUM format gt poses file.\n"
                                "Output: {index}.bin files, SemanticKITTI format gt poses file, calib.txt file.\n";

  const std::string maintainerInfo = "Maintainer: Seoyeon Jang (9uantum01@kaist.ac.kr, Urban Robotics Lab), Revised: 2024/08/01";

  int titlePadding = (bannerWidth - title.length()) / 2;
  titlePadding = titlePadding < 0 ? 0 : titlePadding; // Ensure no negative padding

  // Display the banner with fixed width
  std::cout << cyan << std::string(bannerWidth, '=') << reset << "\n";
  std::cout << std::string(titlePadding, ' ') << yellow << title << reset << "\n";
  std::cout << cyan << std::string(bannerWidth, '*') << reset << "\n";

  // Wrap and display the description and input information
  std::string combinedInfo = description;
  std::istringstream words(combinedInfo);
  std::string word;
  std::string line;
  while (words >> word)
  {
    if (line.length() + word.length() + 1 > bannerWidth)
    {
      std::cout << line << std::endl;
      line = word;
    }
    else
    {
      line += (line.empty() ? "" : " ") + word;
    }
  }
  if (!line.empty())
    std::cout << line << std::endl;

  // Display Information
  std::cout << cyan << std::string(bannerWidth, '-') << reset << "\n";
  std::cout << inputInfo << "\n";
  std::cout << cyan << std::string(bannerWidth, '-') << reset << "\n";
  std::cout << maintainerInfo << "\n";
  std::cout << cyan << std::string(bannerWidth, '=') << reset << "\n\n";
}

void PointCloudProcessor::gatherInput(YAML::Node &config, int sensorType)
{
  // config from yaml file
  std::string sensorString;
  int LiDARTypeInt;

  switch (sensorType)
  {
  case 0:
    sensorString = "Ouster";
    LiDARTypeInt = 0;
    break;
  case 1:
    sensorString = "Velodyne";
    LiDARTypeInt = 1;
    break;
  case 2:
    sensorString = "Avia";
    LiDARTypeInt = 2;
    break;
  case 3:
    sensorString = "Aeva";
    LiDARTypeInt = 3;
    break;
  }
  //* Load & Save Path
  // YAML::Node config = YAML::LoadFile("../config/config-helimos.yaml");
  binPath = config["Path"]["binPath"].as<std::string>() + sensorString + "/";
  trajPath = config["Path"]["trajPath"].as<std::string>() + sensorString + "_gt.txt";
  savePath = config["Path"]["savePath"].as<std::string>() + sensorString + "/";
  savePathLiDAR = savePath + "velodyne/";
  savePathPose = savePath + "poses.txt";
  savePathCalib = savePath + "calib.txt";
  savePathLabel = savePath + "labels/";

  // * Parameters for saving
  undistortFlag = config["Undistort"]["undistortFlag"].as<bool>();
  numIntervals = config["Undistort"]["numIntervals"].as<int>();
  downSampleFlag = config["Save"]["downSampleFlag"].as<bool>();
  downSampleVoxelSize = config["Save"]["downSampleVoxelSize"].as<float>();
  downsamplePointSize = config["Save"]["downsamplePointSize"].as<int>();
  normalizeFlag = config["Save"]["normalizeFlag"].as<bool>();
  saveName = config["Save"]["saveName"].as<std::string>();
  saveAs = config["Save"]["saveAs"].as<std::string>();
  cropFlag = config["Save"]["cropFlag"].as<bool>();
  cropSize = config["Save"]["cropSize"].as<float>();

  distanceThreshold = config["Save"]["distanceThreshold"].as<int>();
  accumulatedSize = config["Save"]["accumulatedSize"].as<int>();
  accumulatedStep = config["Save"]["accumulatedStep"].as<int>();

  // checking input
  if (!std::filesystem::exists(binPath) || binPath.substr(binPath.size() - 1) != "/")
  {
    std::cout << red << "The path does not exist or the path is not end with folder/. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (!std::filesystem::exists(trajPath) || trajPath.substr(trajPath.size() - 4) != ".txt")
  {
    std::cout << red << "The path does not exist or the file is not txt. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (!std::filesystem::exists(savePath))
  {
    std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
    // Attempt to create the directory
    if (!std::filesystem::create_directory(savePath))
    {
      std::cout << red << "Failed to create the directory. Please check config.yaml." << reset << std::endl;
      exit(0);
    }
  }

  if (!std::filesystem::exists(savePathLiDAR))
  {
    std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
    // Attempt to create the directory
    if (!std::filesystem::create_directory(savePathLiDAR))
    {
      std::cout << red << "Failed to create the directory. Please check config.yaml." << reset << std::endl;
      exit(0);
    }
  }

  if (!std::filesystem::exists(savePathLabel))
  {
    std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
    // Attempt to create the directory
    if (!std::filesystem::create_directory(savePathLabel))
    {
      std::cout << red << "Failed to create the directory. Please check config.yaml." << reset << std::endl;
      exit(0);
    }
  }

  if (numIntervals <= 1)
  {
    std::cout << red << "numIntervals should be greater than 1. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (accumulatedSize <= 0)
  {
    std::cout << red << "accumulatedSize should be greater than 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (accumulatedStep < 1)
  {
    std::cout << red << "accumulatedStep should be greater than or equal to 1. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (downSampleFlag && downSampleVoxelSize <= 0 && downsamplePointSize <= 0)
  {
    std::cout << red << "downSampleVoxelSize and downsamplePointSize should be greater than 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (saveName != "Index" && saveName != "Timestamp")
  {
    std::cout << red << "saveName should be Index or Timestamp. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (saveAs != "bin" && saveAs != "pcd")
  {
    std::cout << red << "saveAs should be bin or pcd. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (cropFlag && cropSize <= 0)
  {
    std::cout << red << "cropSize should be greater than 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (distanceThreshold < 0)
  {
    std::cout << red << "distanceThreshold should be greater than or equal to 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  if (LiDARTypeInt < 0 || LiDARTypeInt > 3)
  {
    std::cout << red << "LiDAR should be 0, 1, 2, or 3. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  LiDAR = static_cast<LiDARType>(LiDARTypeInt);
}

void PointCloudProcessor::displayInput()
{
  const int width = 20; // Set the width for the first column

  std::cout << green << std::string(40, '-') << reset << std::endl; // Divider line
  std::cout << green << std::left << std::setw(width) << "Parameter"
            << "Value" << reset << std::endl;
  std::cout << green << std::string(40, '-') << reset << std::endl; // Divider line
  std::cout << green << std::left << std::setw(width) << "binPath:" << binPath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "trajPath:" << trajPath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "savePath:" << savePath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "savePathLiDAR:" << savePathLiDAR << reset << std::endl;
  std::string lidarTypeStr = visualizer.lidarTypeToString(LiDAR);
  std::cout << green << std::left << std::setw(width) << "LiDAR:" << lidarTypeStr << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Distance Threshold:" << distanceThreshold << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Num Intervals:" << numIntervals << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Accumulated Size:" << accumulatedSize << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Accumulated Step:" << accumulatedStep << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Downsample Flag:" << (downSampleFlag ? "True" : "False") << reset << std::endl;
  if (downSampleFlag)
  {
    std::cout << green << std::left << std::setw(width) << "Downsample Voxel Size:" << downSampleVoxelSize << reset << std::endl;
    std::cout << green << std::left << std::setw(width) << "Downsample Point Size:" << downsamplePointSize << reset << std::endl;
  }
  std::cout << green << std::left << std::setw(width) << "Normalize Flag:" << (normalizeFlag ? "True" : "False") << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Save Name:" << saveName << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Save As:" << saveAs << reset << std::endl;
  
  std::cout << green << std::left << std::setw(width) << "Undistort Flag:" << (undistortFlag ? "True" : "False") << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Crop Flag:" << (cropFlag ? "True" : "False") << reset << std::endl;
  if (cropFlag)
    std::cout << green << std::left << std::setw(width) << "Crop Size:" << cropSize << reset << std::endl;
  std::cout << std::endl;
}

void PointCloudProcessor::interpolate(double timestamp, double timeStart, double dt,
                                      const std::vector<Eigen::Quaterniond> &quaternions,
                                      const std::vector<Eigen::Vector3d> &positions,
                                      int numIntervals, Eigen::Quaterniond &qOut,
                                      Eigen::Vector3d &pOut)
{

  // Calculate the index in the quaternions and positions vectors
  int idx = (timestamp - timeStart) / dt;
  if (idx < 0)
    idx = 0;
  if (idx >= numIntervals - 1)
    idx = numIntervals - 2;

  // Calculate the interpolation factor
  double alpha = (timestamp - (timeStart + idx * dt)) / dt;

  // Interpolate the quaternion using Spherical Linear Interpolation (SLERP)
  qOut = quaternions[idx].slerp(alpha, quaternions[idx + 1]);

  // Interpolate the position linearly
  pOut = (1 - alpha) * positions[idx] + alpha * positions[idx + 1];
}

template <class T>
void PointCloudProcessor::processPoint(T &point, double timestamp, double timeStart, double dt, Eigen::Quaterniond &qStart, Eigen::Vector3d &pStart,
                                       const std::vector<Eigen::Quaterniond> &quaternions,
                                       const std::vector<Eigen::Vector3d> &positions,
                                       int numIntervals, Eigen::Quaterniond &qOut,
                                       Eigen::Vector3d &pOut)
{
  Eigen::Quaterniond qScan;
  Eigen::Vector3d pScan;
  interpolate(timestamp, timeStart, dt, quaternions, positions, numIntervals, qScan, pScan);
  Eigen::Vector3d transformedPoint(point.x, point.y, point.z);
  transformedPoint = qStart * ((qScan * transformedPoint + pScan) - pStart);
  point.x = transformedPoint(0);
  point.y = transformedPoint(1);
  point.z = transformedPoint(2);
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    OusterPointXYZIRT point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
    cloud.push_back(point);
  }
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    PointXYZIRT point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
    cloud.push_back(point);
  }
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    LivoxPointXYZI point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    uint8_t intensity;
    file.read(reinterpret_cast<char *>(&intensity), sizeof(uint8_t));
    // uint8_t to float intensity
    point.intensity = intensity;

    // file.read(reinterpret_cast<char *>(&point.intensity), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));

    if (isInFOV(point.x, point.y, point.z))
      cloud.push_back(point);
  }
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double timeStart)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    AevaPointXYZIRT point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
    file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
    if (timeStart > 1691936557946849179 / 1e9)
      file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    cloud.push_back(point);
  }
  file.close();
}

template <class T>
void PointCloudProcessor::accumulateScans(std::vector<pcl::PointCloud<T>> &vecCloud, Point3D &lastPoint)
{
  pcl::PCDWriter pcdWriter;
  pcl::PointCloud<T> accumulatedCloud;

  if (vecCloud.size() == accumulatedSize)
  {
    if (euclidean_distance(lastPoint, scanPoints[0]) > distanceThreshold)
    {
      lastPoint = scanPoints[0];
      for (int i = 0; i < accumulatedSize; i++)
      {
        auto pclIter = vecCloud[i].points.end() - 1;
        for (; pclIter != vecCloud[i].points.begin(); pclIter--)
        {
          T point = *pclIter;
          Eigen::Vector3d transformedPoint(pclIter->x, pclIter->y, pclIter->z);
          transformedPoint = scanQuat[0].conjugate() * ((scanQuat[i] * transformedPoint + scanTrans[i]) - scanTrans[0]);
          point.x = transformedPoint(0);
          point.y = transformedPoint(1);
          point.z = transformedPoint(2);

          if (cropFlag && (point.x * point.x + point.y * point.y + point.z * point.z) > cropSize * cropSize)
            continue;

          accumulatedCloud.push_back(point);
        }
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
      copyPointCloud(accumulatedCloud, *sampledCloud);

      if (downSampleFlag)
      {
        if (downsamplePointSize > 0)
          downsample_to_target_size(*sampledCloud, downsamplePointSize);
        else
          down_sampling_voxel(*sampledCloud, downSampleVoxelSize);
      }

      if (normalizeFlag)
        normalizePointCloud(sampledCloud, cropSize);

      if (saveName == "Index")
        saveFile = savePathLiDAR + padZeros(keyIndex - accumulatedSize, 6);
      else if (saveName == "Timestamp")
        saveFile = savePathLiDAR + scanTimestamps[0];

      visualizer.progressBar(keyIndex, numBins, saveFile + "." + saveAs, true);
      if (saveAs == "bin")
      {
        saveToBinFile(saveFile + ".bin", *sampledCloud);
        // std::ofstream file;
        // file.open(saveFile + ".bin", std::ios::out | std::ios::binary);
        // for (auto &point : sampledCloud->points)
        // {
        //   file.write(reinterpret_cast<const char *>(&point.x), sizeof(point.x));
        //   file.write(reinterpret_cast<const char *>(&point.y), sizeof(point.y));
        //   file.write(reinterpret_cast<const char *>(&point.z), sizeof(point.z));
        //   file.write(reinterpret_cast<const char *>(&point.intensity), sizeof(point.intensity));
        // }
        // file.close();
      }
      else if (saveAs == "pcd")
        pcdWriter.writeBinary(saveFile + ".pcd", *sampledCloud);
    }

    for (int i = 0; i < accumulatedStep; i++)
    {
      vecCloud.erase(vecCloud.begin());
      scanQuat.erase(scanQuat.begin());
      scanTrans.erase(scanTrans.begin());
      scanTimestamps.erase(scanTimestamps.begin());
      scanPoints.erase(scanPoints.begin());
    }
  }
}

void PointCloudProcessor::processFile(const std::string &filename)
{
  pcl::PointCloud<OusterPointXYZIRT>::Ptr scanOuster(new pcl::PointCloud<OusterPointXYZIRT>);
  pcl::PointCloud<PointXYZIRT>::Ptr scanVelodyne(new pcl::PointCloud<PointXYZIRT>);
  pcl::PointCloud<LivoxPointXYZI>::Ptr scanLivox(new pcl::PointCloud<LivoxPointXYZI>);
  pcl::PointCloud<AevaPointXYZIRT>::Ptr scanAeva(new pcl::PointCloud<AevaPointXYZIRT>);

  Eigen::Quaterniond qStart;
  Eigen::Quaterniond qScan;
  Eigen::Vector3d pStart;
  Eigen::Vector3d pScan;

  std::vector<Eigen::Quaterniond> quaternions(numIntervals);
  std::vector<Eigen::Vector3d> positions(numIntervals);

  // Extract the timestamp from the filename
  std::size_t startPos = filename.find_last_of("/") + 1;
  std::size_t endPos = filename.find_last_of(".");
  std::string timestampStr = filename.substr(startPos, endPos - startPos);

  double timeStart = std::stod(timestampStr) / 1e9; // Convert from nanoseconds to seconds

  // time check with trajectory
  if (timeStart < trajPoints[0](0) - 100 || timeStart > trajPoints[trajPoints.size() - 1](0) + 100)
  {
    std::cout << red << "The timestamp is out of range. Please check the trajectory file and bin files." << reset << std::endl;
    exit(0);
  }

  bool success_field = bsplineSE3.get_pose(timeStart, qStart, pStart);

  if (!success_field) // only for initial point
  {
    int idx = 0;
    for (int i = 0; i < trajPoints.size() - 1; i++)
    {
      if (timeStart >= trajPoints[i](0) && timeStart <= trajPoints[i + 1](0))
      {
        idx = i;
        break;
      }
    }
    double alpha = (timeStart - trajPoints[idx](0)) / (trajPoints[idx + 1](0) - trajPoints[idx](0));
    Eigen::Quaterniond q0 = Eigen::Quaterniond(trajPoints[idx](7), trajPoints[idx](4), trajPoints[idx](5), trajPoints[idx](6));
    Eigen::Quaterniond q1 = Eigen::Quaterniond(trajPoints[idx + 1](7), trajPoints[idx + 1](4), trajPoints[idx + 1](5), trajPoints[idx + 1](6));
    qStart = q0.slerp(alpha, q1);
    pStart = (1 - alpha) * trajPoints[idx].block(1, 0, 3, 1) + alpha * trajPoints[idx + 1].block(1, 0, 3, 1);
  }

  switch (LiDAR)
  {
  case OUSTER:
    readBinFile(filename, *scanOuster);
    break;
  case VELODYNE:
    readBinFile(filename, *scanVelodyne);
    break;
  case LIVOX:
    readBinFile(filename, *scanLivox);
    break;
  case AEVA:
    readBinFile(filename, *scanAeva, timeStart);
    break;
  }

  Point3D currPoint;
  currPoint.x = pStart(0);
  currPoint.y = pStart(1);
  currPoint.z = pStart(2);

  // if (success_field) // uncomment it to save same number of scans & poses with LiDAR_gt
  {
    scanPoints.push_back(currPoint);
    scanQuat.push_back(qStart);
    scanTrans.push_back(pStart);
    scanTimestamps.push_back(timestampStr);

    qStart = qStart.conjugate();
    double timeEnd = timeStart + 0.105;
    double dt = (timeEnd - timeStart) / numIntervals;
    bool exitFlag = false;
    for (int i = 0; i < numIntervals; ++i)
    {
      double t = timeStart + i * dt;
      if (!bsplineSE3.get_pose(t, quaternions[i], positions[i]))
      {
        exitFlag = true;
      }
    }

    switch (LiDAR)
    {
    case OUSTER:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanOuster->points)
        {
          double timestamp = timeStart + point.t / float(1000000000);
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecOuster.push_back(*scanOuster);
      break;
    case VELODYNE:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanVelodyne->points)
        {
          double timestamp = timeStart + point.time;
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecVelodyne.push_back(*scanVelodyne);
      break;
    case LIVOX:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanLivox->points)
        {
          double timestamp = timeStart + point.offset_time / float(1000000000);
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecLivox.push_back(*scanLivox);
      break;
    case AEVA:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanAeva->points)
        {
          double timestamp = timeStart + point.time_offset_ns / float(1000000000);
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecAeva.push_back(*scanAeva);
      break;
    }
    keyIndex++;
  }
}

void PointCloudProcessor::loadAndProcessBinFiles()
{
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> binFiles;

  if ((dir = opendir(binPath.c_str())) != NULL)
  {
    while ((ent = readdir(dir)) != NULL)
    {
      std::string filename = ent->d_name;
      if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bin")
      {
        binFiles.push_back(filename);
      }
    }
    closedir(dir);
  }
  
  writeCalibrationFile(savePathCalib);
  foutPose.open(savePathPose);

  std::sort(binFiles.begin(), binFiles.end());
  Point3D lastPoint;
  lastPoint.x = -999;
  lastPoint.y = -999;
  lastPoint.z = -999;
  numBins = binFiles.size();
  

  for (const std::string &filename : binFiles)
  {
    visualizer.progressBar(keyIndex, binFiles.size(), filename, false);
    processFile(binPath + filename);
    switch (LiDAR)
    {
    case OUSTER:
      accumulateScans(vecOuster, lastPoint);
      break;
    case VELODYNE:
      accumulateScans(vecVelodyne, lastPoint);
      break;
    case LIVOX:
      accumulateScans(vecLivox, lastPoint);
      break;
    case AEVA:
      accumulateScans(vecAeva, lastPoint);
      break;
    }
    if(keyIndex - accumulatedSize < 0)
      continue; // * need to be refactored

    Eigen::Matrix4f tf4x4 = gt_poses_[keyIndex - accumulatedSize];
    foutPose << tf4x4(0, 0) << " " << tf4x4(0, 1) << " " << tf4x4(0, 2) << " " << tf4x4(0, 3) << " "
              << tf4x4(1, 0) << " " << tf4x4(1, 1) << " " << tf4x4(1, 2) << " " << tf4x4(1, 3) << " "
              << tf4x4(2, 0) << " " << tf4x4(2, 1) << " " << tf4x4(2, 2) << " " << tf4x4(2, 3) << std::endl; 
  }

  foutPose.close();
}

void PointCloudProcessor::loadTrajectory()
{
  std::string line;
  std::ifstream file(trajPath);

  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    Eigen::VectorXd point(8);
    if (!(iss >> point(0) >> point(1) >> point(2) >> point(3) >> point(4) >> point(5) >> point(6) >> point(7)))
    {
      break;
    }
    point(0) *= 1e-9;
    trajPoints.push_back(point);
  }
  file.close();
  bsplineSE3.feed_trajectory(trajPoints);
}

void PointCloudProcessor::loadTrajectoryAndPoses()
{
  std::string line;
  std::ifstream file(trajPath);

  gt_poses_.clear();
  gt_poses_.reserve(100000);

  while (std::getline(file, line))
  {
    // * 1. load poses
    const auto & [timestamp, pose] = splitLine(line, ' ');
    Eigen::Matrix4f tf4x4_sensor = Eigen::Matrix4f::Identity();
    vec2tf4x4(pose, tf4x4_sensor);
    gt_poses_.push_back(tf4x4_sensor);
    timestamp_lists_.push_back(timestamp);

    // * 2. load trajectory
    std::istringstream iss(line);
    Eigen::VectorXd point(8);
    if (!(iss >> point(0) >> point(1) >> point(2) >> point(3) >> point(4) >> point(5) >> point(6) >> point(7)))
    {
      break;
    }
    point(0) *= 1e-9;
    trajPoints.push_back(point);
  }
  file.close();
  bsplineSE3.feed_trajectory(trajPoints);

}

void PointCloudProcessor::getTransformedCloud(const int i, const Eigen::Matrix4f T_criterion, pcl::PointCloud<pcl::PointXYZI> &transformed, const std::string stage) {
  // An already deskewed file is loaded
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_extrinsic(new pcl::PointCloud<pcl::PointXYZI>);
  transformed.clear();
  if (i < timestamp_lists_.size()) {
    loadCloud(i, savePathLiDAR, "bin", *cloud);
  } else {
    return;
  }

  const auto T_diff = T_criterion.inverse() * gt_poses_.at(i);
  if (stage == "merge") { 
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  } 
  pcl::transformPointCloud(*cloud, transformed, T_diff);
  // transformed = *cloud_extrinsic;
}