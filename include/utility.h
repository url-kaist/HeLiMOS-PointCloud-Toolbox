#ifndef UTILITY_H
#define UTILITY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <cstdlib> // For std::rand()
#include <ctime> // For std::time(), seeding the random number generator

#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <filesystem> // for directory reading
#include <string>
#include <dirent.h>
#include <algorithm>
#include <variant>
#include <iomanip>
#include <cstdlib>
#include <chrono>
#include <unordered_map>
#include <memory>
#include <yaml-cpp/yaml.h>

#define HASH_P 116101
#define MAX_N 10000000000

const std::string green = "\033[32m";  // Green text
const std::string red = "\033[31m";    // Red text
const std::string yellow = "\033[33m"; // Yellow for status
const std::string cyan = "\033[36m";   // Cyan for input
const std::string reset = "\033[0m";   // Reset to default colors

enum LiDARType
{
  OUSTER = 0,
  VELODYNE = 1,
  LIVOX = 2,
  AEVA = 3
};

struct LivoxPointXYZI
{
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  uint32_t offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZI,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(uint32_t, offset_time, offset_time))

struct AevaPointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  float reflectivity;
  float velocity;
  int32_t time_offset_ns;
  uint8_t line_index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(AevaPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, reflectivity, reflectivity)(float, velocity, velocity)(int32_t, time_offset_ns, time_offset_ns)(uint8_t, line_index, line_index))

struct OusterPointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ring;
  uint16_t ambient;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint16_t, ring, ring)(uint16_t, ambient, ambient))

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct Point3D
{
  float x;
  float y;
  float z;
};

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};
// Hash value
namespace std {
template <>
struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
}  // namespace std


std::string padZeros(int val, int num_digits);

bool isInFOV(double x, double y, double z); // Checks if a point is in the FOV of the sensor (Livox Accumulation)

float euclidean_distance(Point3D p1, Point3D p2);

void normalizePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double cropSize);

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

void downsample_to_target_size(pcl::PointCloud<pcl::PointXYZI> &pl_feat, size_t target_pc_size);

// ** added for HeLIMOS
void writeCalibrationFile(const std::string &calibPath);
void vec2tf4x4(const std::vector<float> &pose, Eigen::Matrix4f &tf4x4);
std::pair<long long, std::vector<float>> splitLine(std::string input, char delimiter);

template<typename T>
void saveToBinFile(const std::string& filename, const pcl::PointCloud<T>& cloud) {
    std::ofstream outFile(filename, std::ios::out | std::ios::binary);
    if (!outFile) {
        std::cerr << "Cannot open file for writing: " << filename << std::endl;
        return;
    }

    for (const auto& point : cloud) {
        outFile.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
        outFile.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
        outFile.write(reinterpret_cast<const char*>(&point.z), sizeof(point.z));
        if (std::is_same<T, pcl::PointXYZ>::value) {
          outFile.write(reinterpret_cast<const char *>(0), sizeof(point.z));
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
          outFile.write(reinterpret_cast<const char *>(&point.intensity), sizeof(point.intensity));
        }
    }
    outFile.close();
}

template<typename T>
int loadCloud(size_t idx, std::string cloud_dir, std::string cloud_format, pcl::PointCloud<T> &cloud)
{
  if (cloud_dir.back() == '/') { cloud_dir.pop_back(); }
  std::string filename = cloud_dir + "/" + padZeros(idx, 6) + "." + cloud_format; //(boost::format("%s/%06d.%s") % cloud_dir % idx % cloud_format).str();
  FILE   *file    = fopen(filename.c_str(), "rb");
  if (!file) {
    std::cerr << "Error: failed to load " << filename << std::endl;
    return -1;
  }

  std::vector<float> buffer(2000000);
  size_t    num_points =
                fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) /
                4;
  fclose(file);

  cloud.resize(num_points);
  if (std::is_same<T, pcl::PointXYZ>::value) {
    for (int i = 0; i < num_points; i++) {
      auto &pt = cloud.at(i);
      pt.x = buffer[i * 4];
      pt.y = buffer[i * 4 + 1];
      pt.z = buffer[i * 4 + 2];
    }
  } else if (std::is_same<T, pcl::PointXYZI>::value) {
    for (int i = 0; i < num_points; i++) {
      auto &pt = cloud.at(i);
      pt.x         = buffer[i * 4];
      pt.y         = buffer[i * 4 + 1];
      pt.z         = buffer[i * 4 + 2];
      pt.intensity = buffer[i * 4 + 3];
    }
  }
  return 0;
}

#endif // UTILITY_H