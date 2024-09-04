#include "utility.h"

std::string padZeros(int val, int num_digits)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

bool isInFOV(double x, double y, double z)
{
    // Convert FOV degrees to radians
    const double halfFovX = 70.4 * M_PI / 180.0 / 2.0; // Half FOV for azimuth (x-axis)
    const double halfFovY = 77.2 * M_PI / 180.0 / 2.0; // Half FOV for elevation (y-axis)

    // Convert Cartesian (x, y, z) to Spherical Coordinates (theta, phi)
    double theta = std::atan2(y, x);                      // Azimuth angle
    double phi = std::atan2(z, std::sqrt(x * x + y * y)); // Elevation angle

    // Check if within FOV
    return std::abs(theta) <= halfFovX && phi <= halfFovY && phi >= -25 * M_PI / 180.0;
}

float euclidean_distance(Point3D p1, Point3D p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void normalizePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double cropSize) {

    //find min and max intensity (cloud does not have min and max intensity by default)
    double min_intensity = std::numeric_limits<double>::max();
    double max_intensity = std::numeric_limits<double>::min();
    for (const auto& point : cloud->points) {
        if (point.intensity < min_intensity) {
            min_intensity = point.intensity;
        }
        if (point.intensity > max_intensity) {
            max_intensity = point.intensity;
        }
    }

    // Normalize points and intensity
    for (auto& point : cloud->points) {
        point.x = 1 / cropSize * (point.x);
        point.y = 1 / cropSize * (point.y);
        point.z = 1 / cropSize * (point.z);

        // Normalize intensity to [0, 1]
        if (point.intensity > 256) {
            point.intensity = 1.0;
        }
        else {
        point.intensity = (point.intensity - min_intensity) / (256 - min_intensity);
        }
    }
}

void downsample_to_target_size(pcl::PointCloud<pcl::PointXYZI>& cloud, size_t target_pc_size) {
    double scale_size = 1.001;
    pcl::PointCloud<pcl::PointXYZI> downCloud;

    // Copy the input cloud to downCloud for downsampling
    copyPointCloud(cloud, downCloud);
    down_sampling_voxel(downCloud, scale_size);

    // Adjust voxel size to approach the target point cloud size
    while (downCloud.points.size() < target_pc_size) {
        scale_size -= 0.002;
        if (scale_size <= 0) {
            break;
        }
        copyPointCloud(cloud, downCloud);
        down_sampling_voxel(downCloud, scale_size);
    }
  

    //std::cout << "Downsampled point cloud size: " << downCloud.points.size() << std::endl;
    // Increase scale_size to fine-tune the number of points
    while (downCloud.points.size() > target_pc_size) {
        scale_size += 0.002;
        copyPointCloud(cloud, downCloud);
        down_sampling_voxel(downCloud, scale_size);
    }
    //std::cout << "Downsampled point cloud size: " << downCloud.points.size() << std::endl;
    // If we have fewer points than needed, add random points from the original cloud
    if (downCloud.points.size() < target_pc_size) {
        srand(static_cast<unsigned>(time(nullptr))); // Seed random number generator
        size_t num_extra_points = target_pc_size - downCloud.points.size();

        for (size_t i = 0; i < num_extra_points; ++i) {
            int idx = rand() % cloud.points.size();
            downCloud.points.push_back(cloud.points[idx]);
        }
    }
    //std::cout << "Downsampled point cloud size: " << downCloud.points.size() << std::endl;
    // Copy the downsampled (and potentially expanded) point cloud back to the original
    cloud = downCloud;
}

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  int intensity = rand() % 255;
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      voxel_map[position] = anp;
    }
  }
  plsize = voxel_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

void writeCalibrationFile(const std::string &calibPath){
    std::ofstream foutCalib(calibPath);
    foutCalib << "P0: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl;
    foutCalib << "P1: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl;
    foutCalib << "P2: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl;
    foutCalib << "P3: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl;
    foutCalib << "Tr: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl;
    foutCalib.close();
}


void vec2tf4x4(const std::vector<float> &pose, Eigen::Matrix4f &tf4x4)
{
  Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix();
  tf4x4 << mat3(0, 0), mat3(0, 1), mat3(0, 2), pose[0],
            mat3(1, 0), mat3(1, 1), mat3(1, 2), pose[1],
            mat3(2, 0), mat3(2, 1), mat3(2, 2), pose[2],
            0, 0, 0, 1;
}

std::pair<long long, std::vector<float>> splitLine(std::string input, char delimiter) {
    // The most front part: timestamp should be long long!!!
    std::vector<float> answer;
    std::stringstream  ss(input);
    std::string        temp;

    long long timestamp;
    bool check_ts = true;
    while (getline(ss, temp, delimiter)) {
        if (check_ts) {
            timestamp = stoll(temp);
            check_ts  = false;
            continue;
        }
        answer.push_back(stof(temp));
    }
    return {timestamp, answer};
}

void pcl2nanoflann(const pcl::PointCloud<pcl::PointXYZI>& src_cloud, PointCloud<num_t>& cloud)
{
    int N = src_cloud.points.size();

    cloud.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        cloud.pts[i].x = src_cloud.points[i].x;
        cloud.pts[i].y = src_cloud.points[i].y;
        cloud.pts[i].z = src_cloud.points[i].z;
    }
}

void findDynamicCorrespondences(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, 
                                const pcl::PointCloud<pcl::PointXYZI> &target_cloud, 
                                std::vector<uint32_t>& correspondences) {
    /*
    target_cloud == Merged cloud | query_cloud == A, L, O, V cloud
    */
    PointCloud<num_t> cloud;
    pcl2nanoflann(target_cloud, cloud);

    my_kd_tree_t kdtree(3 /*dim*/, cloud, {10 /* max leaf */});
    correspondences.resize(query_cloud.size());

    int query_idx = 0;
    for (auto &query_pcl: query_cloud.points) {
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
        size_t                num_results = 1;
        std::vector<uint32_t> ret_index(num_results);
        std::vector<num_t>    out_dist_sqr(num_results);

        num_results = kdtree.knnSearch(
                    &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
            
        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);

        float out_dist_sqr_score = out_dist_sqr[0];
         if (out_dist_sqr_score < 0.03) {
                uint32_t target_intensity = target_cloud.points[ret_index[0]].intensity;
                if(target_intensity == 0) {
                    correspondences[query_idx] = 0;
                } else if(target_intensity > 250) {
                    correspondences[query_idx] = 251;
                } else if(target_intensity < 10){
                        correspondences[query_idx] = 9;
                }
        } else {
                correspondences[query_idx] = 9;
        }   
            ++query_idx;
    }

}

void loadLabel(const std::string &label_name, std::vector<uint32_t> &labels) {
   std::ifstream label_input(label_name, std::ios::binary);
    if (!label_input.is_open()) {
        std::cerr << "Could not open the label!" << std::endl;
    }
    label_input.seekg(0, std::ios::end);
    uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
    label_input.seekg(0, std::ios::beg);

    labels.resize(num_points);
    label_input.read((char *) &labels[0], num_points * sizeof(uint32_t));

    label_input.close();
}

void assignLabels(const std::vector<uint32_t> labels, pcl::PointCloud<pcl::PointXYZI> &cloud) {
    for (int i = 0; i < labels.size(); i++) {
        if(labels[i] > 250) {
            cloud.at(i).intensity = 251;
        } else if(labels[i] == 0) {
            cloud.at(i).intensity = 0;
        } else {
            cloud.at(i).intensity = 9;
        }
    }

}

void saveLabels(const std::string &label_name, std::vector<uint32_t> &labels) {
    std::ofstream label_output(label_name, std::ios::binary);
    if (!label_output.is_open()) {
        std::cerr << "Could not open the label!" << std::endl;
    }
    label_output.write(reinterpret_cast<char*>(&labels[0]), labels.size() * sizeof(uint32_t));
    label_output.close();
}



