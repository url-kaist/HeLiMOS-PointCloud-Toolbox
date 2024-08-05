# HeLiMOS Pointcloud Toolbox

## Overview
The HeLiMOS pointcloud toolbox is a data processing software for Moving Object Segmentation in the HeLiPR dataset. It includes an effective merging-and-splitting-based approach for labeling four Heterogeneous LiDARs. The program is mainly composed of three modules: `helimos_saver`, `helimos_merger`, and `helimos_propagator`.   Additionally, 

- For the HeLiPR dataset, visit: [HeLiPR Dataset Site](https://sites.google.com/view/heliprdataset)


The HeLiPR-Pointcloud-Toolbox excels with three core functionalities: 

- `helimos_saver` deskews and saves individual LiDAR data and pose data in the SemanticKITTI format.

- `helimos_merger` synchronizes and merges the saved LiDAR data into a single combined cloud.

<table align="center">
  <tr>
    <td><img src="image/pics_merger/Aeva.png" alt="Aeva" width="300"></td>
    <td><img src="image/pics_merger/livox.png" alt="livox" width="300"></td>
    <td rowspan="2"><img src="image/pics_merger/Merged.png" alt="Merged" width="600"></td>
  </tr>
  <tr>
    <td><img src="image/pics_merger/Velodyne.png" alt="Velodyne" width="300"></td>
    <td><img src="image/pics_merger/Ouster.png" alt="Ouster" width="300"></td>
  </tr>
</table>


- `helimos_propagator` backpropagates the labeled points from the merged scan to the individual clouds.

<table align="center">
  <tr>
    <td rowspan="2"><img src="image/pics_propagator/Merged_dyn.png" alt="Propagated" width="600"></td>
    <td><img src="image/pics_propagator/Aeva_dyn.png" alt="Aevadyn" width="300"></td>
    <td><img src="image/pics_propagator/Livox_dyn.png" alt="Livoxdyn 2" width="300"></td>
  </tr>
  <tr>
    <td><img src="image/pics_propagator/Velodyne_dyn.png" alt="Velodyn" width="300"></td>
    <td><img src="image/pics_propagator/Ouster_dyn.png" alt="OSdyn" width="300"></td>
  </tr>
</table>


## Initial Setup

**Buiild and Compile**: 
Clone and compile the HeLiPR-Pointcloud-Toolbox repository using git.
```bash
git clone https://github.com/url-kaist/HeLiMOS-PointCloud-Toolbox.git
cd HeLiMOS-Pointcloud-Toolbox
mkdir build && cd build
cmake ..
make
```



## Usage
1. To create the HeLiMOS dataset, please refer to the config-helimos in the configuration. 

    **Note!** Except for changing the path settings to your path, please do not change any other settings.

```yaml
Path:
  binPath: "/path/to/HeLiPR/lidar/LiDAR/" # The path containing bin file from HeLiPR dataset
  trajPath: "/path/to/HeLiPR/LiDAR_GT/" # The path containing the gt trajectory of the 4 LiDARs
  savePath: "/home/se0yeon00/helimos/" # The path where you want to save the data
```

2. if you want to use helimos saver,
```bash
./helimos_saver
```
3. or if you want to use helimos merger,
```bash
./helimos_merger
```
4. helimos_propagator is coming soon. 

## Dependencies
- `Eigen`: For advanced mathematical operations, particularly with vectors and quaternions.
- `PCL (Point Cloud Library)`: Crucial for point cloud processing and file management.

## License and Citation
- Original helipr paper: 

```bibtex
@misc{jung2023helipr,
      title={HeLiPR: Heterogeneous LiDAR Dataset for inter-LiDAR Place Recognition under Spatial and Temporal Variations}, 
      author={Minwoo Jung and Wooseong Yang and Dongjae Lee and Hyeonjae Gil and Giseop Kim and Ayoung Kim},
      year={2023},
      eprint={2309.14590},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Copyright Notice
All datasets are copyrighted by SNU RPM Labs and are distributed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 License. This license requires proper attribution to the author for any use, prohibits commercial usage, and mandates that derivative works be licensed similarly.

## Maintainer
- Hyungtae Lim (shapelim@mit.edu)
- Seoyeon Jang (9uantum01@kaist.ac.kr)
