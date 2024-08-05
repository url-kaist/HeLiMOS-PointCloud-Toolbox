<div align="center">
    <h1>HeLiMOS Pointcloud Toolobox</h1>
    <a href="https://github.com/url-kaist/HeLiMOS-PointCloud-Toolbox"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/HeLiMOS-PointCloud-Toolbox"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://sites.google.com/view/heliprdataset"><img src="https://img.shields.io/badge/HeLiPR_Dataset-4285F4?style=for-the-badge&logo=google-cloud&logoColor=white" height="21"/></a>
    <!-- a href="https://ieeexplore.ieee.org/document/9981561"><img src="https://img.shields.io/badge/DOI-10.1109/IROS47612.2022.9981561-004088.svg"/-->
    <br />
    <br />
    <a href=https://urobot.kaist.ac.kr/>Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://urobot.kaist.ac.kr/">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
     <a href=https://urobot.kaist.ac.kr/>Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://urobot.kaist.ac.kr/>Contact Us</a>
  <br />
  <br />
  <p align="center"><img src=image/overview.png alt="animated" /></p>

  [The HeLiMOS pointcloud toolbox][Projectlink] is a data processing software for moving object segmentation (MOS) in the HeLiPR dataset.<br>
  It includes an effective merging-and-splitting-based approach ((a) and (g) in the upper figure) for labeling four heterogeneous LiDAR sensors.
</div>

[YouTubeLInk]: https://urobot.kaist.ac.kr/
[arXivlink]: https://urobot.kaist.ac.kr/
[Proejectlink]: https://urobot.kaist.ac.kr/
[Datasetlink]: https://urobot.kaist.ac.kr/

## :package: Installation
> What we need are just minimal dependencies.

```commandline
sudo apt-get install g++ build-essential libpcl-dev libeigen3-dev python3-pip python3-dev cmake -y
```

### Tested Environment
- Ubuntu 20.04
- Eigen #
- PCL (Point Cloud Library) #

Next, clone and compile the HeLiPR-Pointcloud-Toolbox repository using git as follows:

```bash
git clone https://github.com/url-kaist/HeLiMOS-PointCloud-Toolbox.git
cd HeLiMOS-Pointcloud-Toolbox
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j 16 
```

---

## Overview 

The program is mainly composed of three modules: `helimos_saver`, `helimos_merger`, and `helimos_propagator`.   Additionally, 

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
All point clouds are copyrighted by SNU RPM Labs and MOS labels are copyrighted by KAIST Urban Robotics Lab, and those are distributed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 License. This license requires proper attribution to the author for any use, prohibits commercial usage, and mandates that derivative works be licensed similarly.

## Maintainer

- Seoyeon Jang (9uantum01 `at` kaist `dot` ac `dot` kr)
- Hyungtae Lim (shapelim `at` mit `dot` edu)