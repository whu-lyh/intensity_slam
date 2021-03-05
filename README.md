# Intensity-SLAM Noted
## Intensity-Assisted Simultaneous Localization And Mapping

This is an noted implementation of paper "Intensity-SLAM: Intensity Assisted Localization and Mapping for Large Scale Environment"
[Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 1.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 2. Build 
### 2.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/whu-lyh/intensity_slam.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
### 2.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/open?id=18ilF7GZDg2tmT6sD5pd1RjqO0XJLn9Mv) or [KITTI sequence 07](https://drive.google.com/open?id=1VpoKm7f4es4ISQ-psp4CV3iylcA4eu0-)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by 
```
sudo apt-get install unzip 
```

And this may take a few minutes to unzip the file
```
	cd ~/Downloads
	unzip ~/Downloads/2011_09_30_0018.zip
```

### 3.3 Launch ROS
```
    roslaunch intensity_slam intensity_slam.launch
```

## 4.CodeReading
- the whole slam system is deeply related to the hardware which makes the point cloud preprocess more fancy.
- the intensity calibration module is just indicent angle normalization shown in code but more fancy in paper. Maybe here the calibration is related to the VELODYNE LASER scanner.
- the featureExtraction module is slightly different from the origin LOAM, maybe more closed to A-LOAM. without feature distribution check but more straightforward and easy to understand, more friendly to fresher.
- iscloam is more close to the loam
- update frame to map points the downsampling is done. The down sampling points are used in residual calculation,using ceres to finish a local PGO.
- normal calculation is still based on the matrix decomposition and trilinear interplation is more simplidfied in code level
- different residual blocks shared the equal weights
- transformation should be normalized, more theory support
- loop closure detection stage is same as the paper demonstrated
- inside odomEstimation function, a planer feature match module should be digged more


## 5.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

## 6. Citation
If you use this work for your research, you may want to cite
```
@article{wang2021intensity,
  title={Intensity-SLAM: Intensity Assisted Localization and Mapping for Large Scale Environment},
  author={Wang, Han and Wang, Chen and Xie, Lihua},
  journal={arXiv preprint arXiv:2102.03798},
  year={2021}
}
```
