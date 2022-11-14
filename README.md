# Multisensor Fusion for Localization and Mapping
This project aims to provide a open-sourced framework for multisensor fusion in localization and mapping. <br>
Install the following dependencies to run this project: <br>
1. ROS
2. g2o
3. GeographlicLib
4. glog
5. PCL
6. protobuf
7. sophus
8. YAML

## Dataset
This is a modified kitti rosbag: https://drive.google.com/file/d/1zoc7hm5nZvv0koZVPxt19J5d-TOxVfK1/view?usp=sharing <br>
The original kitti dataset has many missing parts in the raw data. The missing part has been linearly interpolated in this rosbag for the purpose of multisensor fusion. However, linear interpolation does not reflect how the actual missing data are like. <br>

## Multisensor Mapping
This package implements multisensor mapping using LiDAR, IMU, GNSS, and odometer measurements to build a map and evaluates performance on the KITTI dataset.<br>
The frontend module uses NDT CPU from Autoware to provide LiDAR poses. The loop closure module uses Scan Context to detect loop closures. The backend fuses LiDAR relative poses, IMU and odometer preintegrations, GNSS prior positions, and loop closure relative poses as optimization constraints. Since mapping does not need to run in real-time, a sliding window is not implemented here. <br>

### Map
![map](https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/map.png)

### LiDAR Only Mapping
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/laser_mapping_ape.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/laser_mapping_ape_raw.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/laser_mapping_ape_map.png">

### Multisensor Fusion Mapping
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/fusion_mapping_ape.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/fusion_mapping_ape_raw.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/fusion_mapping_ape_map.png">

The experiments show obvious improvement of multisensor fusion on mapping accuracy.

### TODO
1. Integrate gtsam-based factor graph optimization as mapping backend.



## Multisensor Localization
This package implements multisensor localization using LiDAR, IMU, GNSS, and odometry measurements to localize within a prebuilt map and evaluates performance on the KITTI dataset. <br>
The frontend uses NDT CPU from Autoware to provide LiDAR poses and adds Scan Context to detect loop closures. The backend fuses LiDAR relative poses after loop closures, IMU preintegrations, and GNSS prior poses as optimization constraints. A sliding window is implemented to margalize old measurements as prior and speed up optimization. <br>

### Trajectory
![trajectory](https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/trajectory.png)

### LiDAR Only Localization
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/laser_localization_ape.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/laser_localization_ape_raw.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/laser_localization_ape_map.png">

### Multisensor Fusion Localization
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/fusion_localization_ape.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/fusion_localization_ape_raw.png">
<img src="https://github.com/kangqi-ni/multisensor_fusion_for_localization_and_mapping/blob/master/docs/fusion_localization_ape_map.png">

The experiments show obvious improvement of multisensor fusion on localization accuracy.

### TODO
1. Integrate Kalman Filter, Extended Kalman Filter, or Error-State Kalman Filte as localization backend.
2. Integrate gtsam-based factor graph optimization as localization backend.



## References 
[1] Autoware Foundation. (2020). NDT_CPU. https://github.com/autowarefoundation/autoware_ai_perception/tree/master/ndt_cpu. <br>
[2] Qian Ren. (2020). Localization in Auto Driving. https://github.com/Little-Potato-1990/localization_in_auto_driving. <br>
[3] Tong Qin. (2019). VINS-Mono. https://github.com/HKUST-Aerial-Robotics/VINS-Mono. <br>
[4] Yao Ge. (2021). Sensor Fusion for Localization CourseWare. https://github.com/AlexGeControl/Sensor-Fusion-for-Localization-Courseware. <br>
