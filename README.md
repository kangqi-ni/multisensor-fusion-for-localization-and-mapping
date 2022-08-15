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

## Multisensor Mapping
This package implements mutisensor mapping using LiDAR, IMU, GNSS, and odometer measurements to build a map and evaluates performance on the KITTI dataset.<br>
The frontend uses NDT CPU from Autoware to provide LiDAR poses. The backend fuses LiDAR relative poses, IMU and odometer preintegrations, GNSS prior positions, and loop closure relative poses as optimization constraints. Since mapping does not need to run in real-time, a sliding window is not implemented here. <br>

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

The experiments show the obvious improvement of multisensor fusion.
