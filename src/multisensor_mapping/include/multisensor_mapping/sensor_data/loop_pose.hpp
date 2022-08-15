#ifndef MULTISENSOR_MAPPING_SENSOR_DATA_LOOP_POSE_HPP_
#define MULTISENSOR_MAPPING_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace multisensor_mapping {
class LoopPose {
  public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif