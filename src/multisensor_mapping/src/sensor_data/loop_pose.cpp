#include "multisensor_mapping/sensor_data/loop_pose.hpp"

namespace multisensor_mapping {
Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}