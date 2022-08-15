#include <iostream>

#include "multisensor_mapping/tools/print_info.hpp"
#include "glog/logging.h"

namespace multisensor_mapping {
void PrintInfo::PrintPose(std::string head, Eigen::Matrix4f pose) {
    Eigen::Affine3f aff_pose;
    aff_pose.matrix() = pose;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(aff_pose, x, y, z, roll, pitch, yaw);
    std::cout << head
              << x << "," << y << "," << z << "," 
              << roll * 180 / M_PI << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI
              << std::endl;
}
}