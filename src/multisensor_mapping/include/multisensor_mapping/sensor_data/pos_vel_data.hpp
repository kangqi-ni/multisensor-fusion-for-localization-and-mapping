#ifndef MULTISENSOR_MAPPING_SENSOR_DATA_POS_VEL_DATA_HPP_
#define MULTISENSOR_MAPPING_SENSOR_DATA_POS_VEL_DATA_HPP_

#include <string>

#include <Eigen/Dense>

namespace multisensor_mapping {

class PosVelData {
  public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

} // namespace multisensor_mapping

#endif // MULTISENSOR_MAPPING_SENSOR_DATA_POS_VEL_DATA_HPP_