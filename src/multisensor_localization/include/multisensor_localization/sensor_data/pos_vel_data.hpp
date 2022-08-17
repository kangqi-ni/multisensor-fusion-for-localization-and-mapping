#ifndef MULTISENSOR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_
#define MULTISENSOR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_

#include <string>

#include <Eigen/Dense>

namespace multisensor_localization {

class PosVelData {
  public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

} // namespace multisensor_localization

#endif // MULTISENSOR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_