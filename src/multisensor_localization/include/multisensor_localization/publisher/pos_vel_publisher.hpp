#ifndef MULTISENSOR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_HPP_
#define MULTISENSOR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include "multisensor_localization/sensor_data/pos_vel_data.hpp"

#include "multisensor_localization/PosVel.h"

namespace multisensor_localization {

class PosVelPublisher {
  public:
    PosVelPublisher(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      std::string base_frame_id,
      std::string child_frame_id,
      int buff_size
    );
    PosVelPublisher() = default;

    void Publish(const PosVelData &pos_vel_data, const double &time);
    void Publish(const PosVelData &pos_vel_data);

    bool HasSubscribers();

  private:
    void PublishData(
      const PosVelData &pos_vel_data, 
      ros::Time time
    );

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    PosVel pos_vel_msg_;
};

} // namespace multisensor_localization

#endif // MULTISENSOR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_HPP_