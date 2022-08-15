#ifndef MULTISENSOR_MAPPING_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_
#define MULTISENSOR_MAPPING_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "multisensor_mapping/PosVel.h"
#include "multisensor_mapping/sensor_data/pos_vel_data.hpp"

namespace multisensor_mapping {

class PosVelSubscriber {
  public:
    PosVelSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    PosVelSubscriber() = default;
    void ParseData(std::deque<PosVelData>& pos_vel_data_buff);

  private:
    void msg_callback(const PosVelConstPtr& pos_vel_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PosVelData> new_pos_vel_data_;

    std::mutex buff_mutex_; 
};

} // namespace multisensor_mapping

#endif // MULTISENSOR_MAPPING_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_