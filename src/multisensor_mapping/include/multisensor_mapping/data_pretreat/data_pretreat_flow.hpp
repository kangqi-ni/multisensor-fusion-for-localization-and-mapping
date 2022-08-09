#ifndef MULTISENSOR_MAPPING_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define MULTISENSOR_MAPPING_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "multisensor_mapping/subscriber/cloud_subscriber.hpp"
#include "multisensor_mapping/subscriber/imu_subscriber.hpp"
#include "multisensor_mapping/subscriber/velocity_subscriber.hpp"
#include "multisensor_mapping/subscriber/gnss_subscriber.hpp"
#include "multisensor_mapping/tf_listener/tf_listener.hpp"

// publisher
// a. synced lidar measurement
#include "multisensor_mapping/publisher/cloud_publisher.hpp"
// b. synced IMU measurement
#include "multisensor_mapping/publisher/imu_publisher.hpp"
// c. synced GNSS-odo measurement
#include "multisensor_mapping/publisher/pos_vel_publisher.hpp"
// d. synced reference trajectory
#include "multisensor_mapping/publisher/odometry_publisher.hpp"

// models
#include "multisensor_mapping/models/scan_adjust/distortion_adjust.hpp"

namespace multisensor_mapping {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    std::shared_ptr<PosVelPublisher> pos_vel_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    // extrinsics
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    // data buffer
    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    // lastest data
    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    PosVelData pos_vel_;
    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif