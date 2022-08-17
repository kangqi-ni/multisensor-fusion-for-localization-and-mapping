#include "multisensor_localization/data_pretreat/data_pretreat_flow.hpp"
#include "multisensor_localization/global_definition/global_definition.h"

#include "glog/logging.h"

namespace multisensor_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscribers
    // a. velodyne point cloud
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    // b. OXTS IMU
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    // c. OXTS velocity
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    // d. OXTS GNSS
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    // e. extrinsics
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    // publishers
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "map", "/imu_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "map", "map", 100);

    // motion compensation for point cloud
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    // read synced data from buffers
    if (!ReadData())
        return false;

    // read lidar-imu extrinsics
    if (!InitCalibration()) 
        return false;

    // initialize gnss origin
    if (!InitGNSS())
        return false;

    while(HasData()) {
        // get synced latest data
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    // fetch data from buffers
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0)
        return false;

    // use timestamp of lidar measurement as reference
    double cloud_time = cloud_data_buff_.front().time;

    // sync IMU, velocity and GNSS with lidar measurement
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    // only mark lidar as initialized when all the three sensors are synced
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            // pop point cloud
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration() {
    // lookup imu pose in lidar frame
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    // get latest synced data
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;

    // assume the frequency of lidar is 10Hz

    // pop point cloud if point cloud too early 
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    // pop imu if imu too late
    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    // pop velocity if velocity too late
    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    // pop gnss if gnss too late
    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    // pop latest synced data
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    // get reference pose
    gnss_pose_ = Eigen::Matrix4f::Identity();
    // get position from GNSS
    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    // get orientation from IMU
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();

    // compute lidar pose in GNSS/map frame (T_map_lidar = T_map_imu * T_imu_lidar)
    gnss_pose_ *= lidar_to_imu_;

    // set synced pos vel
    pos_vel_.pos.x() = current_gnss_data_.local_E;
    pos_vel_.pos.y() = current_gnss_data_.local_N;
    pos_vel_.pos.z() = current_gnss_data_.local_U;

    pos_vel_.vel.x() = current_velocity_data_.linear_velocity.x;
    pos_vel_.vel.y() = current_velocity_data_.linear_velocity.y;
    pos_vel_.vel.z() = current_velocity_data_.linear_velocity.z;

    // convert velocity from imu frame to lidar frame
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);

    // motion compensation for lidar measurements
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.time);

    pos_vel_pub_ptr_->Publish(pos_vel_, current_cloud_data_.time);
    
    // synced odometry has the following info:
    // a. lidar frame's pose in map
    // b. lidar frame's velocity 
    gnss_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, current_cloud_data_.time);

    return true;
}

}