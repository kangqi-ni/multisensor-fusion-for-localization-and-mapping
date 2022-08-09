#include <ros/ros.h>
#include "glog/logging.h"

#include "multisensor_mapping/global_definition/global_definition.h"
#include "multisensor_mapping/data_pretreat/data_pretreat_flow.hpp"

using namespace multisensor_mapping;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    // subscribe to
    // a. raw Velodyne measurement
    // b. raw GNSS/IMU measurement
    // publish
    // a. undistorted point cloud
    // b. lidar pose in map frame from GNSS and IMU
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100 Hz
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}