#include <ros/ros.h>
#include "glog/logging.h"

#include <multisensor_mapping/optimizeMap.h>
#include "multisensor_mapping/global_definition/global_definition.h"
#include "multisensor_mapping/mapping/back_end/lio_back_end_flow.hpp"

using namespace multisensor_mapping;

bool optimize_map = false;

bool OptimizeMapCallback(
    optimizeMap::Request &request, 
    optimizeMap::Response &response) {
    optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lio_back_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    // subscribe
    // a. undistorted Velodyne point cloud
    // b. lidar pose in map frame
    // c. lidar odometry estimation
    // d. loop close pose
    // publish
    // a. lidar odometry in map frame
    // b. key frame pose and corresponding GNSS/IMU pose
    // c. optimized key frame sequence as trajectory
    std::shared_ptr<LIOBackEndFlow> lio_back_end_flow_ptr = std::make_shared<LIOBackEndFlow>(
        nh, cloud_topic, odom_topic
    );
    ros::ServiceServer service = nh.advertiseService(
        "optimize_map", OptimizeMapCallback
    );

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        lio_back_end_flow_ptr->Run();

        if (optimize_map) {
            lio_back_end_flow_ptr->ForceOptimize();
            lio_back_end_flow_ptr->SaveOptimizedOdometry();
            
            optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}