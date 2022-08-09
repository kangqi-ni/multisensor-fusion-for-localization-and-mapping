#include <ros/ros.h>
#include "glog/logging.h"

#include "multisensor_mapping/global_definition/global_definition.h"
#include "multisensor_mapping/mapping/front_end/front_end_flow.hpp"

using namespace multisensor_mapping;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    // subscribe to
    // a. undistorted Velodyne point cloud
    // publish
    // a. front end odometry
    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, cloud_topic, odom_topic);

    // rate is set to 100 Hz but actually runs below 20 Hz
    // set a higher rate to run front end at fastest rate possible
    ros::Rate rate(100); 
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}