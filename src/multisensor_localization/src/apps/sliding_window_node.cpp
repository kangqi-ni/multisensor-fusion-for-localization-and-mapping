#include <ros/ros.h>

#include "multisensor_localization/global_definition/global_definition.h"
#include "multisensor_localization/matching/back_end/sliding_window_flow.hpp"
#include "multisensor_localization/saveOdometry.h"

#include "glog/logging.h"

using namespace multisensor_localization;

bool save_odometry = false;

bool SaveOdometryCb(saveOdometry::Request &request, saveOdometry::Response &response) {
    save_odometry = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "sliding_window_node");
    ros::NodeHandle nh;

    // subscribe
    // a. lidar odometry
    // b. map matching odometry

    // publish
    // a. optimized key frame sequence as trajectory
    std::shared_ptr<SlidingWindowFlow> sliding_window_flow_ptr = std::make_shared<SlidingWindowFlow>(nh);

    // register service for optimized trajectory save
    ros::ServiceServer service = nh.advertiseService("save_odometry", SaveOdometryCb);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        sliding_window_flow_ptr->Run();

        if (save_odometry) {
            save_odometry = false;
            sliding_window_flow_ptr->SaveOptimizedTrajectory();
        }

        rate.sleep();
    }

    return 0;
}