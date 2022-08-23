#include <ros/ros.h>
#include "glog/logging.h"

#include "multisensor_localization/global_definition/global_definition.h"
#include "multisensor_localization/matching/front_end/matching_flow.hpp"

using namespace multisensor_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lio_matching_node");
    ros::NodeHandle nh;

    // subscribe
    //     a. undistorted point cloud
    //     b. GNSS position

    // publish
    //     a. relative pose estimation
    //     b. map matching estimation
    // this provides initial guess to sliding window backend
    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}