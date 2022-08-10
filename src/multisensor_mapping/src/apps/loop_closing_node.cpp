#include <ros/ros.h>
#include "glog/logging.h"

#include "multisensor_mapping/global_definition/global_definition.h"
#include "multisensor_mapping/mapping/loop_closing/loop_closing_flow.hpp"
#include <multisensor_mapping/saveScanContext.h>

using namespace multisensor_mapping;

bool save_scan_context = false;

bool SaveScanContextCb(saveScanContext::Request &request, saveScanContext::Response &response) {
    save_scan_context = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;

    // subscribe
    // a. key frame pose and corresponding GNSS/IMU pose from backend node
    // publish
    // a. loop closure detection result for backend node
    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);

    // register service for saving scan context
    ros::ServiceServer service = nh.advertiseService("save_scan_context", SaveScanContextCb);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
 
        loop_closing_flow_ptr->Run();

        if (save_scan_context) {
            save_scan_context = false;
            loop_closing_flow_ptr->Save();
        }

        rate.sleep();
    }

    return 0;
}