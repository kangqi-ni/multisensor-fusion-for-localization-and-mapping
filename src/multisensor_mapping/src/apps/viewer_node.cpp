#include <ros/ros.h>
#include "glog/logging.h"

#include <multisensor_mapping/saveMap.h>
#include "multisensor_mapping/global_definition/global_definition.h"
#include "multisensor_mapping/mapping/viewer/viewer_flow.hpp"

using namespace multisensor_mapping;

bool need_save_map = false;

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh, cloud_topic);

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _viewer_flow_ptr->Run();
        if (need_save_map) {
            need_save_map = false;
            _viewer_flow_ptr->SaveMap();
        }

        rate.sleep();
    }

    return 0;
}