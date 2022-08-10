#ifndef MULTISENSOR_MAPPING_MAPPING_VIEWER_VIEWER_HPP_
#define MULTISENSOR_MAPPING_MAPPING_VIEWER_VIEWER_HPP_

#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "multisensor_mapping/sensor_data/cloud_data.hpp"
#include "multisensor_mapping/sensor_data/key_frame.hpp"
#include "multisensor_mapping/sensor_data/pose_data.hpp"
#include "multisensor_mapping/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_mapping {
class Viewer {
  public:
    Viewer();

    bool UpdateWithOptimizedKeyFrames(std::deque<KeyFrame>& optimized_key_frames);
    bool UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                               PoseData transformed_data,
                               CloudData cloud_data);

    bool SaveMap();

    Eigen::Matrix4f& GetCurrentPose();
    CloudData::CLOUD_PTR& GetCurrentScan();
    bool GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool GetGlobalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool HasNewLocalMap();
    bool HasNewGlobalMap();

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, 
                    std::shared_ptr<CloudFilterInterface>& filter_ptr, 
                    const YAML::Node& config_node);

    bool OptimizeKeyFrames();
    bool JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool JointCloudMap(const std::deque<KeyFrame>& key_frames, 
                             CloudData::CLOUD_PTR& map_cloud_ptr);

  private:
    std::string data_path_ = "";
    int local_frame_num_ = 20;

    std::string key_frames_path_ = "";
    std::string map_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
    PoseData optimized_odom_;
    CloudData optimized_cloud_;
    // only key frames with optimized poses
    std::deque<KeyFrame> optimized_key_frames_;
    // all key frames after applying optimizated transformations
    std::deque<KeyFrame> all_key_frames_; 

    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};
}

#endif