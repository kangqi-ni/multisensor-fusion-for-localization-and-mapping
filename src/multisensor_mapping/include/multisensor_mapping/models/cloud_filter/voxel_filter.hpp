#ifndef MULTISENSOR_MAPPING_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define MULTISENSOR_MAPPING_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "multisensor_mapping/models/cloud_filter/cloud_filter_interface.hpp"

namespace multisensor_mapping {
class VoxelFilter: public CloudFilterInterface {
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};
}
#endif