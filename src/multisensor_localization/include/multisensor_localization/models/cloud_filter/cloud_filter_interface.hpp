#ifndef MULTISENSOR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define MULTISENSOR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "multisensor_localization/sensor_data/cloud_data.hpp"

namespace multisensor_localization {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif