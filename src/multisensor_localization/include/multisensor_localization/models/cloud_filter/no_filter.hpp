#ifndef MULTISENSOR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define MULTISENSOR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "multisensor_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace multisensor_localization {

class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};

} // namespace multisensor_localization

#endif // MULTISENSOR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_