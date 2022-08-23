#ifndef MULTISENSOR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_HPP_
#define MULTISENSOR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace multisensor_localization {

class PreIntegrator {
public:
    /**
     * @brief  whether the pre-integrator is inited
     * @return true if inited false otherwise   
     */
    double IsInited(void) const { return is_inited_; }

    /**
     * @brief  get pre-integrator time
     * @return pre-integrator time as double    
     */
    double GetTime(void) const { return time_; }
    
protected:
    PreIntegrator() {}

    bool is_inited_ = false;

    double time_;
};

} // namespace multisensor_localization

#endif // MULTISENSOR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_HPP_