#include "multisensor_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace multisensor_localization {

void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}

}