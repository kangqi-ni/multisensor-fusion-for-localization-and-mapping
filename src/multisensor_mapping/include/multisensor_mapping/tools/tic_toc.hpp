#ifndef MULTISENSOR_MAPPING_TOOLS_TIC_TOC_HPP_
#define MULTISENSOR_MAPPING_TOOLS_TIC_TOC_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace multisensor_mapping {
class TicToc {
  public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        start = std::chrono::system_clock::now();
        return elapsed_seconds.count();
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
}
#endif
