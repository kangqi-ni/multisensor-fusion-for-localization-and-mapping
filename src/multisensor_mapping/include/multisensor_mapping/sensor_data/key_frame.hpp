#ifndef MULTISENSOR_MAPPING_SENSOR_DATA_KEY_FRAME_HPP_
#define MULTISENSOR_MAPPING_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

#include "multisensor_mapping/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp"

namespace multisensor_mapping {

struct KeyFrame {
public:
    // timestamp
    double time = 0.0;

    // key frame ID
    unsigned int index = 0;
    
    // position & orientation
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    // velocity
    struct {
      // linear velocity
      Eigen::Vector3f v = Eigen::Vector3f::Zero();
      // angular velocity
      Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;
    // bias
    struct {
      // accelerometer
      Eigen::Vector3f accel = Eigen::Vector3f::Zero();
      // gyroscope
      Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    } bias;

    KeyFrame() {}

    explicit KeyFrame(const int vertex_id, const g2o::PRVAG &prvag) {
      // set time
      time = prvag.time;
      // set vertex ID
      index = vertex_id;
      // set states
      // position
      pose.block<3, 1>(0, 3) = prvag.pos.cast<float>();
      // orientation
      pose.block<3, 3>(0, 0) = prvag.ori.matrix().cast<float>();
      // linear velocity
      vel.v = prvag.vel.cast<float>();
      // accelerometer bias
      bias.accel = prvag.b_a.cast<float>();
      // gyroscope bias
      bias.gyro = prvag.b_g.cast<float>();
    }

    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;
};

}

#endif