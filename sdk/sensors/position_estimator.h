

#ifndef CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_
#define CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_

#include "sensors/highpass_filter.h"
// #include "sensors/dsp-filters/Elliptic.h"

#include <array>
#include <memory>

namespace cardboard {

class PositionEstimator {
 public:
  PositionEstimator();
  virtual ~PositionEstimator();

  std::array<float, 3> GetPosition(Vector3 acceleration_updated_value, Vector4 orientation, int64_t timestamp_ns_);

 private:
  std::array<double, 3> old_position_;
  std::array<double, 3> old_velocity_;
  std::array<double, 3> old_acceleration_;
  std::array<double, 3> position_;
  std::array<double, 3> velocity_;
  std::array<double, 3> acceleration_;
  std::array<float, 3> position_rotated_;

  HighpassFilter highpass_filter_velocity_;
  // Dsp::Elliptic::AnalogLowPass elliptic_filter_velocity_;

  int64_t past_timestamp_ns_;

  int log_count_;

};

} // namespace cardboard

#endif  // CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_
