

#ifndef CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_
#define CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_

#include "sensors/cauer_filter.h"
#include "sensors/iir_filter_4.h"

#include <array>
#include <memory>

namespace cardboard {

class PositionEstimator {
 public:
  PositionEstimator();
  virtual ~PositionEstimator();

  std::array<float, 3> GetPosition(Vector3 accelerometer_sample_, Vector4 orientation, int64_t timestamp_ns_);

 private:

  const double kThresholdAccelerationStable = 0.09;

  bool StableValueStream( double new_value_, double old_value_, double threshold );

  bool ApproximateEqual( double new_value_, double old_value_, double threshold );

  Vector3 old_accelerometer_sample_;
  Vector3 older_accelerometer_sample_;
  Vector3 even_older_accelerometer_sample_;
  Vector3 accelerometer_sample_filtered_;
  Vector4 old_orientation_;
  Vector3 old_position_;
  Vector3 old_velocity_;
  Vector3 old_acceleration_;
  Vector3 older_acceleration_;
  Vector3 position_;
  Vector3 velocity_;
  Vector3 acceleration_;

  Vector3 mean_acceleration_;

  // CauerFilter filter_jerk_;
  CauerFilter filter_accelerometer_;
  IIRFilter4 filter_velocity_;

  int64_t old_timestamp_ns_;

  int log_count_;

};

} // namespace cardboard

#endif  // CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_
