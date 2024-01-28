

#ifndef CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_
#define CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_

// #include "sensors/iir_filter_4.h"

#include <array>
#include <memory>

#include "util/vector.h"

namespace cardboard
{

  class PositionEstimator
  {
  public:
    PositionEstimator();
    virtual ~PositionEstimator();

    std::array<float, 3> GetPosition(Vector3 accelerometer_sample_, Vector4 orientation, int64_t timestamp_ns_);

  private:
    // Simulation
    // const double kThresholdSignal = 0.3;
    // const double kThresholdAccelerationStable = 0.05;
    // const double kThresholdVelocityBias = 0.01;

    const double kThresholdSignal = 0.35;
    const double kThresholdAccelerationStable = 0.25;
    const double kThresholdVelocityBias = 0.25;
    const double sampling_velocity_time = 0.3;

    bool ApproximateEqual(double new_value_, double old_value_, double threshold);

    Vector3 old_acceleration_;
    Vector3 older_acceleration_;
    Vector3 even_older_acceleration_;
    Vector3 e2_older_acceleration_;

    Vector3 accelerometer_sample_filtered_;
    Vector3 acceleration_;

    Vector3 velocity_;
    Vector3 old_velocity_;
    Vector3 e2_older_velocity_;

    Vector3 old_position_;
    Vector3 position_;

    double gravity_acceleration_;
    double gravity_acceleration_old_;

    double sampling_velocity_;

    // IIRFilter4 filter_velocity_;

    int64_t old_timestamp_ns_;

    int log_count_;
  };

} // namespace cardboard

#endif // CARDBOARD_SDK_SENSORS_POSITION_ESTIMATOR_H_
