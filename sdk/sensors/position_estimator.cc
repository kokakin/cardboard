


#include "sensors/position_estimator.h"
#include "sensors/sensor_fusion_ekf.h"
#include <array>
#include <android/log.h>


namespace cardboard {

PositionEstimator::PositionEstimator()
:
    old_position_({0.0f, 0.0f, 0.0f}),
    old_velocity_({0.0f, 0.0f, 0.0f}),
    old_acceleration_({0.0f, 0.0f, 0.0f}),
    velocity_({0.0f, 0.0f, 0.0f}),
    position_({0.0f, 0.0f, 0.0f}),
    highpass_filter_velocity_(12.0f),
    log_count_(0),
    past_timestamp_ns_(0) {
  __android_log_print(ANDROID_LOG_INFO, "Position", "Position constructor");
 }

PositionEstimator::~PositionEstimator() {}

std::array<float, 3> PositionEstimator::GetPosition( Vector3 acceleration_updated_value_, Vector4 orientation_, int64_t timestamp_ns_ ) {

    if(past_timestamp_ns_ == 0) {
        past_timestamp_ns_ = timestamp_ns_;
        return {0.0f, 0.0f, 0.0f};
    }
    float timestamp_s_ = (static_cast<float>(timestamp_ns_) - past_timestamp_ns_) * 1.0e-9;
    if (timestamp_s_ < 0.000001){
        return old_position_;
    }
    past_timestamp_ns_ = timestamp_ns_;
    velocity_[0] = old_velocity_[0] + (acceleration_updated_value_[0] + old_acceleration_[0]) / timestamp_s_ * 0.1f;
    velocity_[1] = old_velocity_[1] + (acceleration_updated_value_[1] + old_acceleration_[1]) / timestamp_s_ * 0.1f;
    velocity_[2] = old_velocity_[2] + (acceleration_updated_value_[2] + old_acceleration_[2]) / timestamp_s_ * 0.1f;
    // highpass_filter_velocity_.AddSample({velocity_[0], velocity_[1], velocity_[2]}, timestamp_ns_);
    // Vector3 velocity_filtered_ = highpass_filter_velocity_.GetFilteredData();
    old_acceleration_[0] = acceleration_updated_value_[0];
    old_acceleration_[1] = acceleration_updated_value_[1];
    old_acceleration_[2] = acceleration_updated_value_[2];
    position_[0] = old_position_[0] + (velocity_[0] + old_velocity_[0]) / timestamp_s_ * 0.01f;
    position_[1] = old_position_[1] + (velocity_[1] + old_velocity_[1]) / timestamp_s_ * 0.01f;
    position_[2] = old_position_[2] + (velocity_[2] + old_velocity_[2]) / timestamp_s_ * 0.01f;
    old_velocity_[0] = velocity_[0];
    old_velocity_[1] = velocity_[1];
    old_velocity_[2] = velocity_[2];

    if(position_[0] < -3.0f || position_[0] > 3.0f) {
        position_[0] = 0.0f;
        old_position_[0] = 0.0f;
        old_velocity_[0] = 0.0f;
    }
    if(position_[1] < -3.0f || position_[1] > 3.0f) {
        position_[1] = 0.0f;
        old_position_[1] = 0.0f;
        old_velocity_[1] = 0.0f;
    }
    if(position_[2] < -3.0f || position_[2] > 3.0f) {
        position_[2] = 0.0f;
        old_position_[2] = 0.0f;
        old_velocity_[2] = 0.0f;
    }
    old_position_[0] = position_[0];
    old_position_[1] = position_[1];
    old_position_[2] = position_[2];

    log_count_++;
    if (log_count_ > 30) {
    __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%f, %f, %f", acceleration_updated_value_[0], acceleration_updated_value_[1], acceleration_updated_value_[2]);
    __android_log_print(ANDROID_LOG_INFO, "Velocity", "%f, %f, %f", velocity_[0], velocity_[1], velocity_[2]);
    __android_log_print(ANDROID_LOG_INFO, "Position", "%f, %f, %f", position_[0], position_[1], position_[2]);
    __android_log_print(ANDROID_LOG_INFO, "Orientation", "orientation: %f, %f, %f, %f", orientation_[0], orientation_[1], orientation_[2]);
    log_count_ = 0;
    }
  

  return position_;
}

} // namespace cardboard