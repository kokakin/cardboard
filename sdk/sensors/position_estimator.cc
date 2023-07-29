


#include "sensors/position_estimator.h"
#include "sensors/sensor_fusion_ekf.h"
#include "util/rotation.h"
#include <array>
#include <android/log.h>


namespace cardboard {

PositionEstimator::PositionEstimator()
:
    old_position_({0.0, 0.0, 0.0}),
    old_velocity_({0.0, 0.0, 0.0}),
    old_acceleration_({0.0, 0.0, 0.0}),
    velocity_({0.0, 0.0, 0.0}),
    position_({0.0, 0.0, 0.0}),
    acceleration_({0.0, 0.0, 0.0}),
    highpass_filter_velocity_(12.0f, true),
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
    double timestamp_s_ = (static_cast<double>(timestamp_ns_) - past_timestamp_ns_) * 1.0e-9;
    if (timestamp_s_ < 0.001){
        __android_log_print(ANDROID_LOG_INFO,"TimestampTooLow", "%lf", timestamp_s_);
        position_[0] = old_position_[0];
        position_[1] = old_position_[1];
        position_[2] = old_position_[2];
        return {static_cast<float>(position_[0]), static_cast<float>(position_[1]), static_cast<float>(position_[2])};
    }

    Rotation rotation = Rotation::FromQuaternion(
     Vector4({orientation_[0], orientation_[1], orientation_[2], orientation_[3]}));
     const Vector3 acceleration_rotated_ = rotation * Vector3(acceleration_updated_value_[0], acceleration_updated_value_[1], acceleration_updated_value_[2]);

    acceleration_[0] = acceleration_rotated_[0];
    acceleration_[1] = acceleration_rotated_[1];
    acceleration_[2] = acceleration_rotated_[2];

    if(acceleration_[0] < 0.05 && acceleration_[0] > -0.05) {
        acceleration_[0] = 0.0;
    }
    if(acceleration_[1] < 0.05 && acceleration_[1] > -0.05) {
        acceleration_[1] = 0.0;
    }
    if(acceleration_[2] < 0.05 && acceleration_[2] > -0.05) {
        acceleration_[2] = 0.0;
    }

    // elliptic_filter_velocity_
    past_timestamp_ns_ = timestamp_ns_;
    velocity_[0] = old_velocity_[0] + (acceleration_[0] + old_acceleration_[0]) / (2 * timestamp_s_) * 0.001;
    velocity_[1] = old_velocity_[1] + (acceleration_[1] + old_acceleration_[1]) / (2 * timestamp_s_) * 0.001;
    velocity_[2] = old_velocity_[2] + (acceleration_[2] + old_acceleration_[2]) / (2 * timestamp_s_) * 0.001;
    highpass_filter_velocity_.AddSample({velocity_[0], velocity_[1], velocity_[2]}, timestamp_ns_);
    Vector3 velocity_filtered_ = highpass_filter_velocity_.GetFilteredData();
    old_acceleration_[0] = acceleration_[0];
    old_acceleration_[1] = acceleration_[1];
    old_acceleration_[2] = acceleration_[2];
    position_[0] = old_position_[0] + (velocity_filtered_[0] + old_velocity_[0]) / (2 * timestamp_s_) * 0.3;
    position_[1] = old_position_[1] + (velocity_filtered_[1] + old_velocity_[1]) / (2 * timestamp_s_) * 0.3;
    position_[2] = old_position_[2] + (velocity_filtered_[2] + old_velocity_[2]) / (2 * timestamp_s_) * 0.3;
    old_velocity_[0] = velocity_filtered_[0];
    old_velocity_[1] = velocity_filtered_[1];
    old_velocity_[2] = velocity_filtered_[2];

    old_position_[0] = position_[0];
    old_position_[1] = position_[1];
    old_position_[2] = position_[2];

    log_count_++;
    if (log_count_ > 60) {
    __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+.4lf, %+.4lf, %+.4lf", acceleration_updated_value_[0], acceleration_updated_value_[1], acceleration_updated_value_[2]);
    __android_log_print(ANDROID_LOG_INFO, "Orientation", "%+.4f, %+.4f, %+.4f, %+.4f", orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
    __android_log_print(ANDROID_LOG_INFO, "Acceleration_Rot", "%+.4lf, %+.4lf, %+.4lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Velocity", "%lf, %lf, %lf", velocity_[0], velocity_[1], velocity_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Position", "%lf, %lf, %lf", position_[0], position_[1], position_[2]);
    log_count_ = 0;
    }

    if(position_[0] < -5 || position_[0] > 5) {
        position_[0] = 0.0;
        old_position_[0] = 0.0;
        old_velocity_[0] = 0.0;
    }
    if(position_[1] < -5 || position_[1] > 5) {
        position_[1] = 0.0;
        old_position_[1] = 0.0;
        old_velocity_[1] = 0.0;
    }
    if(position_[2] < -5 || position_[2] > 5) {
        position_[2] = 0.0;
        old_position_[2] = 0.0;
        old_velocity_[2] = 0.0;
    }

    return {static_cast<float>(position_[0]), static_cast<float>(position_[1]), static_cast<float>(position_[2])};
}

} // namespace cardboard