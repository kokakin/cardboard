


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
    old_orientation_({0.0, 0.0, 0.0, 0.0}),
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
        old_orientation_ = orientation_;
        return {0.0f, 0.0f, 0.0f};
    }
    double timestamp_s_ = (static_cast<double>(timestamp_ns_) - past_timestamp_ns_) * 1.0e-9;
    if (timestamp_s_ < 0.001){
        __android_log_print(ANDROID_LOG_INFO,"TimestampTooLow", "%lf", timestamp_s_);
        return {static_cast<float>(old_position_[0]), static_cast<float>(old_position_[1]), static_cast<float>(old_position_[2])};
    }

    Rotation rotation = Rotation::FromQuaternion(orientation_);
    
    acceleration_ = rotation * Vector3(-acceleration_updated_value_[2], acceleration_updated_value_[1], acceleration_updated_value_[0]);

    if(acceleration_[0] < 0.02 && acceleration_[0] > -0.02) {
        acceleration_[0] = 0.0;
    }
    if(acceleration_[1] < 0.02 && acceleration_[1] > -0.02) {
        acceleration_[1] = 0.0;
    }
    if(acceleration_[2] < 0.02 && acceleration_[2] > -0.02) {
        acceleration_[2] = 0.0;
    }

    velocity_ = old_velocity_ + (acceleration_ + old_acceleration_) / (2 * timestamp_s_) * 0.001;

    highpass_filter_velocity_.AddSample(velocity_, timestamp_ns_);
    Vector3 velocity_filtered_ = highpass_filter_velocity_.GetFilteredData();
    
    position_ = old_position_ + (velocity_filtered_ + old_velocity_) / (2 * timestamp_s_) * 0.3;

    if (((std::abs(orientation_[0]) > 0.02 && std::abs(old_orientation_[0]) > 0.02) && 
        ((std::abs(orientation_[0]) + std::abs(old_orientation_[0]))/(2*std::abs(old_orientation_[0])) > 1.015 ||
         (std::abs(orientation_[0]) + std::abs(old_orientation_[0]))/(2*std::abs(old_orientation_[0])) < 0.985)) || 
        ((std::abs(orientation_[1]) > 0.02 && std::abs(old_orientation_[1]) > 0.02) && 
        ((std::abs(orientation_[1]) + std::abs(old_orientation_[1]))/(2*std::abs(old_orientation_[1])) > 1.015 ||
         (std::abs(orientation_[1]) + std::abs(old_orientation_[1]))/(2*std::abs(old_orientation_[1])) < 0.985)) || 
        ((std::abs(orientation_[2]) > 0.02 && std::abs(old_orientation_[2]) > 0.02) && 
        ((std::abs(orientation_[2]) + std::abs(old_orientation_[2]))/(2*std::abs(old_orientation_[2])) > 1.015 || 
         (std::abs(orientation_[2]) + std::abs(old_orientation_[2]))/(2*std::abs(old_orientation_[2])) < 0.985)) || 
        ((std::abs(orientation_[3]) > 0.02 && std::abs(old_orientation_[3]) > 0.02) && 
        ((std::abs(orientation_[3]) + std::abs(old_orientation_[3]))/(2*std::abs(old_orientation_[3])) > 1.015 || 
         (std::abs(orientation_[3]) + std::abs(old_orientation_[3]))/(2*std::abs(old_orientation_[3])) < 0.985)))  {
        __android_log_print(ANDROID_LOG_INFO, "OrientationChanged", "%+.3lf", (std::abs(orientation_[0]) + std::abs(old_orientation_[0]))/(2*std::abs(old_orientation_[0])));
        old_orientation_ = orientation_;
        return {static_cast<float>(old_position_[0]), static_cast<float>(old_position_[1]), static_cast<float>(old_position_[2])};
    }
    old_acceleration_ = acceleration_;
    old_velocity_ = velocity_filtered_;
    old_position_ = position_;
    old_orientation_ = orientation_;
    past_timestamp_ns_ = timestamp_ns_;

    // log_count_++;
    // if (log_count_ > 30) {
    // log_count_ = 0;
    // // // __android_log_print(ANDROID_LOG_INFO, "AccelFixBase", "%+.4lf, %+.4lf, %+.4lf", acceleration_updated_value_[0], acceleration_updated_value_[1], acceleration_updated_value_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+.5lf, %+.5lf, %+.5lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Velocity", "%+.5lf, %+.5lf, %+.5lf", velocity_[0], velocity_[1], velocity_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "VelocityF", "%+.5lf, %+.5lf, %+.5lf", velocity_filtered_[0], velocity_filtered_[1], velocity_filtered_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Position", "%+.5lf, %+.5lf, %+.5lf", position_[0], position_[1], position_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Orientation", "%+.5f, %+.5f, %+.5f, %+.5f", orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
    // // __android_log_print(ANDROID_LOG_INFO, "Rotation", "%+.5lf, %+.5lf, %+.5lf, %+.5lf", rotation.GetQuaternion()[0], rotation.GetQuaternion()[1], rotation.GetQuaternion()[2], rotation.GetQuaternion()[3]);
    // }

    if(position_[0] < -5 || position_[0] > 5) {
        position_[0] = 0.0;
        old_acceleration_[0] = 0.0;
        old_velocity_[0] = 0.0;
        old_position_[0] = 0.0;
    }
    if(position_[1] < -5 || position_[1] > 0) {
        position_[1] = 0.0;
        old_acceleration_[1] = 0.0;
        old_velocity_[1] = 0.0;
        old_position_[1] = 0.0;
    }
    if(position_[2] < -5 || position_[2] > 5) {
        position_[2] = 0.0;
        old_acceleration_[2] = 0.0;
        old_velocity_[2] = 0.0;
        old_position_[2] = 0.0;
    }

    return {static_cast<float>(position_[0]), static_cast<float>(position_[1]), static_cast<float>(position_[2])};
}

} // namespace cardboard