


#include "sensors/position_data.h"
#include "sensors/sensor_fusion_ekf.h"
#include <array>
#include <android/log.h>


namespace cardboard {

PositionData::PositionData()
:
    old_position_({0.0f, 0.0f, 0.0f}),
    old_velocity_({0.0f, 0.0f, 0.0f}),
    old_acceleration_({0.0f, 0.0f, 0.0f}),
    velocity_({0.0f, 0.0f, 0.0f}),
    acceleration_({0.0f, 0.0f, 0.0f}),
    position_({0.0f, 0.0f, 0.0f}),
    log_count_(0) {
  __android_log_print(ANDROID_LOG_INFO, "Position", "Position constructor");
 }

PositionData::~PositionData() {}

std::array<float, 3> PositionData::GetPosition( Vector3 acceleration_updated_value_, Vector4 orientation_ ) {

    Vector3 acceleration_minus_gravity_= {0.0f, 0.0f, 0.0f}; 
    velocity_[0] = old_velocity_[0] + (acceleration_updated_value_[0] + old_acceleration_[0]) / 0.01f * 0.00001f;
    velocity_[1] = old_velocity_[1] + (acceleration_updated_value_[1] + old_acceleration_[1]) / 0.01f * 0.00001f;
    velocity_[2] = old_velocity_[2] + (acceleration_updated_value_[2] + old_acceleration_[2]) / 0.01f * 0.00001f;
    old_acceleration_[0] = acceleration_updated_value_[0];
    old_acceleration_[1] = acceleration_updated_value_[1];
    old_acceleration_[2] = acceleration_updated_value_[2];
    position_[0] = old_position_[0] + (velocity_[0] + old_velocity_[0]) / 0.01f * 0.00001f;
    position_[1] = old_position_[1] + (velocity_[1] + old_velocity_[1]) / 0.01f * 0.00001f;
    position_[2] = old_position_[2] + (velocity_[2] + old_velocity_[2]) / 0.01f * 0.00001f;
    old_velocity_[0] = velocity_[0];
    old_velocity_[1] = velocity_[1];
    old_velocity_[2] = velocity_[2];

    if(position_[0] < -10.0f || position_[0] > 10.0f) {
        position_[0] = 0.0f;
    }
    if(position_[1] < -10.0f || position_[1] > 10.0f) {
        position_[1] = 0.0f;
    }
    if(position_[2] < -10.0f || position_[2] > 10.0f) {
        position_[2] = 0.0f;
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