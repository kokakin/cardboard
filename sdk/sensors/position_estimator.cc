


#include "sensors/position_estimator.h"
#include "sensors/sensor_fusion_ekf.h"
#include "util/rotation.h"
#include <array>
#include <android/log.h>


namespace cardboard {

PositionEstimator::PositionEstimator()
:
    old_accelerometer_sample_({0.0, 0.0, 0.0}),
    older_accelerometer_sample_({0.0, 0.0, 0.0}),
    even_older_accelerometer_sample_({0.0, 0.0, 0.0}),
    old_velocity_({0.0, 0.0, 0.0}),
    older_velocity_({0.0, 0.0, 0.0}),
    even_older_velocity_({0.0, 0.0, 0.0}),
    accelerometer_sample_filtered_({0.0, 0.0, 0.0}),
    old_position_({0.0, 0.0, 0.0}),
    velocity_({0.0, 0.0, 0.0}),
    position_({0.0, 0.0, 0.0}),
    acceleration_({0.0, 0.0, 0.0}),
    filter_velocity_(
        {0.07018592810153744, 
         0.2385278285406635, 
         0.3402517160736358,
         0.2385278285406635,
         0.07018592810153743
        }, {
         1,
         -0.7055230116026355, 
         1.084324954301363,
         -0.5409646222138151,
         0.2366964481564892}),
    log_count_(0),
    old_timestamp_ns_(0) {
        filter_velocity_.Reset();
  __android_log_print(ANDROID_LOG_INFO, "Position", "Position constructor");
 }

PositionEstimator::~PositionEstimator() {}

// Only works for small same sign differences ie doesnt work for 0.1 and -0.1
bool PositionEstimator::ApproximateEqual( double new_value_, double old_value_, double threshold) {
    if( std::abs( std::abs( new_value_ ) - std::abs( old_value_ ) ) > threshold) {
        return false;
    }
    return true;
}

std::array<float, 3> PositionEstimator::GetPosition( Vector3 accelerometer_sample_xyz_, Vector4 orientation_, int64_t timestamp_ns_ ) {

    const Rotation rotation_ = Rotation::FromQuaternion(orientation_);
    const Rotation anti_rotation_ = Rotation::FromQuaternion(Vector4(-orientation_[0], -orientation_[1], -orientation_[2], orientation_[3]));
    
    const Vector3 accelerometer_sample_raw_ = Vector3(-accelerometer_sample_xyz_[1], accelerometer_sample_xyz_[0], accelerometer_sample_xyz_[2]);
    const Vector3 gravity_acceleration_ = rotation_ * Vector3(0.0, 9.87516301, 0.0);
    const Vector3 accelerometer_sample_ = anti_rotation_ * (accelerometer_sample_raw_ - gravity_acceleration_);

    if(old_timestamp_ns_ == 0.0) {
        old_timestamp_ns_ = timestamp_ns_;
        return {0.0f, 0.0f, 0.0f};
    }

    const double delta_s_ = (static_cast<double>(timestamp_ns_) - static_cast<double>(old_timestamp_ns_)) * 1.0e-9;

    acceleration_ = -accelerometer_sample_;

    
    if(accelerometer_sample_[0] < kThresholdSignal && ApproximateEqual(accelerometer_sample_[0], old_accelerometer_sample_[0], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[0],older_accelerometer_sample_[0], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[0],even_older_accelerometer_sample_[0], kThresholdAccelerationStable)){
        acceleration_[0] = 0.0;
    }
    if(accelerometer_sample_[1] < kThresholdSignal && ApproximateEqual(accelerometer_sample_[1], old_accelerometer_sample_[1], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[1],older_accelerometer_sample_[1], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[1],even_older_accelerometer_sample_[1], kThresholdAccelerationStable)){
        acceleration_[1] = 0.0;
    }
    if(accelerometer_sample_[2] < kThresholdSignal && ApproximateEqual(accelerometer_sample_[2], old_accelerometer_sample_[2], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[2],older_accelerometer_sample_[2], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[2],even_older_accelerometer_sample_[2], kThresholdAccelerationStable)){
        acceleration_[2] = 0.0;
    }

    velocity_ = old_velocity_ + (acceleration_ * 5) * delta_s_;
    
    if ((acceleration_[0] == 0) && ApproximateEqual(velocity_[0], old_velocity_[0], kThresholdVelocityBias) && ApproximateEqual(velocity_[0], older_velocity_[0], kThresholdVelocityBias) && ApproximateEqual(velocity_[0], even_older_velocity_[0], kThresholdVelocityBias)){
        velocity_[0] = velocity_[0] * kDecay;
    }
    if ((acceleration_[1] == 0) && ApproximateEqual(velocity_[1], old_velocity_[1], kThresholdVelocityBias) && ApproximateEqual(velocity_[1], older_velocity_[1], kThresholdVelocityBias) && ApproximateEqual(velocity_[1], even_older_velocity_[1], kThresholdVelocityBias)){
        velocity_[1] = velocity_[1] * kDecay;
    }
    if ((acceleration_[2] == 0) && ApproximateEqual(velocity_[2], old_velocity_[2], kThresholdVelocityBias) && ApproximateEqual(velocity_[2], older_velocity_[2], kThresholdVelocityBias) && ApproximateEqual(velocity_[2], even_older_velocity_[2], kThresholdVelocityBias)){
        velocity_[2] = velocity_[2] * kDecay;
    }

    // filter_velocity_.AddSample(velocity_, timestamp_ns_);
    // Vector3 velocity_filtered_ = filter_velocity_.GetFilteredData();

    position_ = old_position_ + 0.5 * (velocity_ + old_velocity_) * delta_s_;


    even_older_accelerometer_sample_ = older_accelerometer_sample_;
    older_accelerometer_sample_ = old_accelerometer_sample_;
    old_accelerometer_sample_ = accelerometer_sample_;

    even_older_velocity_ = older_velocity_;
    older_velocity_ = old_velocity_;
    old_velocity_ = velocity_;

    old_position_ = position_;
    old_timestamp_ns_ = timestamp_ns_;

    // log_count_++;
    // if (log_count_ > 10) {
    // log_count_ = 0;
    
    // // __android_log_print(ANDROID_LOG_INFO, "Samples", "%+4.5lf, %+4.5lf, %+4.5lf", accelerometer_sample_[0], accelerometer_sample_[1], accelerometer_sample_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+4.5lf, %+4.5lf, %+4.5lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+4.5lf, %+4.5lf, %+4.5lf", acceleration_unfiltered_[0], acceleration_unfiltered_[1], acceleration_unfiltered_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Gravity", "%+4.5lf, %+4.5lf, %+4.5lf", gravity_acceleration_[0], gravity_acceleration_[1], gravity_acceleration_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Acceleration_result", "%+4.5lf, %+4.5lf, %+4.5lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Gravity", "%+2.8lf", gravity);
    // // __android_log_print(ANDROID_LOG_INFO, "AccelerationF", "%+4.5lf, %+4.5lf, %+4.5lf", accelerometer_sample_filtered_[0], accelerometer_sample_filtered_[1], accelerometer_sample_filtered_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "AccelerationRotated", "%+4.5lf, %+4.5lf, %+4.5lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Velocity", "%+.5lf, %+.5lf, %+.5lf", velocity_[0], velocity_[1], velocity_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "VelocityF", "%+.5lf, %+.5lf, %+.5lf", velocity_filtered_[0], velocity_filtered_[1], velocity_filtered_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "Position", "%+.5lf, %+.5lf, %+.5lf", position_[0], position_[1], position_[2]);
    // // __android_log_print(ANDROID_LOG_INFO, "rotation", "%+.5f, %+.5f, %+.5f, %+.5f", orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
    // // __android_log_print(ANDROID_LOG_INFO, "rotation", "%+.5f, %+.5f, %+.5f, %+.5f", rotation_.GetQuaternion()[0], rotation_.GetQuaternion()[1], rotation_.GetQuaternion()[2], rotation_.GetQuaternion()[3]);
    // // __android_log_print(ANDROID_LOG_INFO, "Angles", "P:%+4.5lf, Y:%+4.5lf, R:%+4.5lf", rotation_.GetPitchAngle()*180.0/M_PI, rotation_.GetYawAngle()*180.0/M_PI, rotation_.GetRollAngle()*180.0/M_PI);
    // __android_log_print(ANDROID_LOG_INFO, "delta_s", "%+4.5lf", delta_s_);
    // }

    if(position_[0] < -5.0 ){
        position_[0] = -5.0;
        old_position_[0] = -5.0;
        old_velocity_ = {0.0, 0.0, 0.0};
    } 
    if(position_[0] > 5.0 ) {
        position_[0] = 5.0;
        old_position_[0] = 5.0;
        old_velocity_ = {0.0, 0.0, 0.0};
    }
    if(position_[1] > 1.0 ) {
        position_[1] = 1.0;
        old_position_[1] = 1.0;
        old_velocity_ = {0.0, 0.0, 0.0};
    }
    if(position_[1] < -4.0 ) {
        position_[1] = -4.0;
        old_position_[1] = -4.0;
        old_velocity_ = {0.0, 0.0, 0.0};
    }
    if(position_[2] < -5.0 ) {
        position_[2] = -5.0;
        old_position_[2] = -5.0;
        old_velocity_ = {0.0, 0.0, 0.0};
    } 
    if(position_[2] > 5.0 ) {
        position_[2] = 5;
        old_position_[2] = 5.0;
        old_velocity_ = {0.0, 0.0, 0.0};
    }

    return {static_cast<float>(position_[0]), static_cast<float>(position_[1]), static_cast<float>(position_[2])};
}

} // namespace cardboard