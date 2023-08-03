


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
    accelerometer_sample_filtered_({0.0, 0.0, 0.0}),
    old_position_({0.0, 0.0, 0.0}),
    old_velocity_({0.0, 0.0, 0.0}),
    old_acceleration_({0.0, 0.0, 0.0}),
    older_acceleration_({0.0, 0.0, 0.0}),
    old_orientation_({0.0, 0.0, 0.0, 0.0}),
    velocity_({0.0, 0.0, 0.0}),
    position_({0.0, 0.0, 0.0}),
    acceleration_({0.0, 0.0, 0.0}),
    mean_acceleration_({0.0, 0.0, 0.0}),
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
    filter_accelerometer_(
        {0.07044588935387128, 
         0.1408035820770175, 
         0.07044588935387128
        }, {
         1,
         -1.199678682863306, 
         0.5157460761550564 }),
    log_count_(0),
    old_timestamp_ns_(0) {
        filter_velocity_.Reset();
        filter_accelerometer_.Reset();
  __android_log_print(ANDROID_LOG_INFO, "Position", "Position constructor");
 }

PositionEstimator::~PositionEstimator() {}

// bool PositionEstimator::StableValueStream( double new_value_, double old_value_, double threshold) {
//     // Not comparable if old_value_ is too small
//     if (std::abs(old_value_) < 0.001){
//         return true;
//     }
//     double value_stability_ = (std::abs(new_value_) + std::abs(old_value_))/(2*std::abs(old_value_));
//     if(( value_stability_ > 1 + threshold || 
//          value_stability_ < 1 - threshold)) {
//         return false;
//     }
//     return true;
// }

bool PositionEstimator::ApproximateEqual( double new_value_, double old_value_, double threshold) {
    if( std::abs( new_value_ - old_value_ ) > threshold) {
        return false;
    }
    return true;
}

std::array<float, 3> PositionEstimator::GetPosition( Vector3 accelerometer_sample_, Vector4 orientation_, int64_t timestamp_ns_ ) {

    if(old_timestamp_ns_ == 0.0) {
        old_timestamp_ns_ = timestamp_ns_;
        old_orientation_ = orientation_;
        mean_acceleration_ = accelerometer_sample_;
        return {0.0f, 0.0f, 0.0f};
    }

//    if (!ApproximateEqual(orientation_[0], old_orientation_[0], 0.004) || 
//         !ApproximateEqual(orientation_[1], old_orientation_[1], 0.004) || 
//         !ApproximateEqual(orientation_[2], old_orientation_[2], 0.004) || 
//         !ApproximateEqual(orientation_[3], old_orientation_[3], 0.004))  {
//             mean_acceleration_ = accelerometer_sample_;
//             __android_log_print(ANDROID_LOG_INFO, "OrientationChanged", "!");
//         acceleration_ = {0.0, 0.0, 0.0};
//         old_acceleration_ = acceleration_;
//         older_acceleration_ = old_acceleration_;
//         old_accelerometer_sample_ = accelerometer_sample_;
//         older_accelerometer_sample_ = old_accelerometer_sample_;
//         even_older_accelerometer_sample_ = older_accelerometer_sample_;
//         old_orientation_ = orientation_;
//         old_timestamp_ns_ = timestamp_ns_;

//         return {static_cast<float>(old_position_[0]), static_cast<float>(old_position_[1]), static_cast<float>(old_position_[2])};
//     }

    double timestamp_s_ = (static_cast<double>(timestamp_ns_) - old_timestamp_ns_) * 1.0e-9;
    // if (timestamp_s_ < 0.001){
    //     __android_log_print(ANDROID_LOG_INFO,"TimestampTooLow", "%lf", timestamp_s_);
    //     old_timestamp_ns_ = timestamp_ns_;
    //     old_orientation_ = orientation_;
    //     return {static_cast<float>(old_position_[0]), static_cast<float>(old_position_[1]), static_cast<float>(old_position_[2])};
    // }

    if(ApproximateEqual(accelerometer_sample_[0], old_accelerometer_sample_[0],  kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[0],older_accelerometer_sample_[0], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[0],even_older_accelerometer_sample_[0], kThresholdAccelerationStable)){
        mean_acceleration_[0] = accelerometer_sample_[0];
    }
    if(ApproximateEqual(accelerometer_sample_[1], old_accelerometer_sample_[1],  kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[1],older_accelerometer_sample_[1], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[1],even_older_accelerometer_sample_[1], kThresholdAccelerationStable)){
        mean_acceleration_[1] = accelerometer_sample_[1];
    }
    if(ApproximateEqual(accelerometer_sample_[2], old_accelerometer_sample_[2],  kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[2],older_accelerometer_sample_[2], kThresholdAccelerationStable) && ApproximateEqual(accelerometer_sample_[2],even_older_accelerometer_sample_[2], kThresholdAccelerationStable)){
        mean_acceleration_[2] = accelerometer_sample_[2];
    }

    // filter_accelerometer_.AddSample(accelerometer_sample_ - mean_acceleration_, timestamp_ns_);
    // Vector3 accelerometer_sample_filtered_ = filter_accelerometer_.GetFilteredData();

    Vector3 accelerometer_sample_filtered_ = accelerometer_sample_ - mean_acceleration_;

    acceleration_ = 0.5 * (Rotation::FromQuaternion(orientation_) * Vector3(-accelerometer_sample_filtered_[2], accelerometer_sample_filtered_[1], accelerometer_sample_filtered_[0])) + 0.3*(old_acceleration_) + 0.2*(older_acceleration_);

    // if (!StableValueStream(acceleration_[0], old_acceleration_[0], 0.3)) {
    //     __android_log_print(ANDROID_LOG_INFO, "Jerk Too HighX", "%lf", (std::abs(acceleration_[0]) + std::abs(old_acceleration_[0]))/(std::abs(old_acceleration_[0]) + std::abs(old_acceleration_[0])));
    // }
    // if (!StableValueStream(acceleration_[1], old_acceleration_[1], 0.3)) {
    //     __android_log_print(ANDROID_LOG_INFO, "Jerk Too HighY", "%lf", (std::abs(acceleration_[1]) + std::abs(old_acceleration_[1]))/(std::abs(old_acceleration_[1]) + std::abs(old_acceleration_[1])));
    // }
    // if (!StableValueStream(acceleration_[2], old_acceleration_[2], 0.3)) {
    //     __android_log_print(ANDROID_LOG_INFO, "Jerk Too HighZ", "%lf", (std::abs(acceleration_[2]) + std::abs(old_acceleration_[2]))/(std::abs(old_acceleration_[2]) + std::abs(old_acceleration_[2])));
    // }

    velocity_ = old_velocity_ + (acceleration_) * timestamp_s_*13;

    filter_velocity_.AddSample(velocity_, timestamp_ns_);
    Vector3 velocity_filtered_ = filter_velocity_.GetFilteredData() * 0.5 + old_velocity_ * 0.5;
    
    position_ = old_position_ + 0.5* (velocity_filtered_ + old_velocity_) * timestamp_s_*13;

    old_accelerometer_sample_ = accelerometer_sample_;
    older_accelerometer_sample_ = old_accelerometer_sample_;
    even_older_accelerometer_sample_ = older_accelerometer_sample_;

    old_acceleration_ = acceleration_;
    older_acceleration_ = old_acceleration_;

    old_velocity_ = velocity_filtered_;
    old_position_ = position_;

    old_orientation_ = orientation_;
    old_timestamp_ns_ = timestamp_ns_;

    log_count_++;
    if (log_count_ > 0) {
    log_count_ = 0;
    __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+4.5lf, %+4.5lf, %+4.5lf", accelerometer_sample_[0], accelerometer_sample_[1], accelerometer_sample_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "AccelerationF", "%+4.5lf, %+4.5lf, %+4.5lf", accelerometer_sample_filtered_[0], accelerometer_sample_filtered_[1], accelerometer_sample_filtered_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "AccelerationMean", "%+4.5lf, %+4.5lf, %+4.5lf", mean_acceleration_[0], mean_acceleration_[1], mean_acceleration_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "AccelerationRotated", "%+4.5lf, %+4.5lf, %+4.5lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Velocity", "%+.5lf, %+.5lf, %+.5lf", velocity_[0], velocity_[1], velocity_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "VelocityF", "%+.5lf, %+.5lf, %+.5lf", velocity_filtered_[0], velocity_filtered_[1], velocity_filtered_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Position", "%+.5lf, %+.5lf, %+.5lf", position_[0], position_[1], position_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Orientation", "%+.5f, %+.5f, %+.5f, %+.5f", orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
    // __android_log_print(ANDROID_LOG_INFO, "Rotation", "%+.5lf, %+.5lf, %+.5lf, %+.5lf", rotation.GetQuaternion()[0], rotation.GetQuaternion()[1], rotation.GetQuaternion()[2], rotation.GetQuaternion()[3]);
    // __android_log_print(ANDROID_LOG_INFO, "Timestamp", "%+.5lf", timestamp_s_);
    }

    if(position_[0] < -5 || position_[0] > 5) {
        position_[0] = 0.0;
        old_acceleration_[0] = 0.0;
        older_acceleration_[0] = 0.0;
        old_accelerometer_sample_[0] = 0.0;
        older_accelerometer_sample_[0] = 0.0;
        even_older_accelerometer_sample_[0] = 0.0;
        old_velocity_[0] = 0.0;
        old_position_[0] = 0.0;
    }
    if(position_[1] > 0) {
        position_[1] = 0.0;
        old_acceleration_[1] = 0.0;
        older_acceleration_[1] = 0.0;
        old_accelerometer_sample_[1] = 0.0;
        older_accelerometer_sample_[1] = 0.0;
        even_older_accelerometer_sample_[1] = 0.0;
        old_velocity_[1] = 0.0;
        old_position_[1] = 0.0;
    }
    if(position_[1] < -3) {
        position_[1] = -3.0;
        old_position_[1] = -3.0;
        // __android_log_print(ANDROID_LOG_INFO, "Ceiling", "%+.5lf", position_[1]);
    }
    
    if(position_[2] < -5 || position_[2] > 5) {
        position_[2] = 0.0;
        old_acceleration_[2] = 0.0;
        older_acceleration_[2] = 0.0;
        old_accelerometer_sample_[2] = 0.0;
        older_accelerometer_sample_[2] = 0.0;
        even_older_accelerometer_sample_[2] = 0.0;
        old_velocity_[2] = 0.0;
        old_position_[2] = 0.0;
    }

    return {static_cast<float>(position_[0]), static_cast<float>(position_[1]), static_cast<float>(position_[2])};
}

} // namespace cardboard