


#include "sensors/position_estimator.h"
#include "sensors/sensor_fusion_ekf.h"
#include "util/rotation.h"
#include <array>
#include <android/log.h>


namespace cardboard {

PositionEstimator::PositionEstimator()
:
    accelerometer_sample_filtered_({0.0, 0.0, 0.0}),
    old_position_({0.0, 0.0, 0.0}),
    old_velocity_({0.0, 0.0, 0.0}),
    old_acceleration_({0.0, 0.0, 0.0}),
    older_acceleration_({0.0, 0.0, 0.0}),
    old_orientation_({0.0, 0.0, 0.0, 0.0}),
    velocity_({0.0, 0.0, 0.0}),
    position_({0.0, 0.0, 0.0}),
    acceleration_({0.0, 0.0, 0.0}),
    filter_velocity_(
        {0.04270192883236114, 
         0.08530595930880489, 
         0.04270192883236113
        }, {
         1,
         -1.411292433720418, 
         0.6028319986952257}),
    filter_accelerometer_(
        {0.198901419871832, 
         0.0, 
         -0.198901419871832
        }, {
         1,
         -1.519457875345425, 
         0.6021971602563359}),
    log_count_(0),
    old_timestamp_ns_(0) {
        filter_velocity_.Reset();
        filter_accelerometer_.Reset();
  __android_log_print(ANDROID_LOG_INFO, "Position", "Position constructor");
 }

PositionEstimator::~PositionEstimator() {}

bool PositionEstimator::StableValueStream( double new_value_, double old_value_, double threshold) {
    // Not comparable if old_value_ is too small
    if (std::abs(old_value_) < 0.001){
        return true;
    }
    double value_stability_ = (std::abs(new_value_) + std::abs(old_value_))/(2*std::abs(old_value_));
    if(( value_stability_ > 1 + threshold || 
         value_stability_ < 1 - threshold)) {
        return false;
    }
    return true;
}

std::array<float, 3> PositionEstimator::GetPosition( Vector3 accelerometer_sample_, Vector4 orientation_, int64_t timestamp_ns_ ) {

    if(old_timestamp_ns_ == 0.0) {
        old_timestamp_ns_ = timestamp_ns_;
        old_orientation_ = orientation_;
        return {0.0f, 0.0f, 0.0f};
    }

//    if ((std::abs(orientation_[0]) > 0.02 && 
//         !StableValueStream(orientation_[0], old_orientation_[0], 0.011)) || 
//         (std::abs(orientation_[1]) > 0.02 && 
//         !StableValueStream(orientation_[1], old_orientation_[1], 0.011)) || 
//         (std::abs(orientation_[2]) > 0.02 && 
//         !StableValueStream(orientation_[2], old_orientation_[2], 0.011)) || 
//         (std::abs(orientation_[3]) > 0.02 && 
//         !StableValueStream(orientation_[3], old_orientation_[3], 0.011)))  {
//         // __android_log_print(ANDROID_LOG_INFO, "OrientationChanged", "%+.3lf, %+.3lf, %+.3lf, %+.3lf", (std::abs(orientation_[0]) + std::abs(old_orientation_[0]))/(2*std::abs(old_orientation_[0])), (std::abs(orientation_[1]) + std::abs(old_orientation_[1]))/(2*std::abs(old_orientation_[1])), (std::abs(orientation_[2]) + std::abs(old_orientation_[2]))/(2*std::abs(old_orientation_[2])), (std::abs(orientation_[3]) + std::abs(old_orientation_[3]))/(2*std::abs(old_orientation_[3])));
//         old_timestamp_ns_ = timestamp_ns_;
//         old_orientation_ = orientation_;
//         return {static_cast<float>(old_position_[0]), static_cast<float>(old_position_[1]), static_cast<float>(old_position_[2])};
//     }

    double timestamp_s_ = (static_cast<double>(timestamp_ns_) - old_timestamp_ns_) * 1.0e-9;
    if (timestamp_s_ < 0.001){
        __android_log_print(ANDROID_LOG_INFO,"TimestampTooLow", "%lf", timestamp_s_);
        old_timestamp_ns_ = timestamp_ns_;
        old_orientation_ = orientation_;
        return {static_cast<float>(old_position_[0]), static_cast<float>(old_position_[1]), static_cast<float>(old_position_[2])};
    }

    filter_accelerometer_.AddSample(accelerometer_sample_, timestamp_ns_);
    Vector3 accelerometer_sample_filtered_ = filter_accelerometer_.GetFilteredData();

    acceleration_ = 0.5 * (Rotation::FromQuaternion(orientation_) * Vector3(-accelerometer_sample_filtered_[2], accelerometer_sample_filtered_[1], accelerometer_sample_filtered_[0])) + 0.25*(old_acceleration_) + 0.25*(older_acceleration_);
    
    // acceleration_[0] = acceleration_[0]/std::abs(acceleration_[0]) * log10(std::abs(acceleration_[0]*10)) * 100;
    // acceleration_[1] = acceleration_[1]/std::abs(acceleration_[1]) * log10(std::abs(acceleration_[1]*10)) * 100;
    // acceleration_[2] = acceleration_[2]/std::abs(acceleration_[2]) * log10(std::abs(acceleration_[2]*10)) * 100;

    // if (!StableValueStream(acceleration_[0], old_acceleration_[0], 0.3)) {
    //     __android_log_print(ANDROID_LOG_INFO, "Jerk Too HighX", "%lf", (std::abs(acceleration_[0]) + std::abs(old_acceleration_[0]))/(std::abs(old_acceleration_[0]) + std::abs(old_acceleration_[0])));
    // }
    // if (!StableValueStream(acceleration_[1], old_acceleration_[1], 0.3)) {
    //     __android_log_print(ANDROID_LOG_INFO, "Jerk Too HighY", "%lf", (std::abs(acceleration_[1]) + std::abs(old_acceleration_[1]))/(std::abs(old_acceleration_[1]) + std::abs(old_acceleration_[1])));
    // }
    // if (!StableValueStream(acceleration_[2], old_acceleration_[2], 0.3)) {
    //     __android_log_print(ANDROID_LOG_INFO, "Jerk Too HighZ", "%lf", (std::abs(acceleration_[2]) + std::abs(old_acceleration_[2]))/(std::abs(old_acceleration_[2]) + std::abs(old_acceleration_[2])));
    // }


    // if(acceleration_[0] < 0.009 && acceleration_[0] > -0.009) {
    //     acceleration_[0] = 0.0;
    // }
    // if(acceleration_[1] < 0.009 && acceleration_[1] > -0.009) {
    //     acceleration_[1] = 0.0;
    // }
    // if(acceleration_[2] < 0.009 && acceleration_[2] > -0.009) {
    //     acceleration_[2] = 0.0;
    // }

    // __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+.4lf, %+.4lf, %+.4lf", acceleration_[0], acceleration_[1], acceleration_[2]);

    velocity_ = old_velocity_ + (acceleration_) * timestamp_s_*20;

    filter_velocity_.AddSample(velocity_, timestamp_ns_);
    Vector3 velocity_filtered_ = filter_velocity_.GetFilteredData()*0.5 + old_velocity_*0.5;
    
    position_ = old_position_ + (old_velocity_) * timestamp_s_*20 + (acceleration_) * timestamp_s_*20 * timestamp_s_*20 * 0.5;

    old_acceleration_ = acceleration_;
    older_acceleration_ = old_acceleration_;
    old_velocity_ = velocity_filtered_;
    old_position_ = position_;
    old_orientation_ = orientation_;
    old_timestamp_ns_ = timestamp_ns_;

    log_count_++;
    if (log_count_ > 30) {
    log_count_ = 0;
    // __android_log_print(ANDROID_LOG_INFO, "Acceleration", "%+4.5lf, %+4.5lf, %+4.5lf", accelerometer_sample_[0], accelerometer_sample_[1], accelerometer_sample_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "AccelerationF", "%+4.5lf, %+4.5lf, %+4.5lf", accelerometer_sample_filtered_[0], accelerometer_sample_filtered_[1], accelerometer_sample_filtered_[2]);
    __android_log_print(ANDROID_LOG_INFO, "AccelerationRotated", "%+4.5lf, %+4.5lf, %+4.5lf", acceleration_[0], acceleration_[1], acceleration_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Velocity", "%+.5lf, %+.5lf, %+.5lf", velocity_[0], velocity_[1], velocity_[2]);
    __android_log_print(ANDROID_LOG_INFO, "VelocityF", "%+.5lf, %+.5lf, %+.5lf", velocity_filtered_[0], velocity_filtered_[1], velocity_filtered_[2]);
    __android_log_print(ANDROID_LOG_INFO, "Position", "%+.5lf, %+.5lf, %+.5lf", position_[0], position_[1], position_[2]);
    // __android_log_print(ANDROID_LOG_INFO, "Orientation", "%+.5f, %+.5f, %+.5f, %+.5f", orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
    // __android_log_print(ANDROID_LOG_INFO, "Rotation", "%+.5lf, %+.5lf, %+.5lf, %+.5lf", rotation.GetQuaternion()[0], rotation.GetQuaternion()[1], rotation.GetQuaternion()[2], rotation.GetQuaternion()[3]);
    // __android_log_print(ANDROID_LOG_INFO, "Timestamp", "%+.5lf", timestamp_s_);
    }

    if(position_[0] < -5 || position_[0] > 5) {
        position_[0] = 0.0;
        old_acceleration_[0] = 0.0;
        older_acceleration_[0] = 0.0;
        old_velocity_[0] = 0.0;
        old_position_[0] = 0.0;
    }
    if(position_[1] < -5 || position_[1] > 0) {
        position_[1] = 0.0;
        old_acceleration_[1] = 0.0;
        older_acceleration_[1] = 0.0;
        old_velocity_[1] = 0.0;
        old_position_[1] = 0.0;
    }
    if(position_[2] < -5 || position_[2] > 5) {
        position_[2] = 0.0;
        old_acceleration_[2] = 0.0;
        older_acceleration_[2] = 0.0;
        old_velocity_[2] = 0.0;
        old_position_[2] = 0.0;
    }

    return {static_cast<float>(position_[0]), static_cast<float>(position_[1]), static_cast<float>(position_[2])};
}

} // namespace cardboard