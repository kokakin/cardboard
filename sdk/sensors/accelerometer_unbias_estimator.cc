//
// Created by pedro on 25/07/2023.
//

#include "accelerometer_unbias_estimator.h"

namespace cardboard {


AccelerometerUnbiasEstimator::AccelerometerUnbiasEstimator()
    : accelerometer_highpass_filter_(12.0f),
    accelerometer_lowpass_filter_(6.0f) {
  Reset();
}

AccelerometerUnbiasEstimator::~AccelerometerUnbiasEstimator() {}

void AccelerometerUnbiasEstimator::Reset() {
  accelerometer_highpass_filter_.Reset();
}

void AccelerometerUnbiasEstimator::ProcessAccelerometer(
    const Vector3& accelerometer_sample, uint64_t timestamp_ns) {

  
    accelerometer_lowpass_filter_.AddSample(accelerometer_sample,
                                          timestamp_ns);
  
  accelerometer_minus_gravity_ =
    accelerometer_lowpass_filter_.GetFilteredData();
  
  
  // Update accel and accel delta low-pass filters.
  accelerometer_highpass_filter_.AddSample(accelerometer_minus_gravity_, timestamp_ns);

  accelerometer_minus_gravity_lowpass_ =
    accelerometer_highpass_filter_.GetFilteredData();


  
}


} // cardboard