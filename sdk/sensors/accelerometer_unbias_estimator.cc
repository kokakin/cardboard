//
// Created by pedro on 25/07/2023.
//

#include "accelerometer_unbias_estimator.h"

namespace cardboard {


AccelerometerUnbiasEstimator::AccelerometerUnbiasEstimator()
    : accelerometer_highpass_filter_(2.0f, false),
    accelerometer_lowpass_filter_(2.0f) {
  Reset();
}

AccelerometerUnbiasEstimator::~AccelerometerUnbiasEstimator() {}

void AccelerometerUnbiasEstimator::Reset() {
  accelerometer_highpass_filter_.Reset();
}

void AccelerometerUnbiasEstimator::ProcessAccelerometer(
    const Vector3& accelerometer_sample, uint64_t timestamp_ns) {


  accelerometer_highpass_filter_.AddSample(accelerometer_sample, timestamp_ns);

  accelerometer_minus_gravity_lowpass_ =
    accelerometer_highpass_filter_.GetFilteredData();
  
  //   accelerometer_lowpass_filter_.AddSample(accelerometer_minus_gravity_,
  //                                         timestamp_ns);
  
  // accelerometer_minus_gravity_lowpass_ =
  //   accelerometer_lowpass_filter_.GetFilteredData();
  
}


} // cardboard