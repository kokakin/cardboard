/*
 * Copyright 2019 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sensors/highpass_filter.h"

#include <cmath>

namespace {

const double kSecondsFromNanoseconds = 1e-9;

// Minimum time step between sensor updates. This corresponds to 1000 Hz.
const double kMinTimestepS = 0.001f;

// Maximum time step between sensor updates. This corresponds to 1 Hz.
const double kMaxTimestepS = 1.00f;

}  // namespace

namespace cardboard {

HighpassFilter::HighpassFilter(double cutoff_freq_hz, bool velocity_filter_)
    : cutoff_time_constant_(1.0 / (2.0 * M_PI * cutoff_freq_hz)),
      initialized_(false) {
        
        double ripple = 2;
        double attenuation = 120.0;
        double cutoff_freq = 14.28;
        double sample_rate = 47.6;

        double omega_c = 2.0 * M_PI * cutoff_freq / sample_rate;
        double epsilon = sqrt(pow(10.0, 0.1 * ripple) - 1.0);
        double delta = sqrt(pow(10.0, 0.1 * attenuation) - 1.0);
        double K = delta / epsilon;
        double v0 = 1.0 / (1.0 + epsilon);

        if(velocity_filter_){
          b_0_ = 0.1362835867548736;
          b_1_ = 0.2725191945234873;
          b_2_ = 0.1362835867548736;
          a_1_ = -0.7669774525135551;
          a_2_ = 0.4532005328531817; 
        }
        else {
          b_0_ = 0.09330921180937071;
          b_1_ = 0;
          b_2_ = -0.09330921180937071;
          a_1_ = -1.550943005005771;
          a_2_ = 0.8133815763812584;
        }

  Reset();
}

void HighpassFilter::AddSample(const Vector3& sample, uint64_t timestamp_ns) {
  AddWeightedSample(sample, timestamp_ns, 1.0);
}

void HighpassFilter::AddWeightedSample(const Vector3& sample,
                                      uint64_t timestamp_ns, double weight) {
  if (!initialized_) {
    // Initialize filter state
    filtered_data_ = {sample[0], sample[1], sample[2]};
    timestamp_most_recent_update_ns_ = timestamp_ns;
    initialized_ = true;
    return;
  }

  if (timestamp_ns < timestamp_most_recent_update_ns_) {
    timestamp_most_recent_update_ns_ = timestamp_ns;
    return;
  }

  const double delta_s =
      static_cast<double>(timestamp_ns - timestamp_most_recent_update_ns_) *
      kSecondsFromNanoseconds;
  if (delta_s <= kMinTimestepS || delta_s > kMaxTimestepS) {
    timestamp_most_recent_update_ns_ = timestamp_ns;
    return;
  }

  filtered_data_ = b_0_ * sample + b_1_ * x_1_ + b_2_ * x_2_ - a_1_ * y_1_ - a_2_ * y_2_;
  x_2_ = x_1_;
  x_1_ = sample;
  y_2_ = y_1_;
  y_1_ = filtered_data_;
  
  timestamp_most_recent_update_ns_ = timestamp_ns;
}

void HighpassFilter::Reset() {
  initialized_ = false;
  filtered_data_ = x_1_ = x_2_ = y_1_ = y_2_ = {0, 0, 0};
}

}  // namespace cardboard
