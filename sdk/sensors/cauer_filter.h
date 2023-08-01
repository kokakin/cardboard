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
#ifndef CARDBOARD_SDK_SENSORS_CAUER_FILTER_H_
#define CARDBOARD_SDK_SENSORS_CAUER_FILTER_H_

#include <array>
#include <memory>

#include "util/vector.h"

namespace cardboard {

// Implements an IIR, second order
class CauerFilter {
 public:
  // Initializes a filter with the given poles and zeros.
  explicit CauerFilter(std::array<double, 3> b_, std::array<double, 3> a_);

  // Updates the filter with the given sample. Note that samples with
  // non-monotonic timestamps and successive samples with a time steps below 1
  // ms or above 1 s are ignored.
  //
  // @param sample current sample data.
  // @param timestamp_ns timestamp associated to this sample in nanoseconds.
  void AddSample(const Vector3& sample, uint64_t timestamp_ns);

  // Updates the filter with the given weighted sample.
  //
  // @param sample current sample data.
  // @param timestamp_ns timestamp associated to this sample in nanoseconds.
  // @param weight typically a [0, 1] weight factor used when applying a new
  //     sample. A weight of 1 corresponds to calling AddSample. A weight of 0
  //     makes the update no-op. The first initial sample is not affected by
  //     this.
  void AddWeightedSample(const Vector3& sample, uint64_t timestamp_ns,
                         double weight);

  // Returns the filtered value. A vector with zeros is returned if no samples
  // have been added.
  Vector3 GetFilteredData() const { return filtered_data_; }

  // Returns the most recent update timestamp in ns.
  uint64_t GetMostRecentTimestampNs() const {
    return timestamp_most_recent_update_ns_;
  }

  // Returns true when the filter is initialized.
  bool IsInitialized() const { return initialized_; }

  // Resets filter state.
  void Reset();

 private:

  // Filter coefficients.
  void FindAndSetCoefficients();

  // const double cutoff_time_constant_;
  uint64_t timestamp_most_recent_update_ns_;
  bool initialized_;

  double sampling_frequency_;
  double cutoff_frequency_;
  double a_1_, a_2_, b_0_, b_1_, b_2_;
  Vector3 x_1_, x_2_, y_1_, y_2_;

  Vector3 filtered_data_;
};

}  // namespace cardboard

#endif  // CARDBOARD_SDK_SENSORS_CAUER_FILTER_H_
