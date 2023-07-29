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
#include "sensors/device_pose_6dof_sensor.h"

#include <android/looper.h>
#include <android/sensor.h>
#include <stddef.h>

#include <memory>
#include <mutex>  // NOLINT

#include "util/logging.h"

// Workaround to avoid the inclusion of "android_native_app_glue.h.
#ifndef LOOPER_ID_USER
#define LOOPER_ID_USER 3
#endif

namespace cardboard {

namespace {

// Creates an Android sensor event queue for the current thread.
static ASensorEventQueue* CreateSensorQueue(ASensorManager* sensor_manager) {
  ALooper* event_looper = ALooper_forThread();

  if (event_looper == nullptr) {
    event_looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    CARDBOARD_LOGI(
        "Pose6DOFSensor: Created new event looper for Pose6DOF "
        "sensor capture thread.");
  }

  return ASensorManager_createEventQueue(sensor_manager, event_looper,
                                         LOOPER_ID_USER, nullptr, nullptr);
}

const ASensor* InitSensor(ASensorManager* sensor_manager) {
  return ASensorManager_getDefaultSensor(sensor_manager,
                                         ASENSOR_TYPE_POSE_6DOF);
}

bool PollLooper(int timeout_ms, int* num_events) {
  void* source = nullptr;
  const int looper_id = ALooper_pollAll(timeout_ms, NULL, num_events,
                                        reinterpret_cast<void**>(&source));
  if (looper_id != LOOPER_ID_USER) {
    return false;
  }
  if (*num_events <= 0) {
    return false;
  }
  return true;
}

class SensorEventQueueReader {
 public:
  SensorEventQueueReader(ASensorManager* manager, const ASensor* sensor)
      : manager_(manager),
        sensor_(sensor),
        queue_(CreateSensorQueue(manager_)) {}

  ~SensorEventQueueReader() {
    ASensorManager_destroyEventQueue(manager_, queue_);
  }

  bool Start() {
    ASensorEventQueue_enableSensor(queue_, sensor_);
    const int min_delay = ASensor_getMinDelay(sensor_);
    // Set sensor capture rate to the highest possible sampling rate.
    ASensorEventQueue_setEventRate(queue_, sensor_, min_delay);
    return true;
  }

  void Stop() { ASensorEventQueue_disableSensor(queue_, sensor_); }

  bool WaitForEvent(int timeout_ms, ASensorEvent* event) {
    int num_events;
    if (!PollLooper(timeout_ms, &num_events)) {
      return false;
    }
    return (ASensorEventQueue_getEvents(queue_, event, 1) > 0);
  }

  bool ReadEvent(ASensorEvent* event) {
    return (ASensorEventQueue_getEvents(queue_, event, 1) > 0);
  }

 private:
  ASensorManager* manager_;   // Owned by android library.
  const ASensor* sensor_;     // Owned by android library.
  ASensorEventQueue* queue_;  // Owned by this.
};

void ParseAccelerometerEvent(const ASensorEvent& event,
                             Pose6DOFData* sample) {
  sample->sensor_timestamp_ns = event.timestamp;
  sample->system_timestamp = event.timestamp;
  // The event values in ASensorEvent (event, acceleration and
  // magnetic) are all in the same union type so they can be
  // accessed by event.
  sample->data = {event.data[0], event.data[1], event.data[2],event.data[3],
                  event.data[4], event.data[5], event.data[6],
                  event.data[7], event.data[8], event.data[9], event.data[10],
                  event.data[11], event.data[12], event.data[13],
                  event.data[14]};
}

}  // namespace

// This struct holds android specific sensor information.
struct DevicePose6DOFSensor::SensorInfo {
  SensorInfo() : sensor_manager(nullptr), sensor(nullptr) {}

  ASensorManager* sensor_manager;
  const ASensor* sensor;
  std::unique_ptr<SensorEventQueueReader> reader;
};

DevicePose6DOFSensor::DevicePose6DOFSensor()
    : sensor_info_(new SensorInfo()) {
  sensor_info_->sensor_manager = ASensorManager_getInstance();
  sensor_info_->sensor = InitSensor(sensor_info_->sensor_manager);
  if (!sensor_info_->sensor) {
    CARDBOARD_LOGE(
        "Pose6DOFSensor: Pose6DOF sensor is not available on this device.");
    return;
  }

  sensor_info_->reader =
      std::unique_ptr<SensorEventQueueReader>(new SensorEventQueueReader(
          sensor_info_->sensor_manager, sensor_info_->sensor));
}

DevicePose6DOFSensor::~DevicePose6DOFSensor() {}

void DevicePose6DOFSensor::PollForSensorData(
    int timeout_ms, std::vector<Pose6DOFData>* results) const {
  results->clear();
  ASensorEvent event;
  if (!sensor_info_->reader->WaitForEvent(timeout_ms, &event)) {
    return;
  }
  do {
    Pose6DOFData sample;
    ParseAccelerometerEvent(event, &sample);
    results->push_back(sample);
  } while (sensor_info_->reader->ReadEvent(&event));
}

bool DevicePose6DOFSensor::Start() {
  if (!sensor_info_->reader) {
    CARDBOARD_LOGE("Could not start Pose 6DOF sensor");
    return false;
  }
  return sensor_info_->reader->Start();
}

void DevicePose6DOFSensor::Stop() {
  if (!sensor_info_->reader) {
    return;
  }
  sensor_info_->reader->Stop();
}

}  // namespace cardboard
