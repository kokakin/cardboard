#ifndef CARDBOARD_SDK_SENSORS_POSITION_DATA_H_
#define CARDBOARD_SDK_SENSORS_POSITION_DATA_H_

#include "sensors/sensor_fusion_ekf.h"

#include <array>
#include <memory>

namespace cardboard {

class PositionData {
 public:
  PositionData();
  virtual ~PositionData();

  std::array<float, 3> GetPosition(Vector3 acceleration_updated_value, Vector4 orientation);


 private:
  std::array<float, 3> old_position_;
  std::array<float, 3> old_velocity_;
  std::array<float, 3> old_acceleration_;
  std::array<float, 3> position_;
  std::array<float, 3> velocity_;
  std::array<float, 3> acceleration_;

  SensorFusionEkf sensor_fusion_ekf_;

  int log_count_;

};

} // namespace cardboard

#endif  // CARDBOARD_SDK_SENSORS_POSITION_DATA_H_
