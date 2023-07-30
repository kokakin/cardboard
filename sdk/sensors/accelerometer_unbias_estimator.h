//
// Created by pedro on 25/07/2023.
//

#ifndef CARDBOARD_ACCELEROMETER_UNBIAS_ESTIMATOR_H
#define CARDBOARD_ACCELEROMETER_UNBIAS_ESTIMATOR_H


#include "sensors/highpass_filter.h"


namespace cardboard {

    class AccelerometerUnbiasEstimator {
        public:
        AccelerometerUnbiasEstimator();
        virtual ~AccelerometerUnbiasEstimator();

        virtual void ProcessAccelerometer(const Vector3& accelerometer_sample,
                                          uint64_t timestamp_ns);

        void Reset();

        Vector3 GetAccelerometerWithoutGravity() const {
            return accelerometer_minus_gravity_;
        }

        private:

          HighpassFilter accelerometer_highpass_filter_;

          Vector3 accelerometer_minus_gravity_;

    };

} // cardboard

#endif //CARDBOARD_ACCELEROMETER_UNBIAS_ESTIMATOR_H
