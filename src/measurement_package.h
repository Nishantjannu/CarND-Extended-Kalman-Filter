#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  // SensorType is the flag (enum name) and Laser/Radar are its two possible values
  // enum SensorType;
  // SensorType sensor_type_;
  // doing both in 1 go
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;
  // vector is created
  Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
