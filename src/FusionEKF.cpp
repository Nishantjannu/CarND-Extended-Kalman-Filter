#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

// initializes the filter, calls the predict function
// calls the update function

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.R_laser_ = MatrixXd(2, 2); 
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.Hj_ = MatrixXd(3, 4); 
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  
  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF. (DONE)
   * TODO: Set the process and measurement noises (DONE)
   */
  
  //initial state covariance matrix
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  //measurement matrix - laser
  ekf_.H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // initial state transition matrix
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  
  // set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement. (DONE)
     * TODO: Create the covariance matrix. (ALREADY created above in constructor????)
     * You'll need to convert radar from polar to cartesian coordinates. (DONE)
     */
    
    // first measurement // (initial) state
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4); // sizing
    ekf_.x_ << 1, 1, 1, 1; // populating // avoids any zero divison error at start
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      ekf_.x_[0] = (measurement_pack.raw_measurements_[0])*cos(measurement_pack.raw_measurements_[1]);
      ekf_.x_[1] = (measurement_pack.raw_measurements_[0])*sin(measurement_pack.raw_measurements_[1]);
      //continue
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 1, 1;
      // px, py initialised with measurement 
      // vx, vy unchanged as no new information (vx,vy = 1,1)

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_; // ADDED BY ME
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds. (DONE)
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix. (DONE)
   */
  
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  
  
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    
    ekf_.Hj_= tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
