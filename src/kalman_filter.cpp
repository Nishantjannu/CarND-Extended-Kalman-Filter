#include "kalman_filter.h"
//#include <math.h>
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

// defines the predict function, the update function for lidar, and the update function for radar

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/*
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
*/

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_; // mean acceleration = 0
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  Eigen::VectorXd z_pred = H_laser_ * x_;
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd Ht = H_laser_.transpose();
  Eigen::MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  
  Eigen::VectorXd z_pred = VectorXd(3);
  
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
    
  float c = sqrt(px*px + py*py);
  
  // check division by zero
  if (fabs(c) < 0.0001) {
    cout << "UpdateEKF () - Error - h(x) - rho_dot - Division by Zero" << endl;
  }
  
  z_pred << c, atan2(py,px), (px*vx + py*vy)/c ;
  
  Eigen::VectorXd y = z - z_pred;
  
  // normalize phi in the y vector so that its angle is between -pi and pi
  if (fabs(y[1]) > M_PI){
    if (y[1] > 0){
      y[1] -= 2*M_PI;
    }  
    else {
      y[1] += 2*M_PI;
    }   
  }
      
  Eigen::MatrixXd Ht = Hj_.transpose();
  Eigen::MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
  
}
