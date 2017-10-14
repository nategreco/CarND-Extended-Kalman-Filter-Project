#include <math.h>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  I_ = Eigen::MatrixXd::Identity(4,4);
}

void KalmanFilter::Predict() {
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //Get matrices
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
    
  //Calculate new State
  x_ = x_ + (K*y);
  P_ = (I_-K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //Create Hf
  VectorXd Hf(3);
  float rho = sqrt( x_[0]*x_[0] + x_[1]*x_[1] );
  Hf << rho, atan2(x_[1], x_[0]), (x_[0]*x_[2] + x_[1]*x_[3]) / rho;

  //Get matrices
  VectorXd y = z - Hf;
  if( y[1] > M_PI )	y[1] -= 2 * M_PI;
  if( y[1] < -M_PI ) y[1] += 2 * M_PI;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
    
  //Calculate new State
  x_ = x_ + (K*y);
  P_ = (I_-K*H_) * P_;
}
