#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ *x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  // new estimates
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Tools tools;

  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd y = z - tools.CartesianToPolar(x_);
  y[1] = tools.WrapAnglePi(y[1]);

  VectorXd Hx = VectorXd(3);
  float ro = z(0);
  float phi = z(1);
  float px = ro*cos(phi);
  float py = ro*sin(phi);
  float v = z(2);
  float vx = v* cos(phi);
  float vy = v* sin(phi);
  VectorXd x_state = VectorXd(4);
  x_state << px,py,vx,vy;
  MatrixXd Hj = tools.CalculateJacobian(x_state);

}
