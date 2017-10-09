#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if(estimations.size()!= ground_truth.size() || estimations.size()==0){
    cout<<"Invalid estimation or ground_truth data"<<endl;
    return rmse;
  }
  for(unsigned int i=0;i<estimations.size();++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse+= residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  if(fabs(c1)< 0.0001){
    cout<<"Calculate Jacobian() - Error - Divisible by zero"<<endl;
    return Hj;
  }
  Hj << (px/c2),(py/c2),0,0,
        -(py/c1),(px/c1),0,0,
        py*(vx*py - vy*px)/c3,px*(px*vy - py*vx)/c3,px/c2,py/c2;
  return Hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd& x) {
  //  a self-explanatory method
  assert(x.size() == 4);
  VectorXd polar_output = VectorXd(3);

  polar_output << sqrt(pow(x[0], 2) + pow(x[1], 2)),
    atan2(x[1], x[0]),
    (x[0]*x[2] + x[1]*x[3])/sqrt(pow(x[0], 2) + pow(x[1],  2));
  return polar_output;
}


float Tools::WrapAnglePi(const float angle)  {
  // scale module 2pi to  wrap the angle between [-pi, pi]
  float angle_result = angle;
  if ( angle > 0 ) {
    angle_result = fmod(angle + M_PI, 2*M_PI) - M_PI;
  } else {
    angle_result = fmod(angle - M_PI, 2*M_PI) + M_PI;
  }

  return angle_result;
}
