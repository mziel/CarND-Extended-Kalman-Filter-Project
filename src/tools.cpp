#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
          || estimations.size() == 0){
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
  }
  
  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
  
      VectorXd residual = estimations[i] - ground_truth[i];
  
      //coefficient-wise multiplication
      residual = residual.array()*residual.array();
      rmse += residual;
  }
  
  //calculate the mean
  rmse = rmse/estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return the result
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  
  //check division by zero
  if(fabs(c1) < 0.0001){
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      return Hj;
  }
  
  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  return Hj;
}


VectorXd Tools::CarthesianToPolar(const VectorXd& x_input) {

  VectorXd x_out = VectorXd(3);

  float p_x = x_input[0];
  float p_y = x_input[1];
  float v_x = x_input[2];
  float v_y = x_input[3];

  float len_r = sqrt(p_x*p_x + p_y*p_y);
  float rate;
  if (len_r < 0.00001)
  {
    rate = 0.0;
  }
  else
  {
    rate = (p_x*p_y + p_y*v_y) / len_r;
  }

  x_out << len_r, atan2(p_y, p_x), rate;

  return x_out;
}


VectorXd Tools::PolarToCarthesian(const VectorXd& x_input) {

  VectorXd x_out = VectorXd(4);

  float rho = x_input[0];
  float phi = x_input[1];
  float rho_rate = x_input[2];

  float p_x = rho * cos(phi);
  float p_y = rho * sin(phi);
  float v_x = rho_rate * cos(phi);
  float v_y = rho_rate * sin(phi);

  x_out << p_x, p_y, v_x, v_y;

  return x_out;
}
