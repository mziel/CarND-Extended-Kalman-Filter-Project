#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //measurement covariance matrix
  MatrixXd R_in = MatrixXd(1, 1);
  R_in << 1;

  // measurement mapping matrix - laser
  H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  // state vector
  VectorXd x_in = VectorXd(4);
  x_in << 0, 0, 0, 0;

  // state covariance matrix
  MatrixXd P_in = MatrixXd(4, 4);
  P_in << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  // state transistion matrix (dt == 0 initially)
  MatrixXd F_in = MatrixXd(4, 4);
  F_in << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

  // process covariance matrix
  MatrixXd Q_in = MatrixXd(4, 4);
  Q_in << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  //Initializes Kalman filter
  ekf_.Init(x_in, P_in, F_in, H_laser_, R_in, Q_in);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_ = tools.PolarToCarthesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state (0.0 since not moving)
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
    }

    // Create the covariance matrix
    float p_variance = 100;
    float v_variance = 10;
    ekf_.P_ << p_variance, 0, 0, 0,
             0, p_variance, 0, 0,
             0, 0, v_variance, 0,
             0, 0, 0, v_variance;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Update the state transition matrix F according to the new elapsed time
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix
  int noise_ax = 9;
  int noise_ay = 9;

  // Update the process noise covariance matrix
  MatrixXd Q_v = MatrixXd(2, 2);
  Q_v << noise_ax, 0,
        0, noise_ay;

  float dt2 = dt * dt;
  MatrixXd G = MatrixXd(4, 2);
  G << dt2/2.0, 0,
       0, dt2/2.0,
       dt,       0,
       0,       dt;
  ekf_.Q_ = G*Q_v*G.transpose();

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  VectorXd measurements = measurement_pack.raw_measurements_;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Validate the measurements
    if (measurements[0] != 0.0 && measurements[1] != 0.0 && measurements[2] != 0.0) {
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurements); // Radar updates
    }
  } else {
    // Validate the measurements
    if (measurement_pack.raw_measurements_[0] != 0.0 && measurement_pack.raw_measurements_[1] != 0.0) {
      ekf_.R_ = R_laser_;
      ekf_.Update(measurements); // Laser updates
    }
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
