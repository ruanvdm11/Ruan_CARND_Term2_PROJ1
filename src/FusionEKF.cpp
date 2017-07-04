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
  R_laser_ << 0.0225, 0,
        0, 0.0225;
  R_radar_ << 0.09, 0, 0,
    		0, 0.0009, 0,
    		0, 0, 0.09;

  ekf_.H_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  /**
  TODO: Done
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // initiate x state vector
  ekf_.x_ = VectorXd(4);
  // initiate covariance matrix   ### VALUE UNSURE ###
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  // Setup measurement matrix LASER
  ekf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;
  // Setup measurement matrx RADAR
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // Set the acceleration noise components ### DONE ###

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float roh = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rohdot = measurement_pack.raw_measurements_(2);
      phi = atan2(sin(phi),cos(phi));
      ekf_.x_ << roh*cos(phi), roh*sin(phi), 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /*
      Initialize state. DONE
      */
      ekf_.x_ << measurement_pack.raw_measurements_(0,0), measurement_pack.raw_measurements_(1,0), 0., 0.;
      
    }
    if(fabs(ekf_.x_(0))< 0.0001 and fabs(ekf_.x_(1)))
    {
    	ekf_.x_(0) = 0.0001;
    	ekf_.x_(1) = 0.0001;
    }
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt*dt;
  float dt_3 = dt_2*dt;
  float dt_4 = dt_3*dt;

  // Modify the F matrix so that the time is integrated RWM
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  float noise_ax = 9.;
  float noise_ay = 9.;
  // Set the process covariance Matrix Q RWM
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (dt_4/4)*noise_ax, 0, (dt_3/2)*noise_ax, 0,
             0, (dt_4/4)*noise_ay, 0, (dt_3/2)*noise_ay,
             (dt_3/2)*noise_ax, 0, dt_2*noise_ax, 0,
             0, (dt_3/2)*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
