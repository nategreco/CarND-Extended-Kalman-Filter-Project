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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0.0000,
    	      0.0000, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.0900, 0.0000, 0.0000,
     	      0.0000, 0.0009, 0.0000,
   	          0.0000, 0.0000, 0.0900;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
		      0, 1, 0, 0;

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
    cout << "Initializing EKF" << endl;
    VectorXd x;
	x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  //Polar -> Cartesian
	  float px = measurement_pack.raw_measurements_[0] *
		cos(measurement_pack.raw_measurements_[1]); 
	  float py = measurement_pack.raw_measurements_[0] *
		sin(measurement_pack.raw_measurements_[1]);
	  float vx = measurement_pack.raw_measurements_[2] *
		cos(measurement_pack.raw_measurements_[1]);
	  float vy = measurement_pack.raw_measurements_[2] *
		sin(measurement_pack.raw_measurements_[1]);
	  x << px, py, vx , vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  x << measurement_pack.raw_measurements_[0],
		   measurement_pack.raw_measurements_[1],
		   0,
		   0;
    }

	//Prevent division by zero with dataset 2
    if (fabs(x[0]) < 0.0001 && fabs(x[1]) < 0.0001) {
	  x[0] = 0.0001;
	  x[1] = 0.0001;
	}
	  
	//Initialize covariance matrix
	MatrixXd P;
	P = MatrixXd(4,4);
	P << 1, 0, 0,    0,
	     0, 1, 0,    0,
	     0, 0, 1000, 0,
	     0, 0, 0,    1000;

	//Update state transition matrix
	MatrixXd F;
	F = MatrixXd(4,4);
	F << 1, 0, 0, 0,
	     0, 1, 0, 0,
	     0, 0, 1, 0,
	     0, 0, 0, 1;	
	  
	//Init call
    ekf_.Init(x, //x_in
              P, //P_in
              F); //F_in
	  
	//Save initial timestamp
	previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //Get time elapsed
  float t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
	
  //Update state transition matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, t, 0,
			 0, 1, 0, t,
	         0, 0, 1, 0,
			 0, 0, 0, 1;
  //Noise covariance matrix calculation
  float ax = 9;
  float ay = 9;

  //Get some exponenets of t
  float t2 = t*t;
  float t3 = t2*t;
  float t4 = t2*t2;
  float t44 = t4/4;
  float t32 = t3/2;

  //Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << t44*ax, 0,      t32*ax, 0,
			 0     , t44*ay, 0,      t32*ay,
			 t32*ax, 0,      t2*ax,  0,
			 0,      t32*ay, 0,      t2*ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	//Use Jacobian instead of H
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
	//Use H
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
