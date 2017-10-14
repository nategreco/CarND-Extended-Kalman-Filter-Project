#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /*
	Calculation of RMSE
  */
	//Initialization
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	
	//Check inputs
	if ((estimations.size() != 0) && (estimations.size() == ground_truth.size())) {
		//Get sums of squared error
		for (int i=0; i< estimations.size(); ++i) {
			//Get Error
			VectorXd err = estimations[i] - ground_truth[i];
			//Square
			err = err.array()*err.array();
			//Sum
			rmse += err;
		}
		//Mean
		rmse = rmse / estimations.size();
		
		//Root
		rmse = rmse.array().sqrt();
	} else {
		//Bogus data, future TODO - throw invalid data exception
	}
	
	//Return data
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /*
	Calculation of Jacobian
  */

	//Declare Jacobian
	MatrixXd Hj(3,4);
	
	//Get position/velocities/coefficients
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	float c1 = px*px+py*py;
	
	//Check division by zero
	if (fabs(px) < 0.0001 and fabs(py) < 0.0001) {
		px = 0.0001;
		py = 0.0001;
	}
	if(fabs(c1) < 0.000001){
		c1 = 0.000001;
	}
	
	//Get other coefficients
	float c2 = sqrt(c1);
	float c3 = c1 * c2;

	//Compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		 -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
	
	//Return value
	return Hj;
}
