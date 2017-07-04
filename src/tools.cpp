#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO: DONE
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check validity of ground truth data RWM
  if(estimations.size() != ground_truth.size()
  		|| estimations.size() == 0){
  	std::cout<< "Invalid: Ground truth data estimation incorrect" << endl;
  	return rmse;
  }

  // Accumulate squared residuals RWM
  for(unsigned int i=0; i < estimations.size(); i++){

  	VectorXd residual = estimations[i] - ground_truth[i];

  	// coefficient-wise multiplication RWM
  	residual = residual.array()*residual.array();
  	rmse += residual;
  }

  // Calculate mean RWM
  rmse = rmse/estimations.size();

  // Calculate the squared root RWM
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO: DONE
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	// Recover state parameters RWM
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	// Pre-compute a set of terms for EoU RWM
	float c1 = (px*px)+(py*py);
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
	//cout<<"Simplifying Calcs done"<<endl;////////////////////

	// Check validity of Calc RWM
	if(fabs(c1)<0.0001){
		//cout<<"Calculate Jacobian () - Error - Division by Zero" << endl;
		c1 = 0.001;
		//return Hj;
	}
	//cout<<"Checked validity of the Jacobian calc to be done"<<endl;////////////////////////////////
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    //cout<<"Completed Jacobian Calc"<<endl;///////////////////////////////////////
    return Hj;
}
