#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
Tools tools;

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
  TODO: Done
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO: Done
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New Estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	cout<<"Laser Update"<<endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	//measurement covariance matrix - radar
	Tools tools;
	vector<VectorXd> x_state;
	VectorXd h_(3);
	double x = x_(0);
	double yy = x_(1);
	double vx = x_(2);
	double vy = x_(3);
    if(((x*x)<0.0001) || ((yy * yy) < 0.0001)){
    	h_ << 0, 0, 0;
    	return;

    }
    else {
    	float roh = sqrt((x*x)+(yy*yy));
		if(roh < 0.0001){
		roh = 0.001;
		}
		float phi = atan2(yy,x);
		phi = atan2(sin(phi),cos(phi));

		float rohdot = (((x_[0]*x_[2])+(x_[1]*x_[3]))/roh);
		h_ << roh, phi, rohdot;
    }
	
	

	VectorXd y = z - h_;
	y(1) = atan2(sin(y(1)),cos(y(1)));

	MatrixXd Hj = tools.CalculateJacobian(x_);
	MatrixXd Hj_t = Hj.transpose();
	MatrixXd S = Hj * P_ * Hj_t + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHj_t = P_*Hj_t;
	MatrixXd K = PHj_t * Si;
	//New Estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;
	cout<<"Radar Update"<<endl;

}
