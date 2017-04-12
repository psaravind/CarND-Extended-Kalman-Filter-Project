#include "kalman_filter.h"

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
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	VectorXd z_polar_radar = VectorXd(3);
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);
	
	z_polar_radar(0) = c2;
	z_polar_radar(1) = atan2(py, px);
	
	if (c2 < 0.0001) {
		z_polar_radar(2) = (px * vx + py * vy) / 0.0001;
	} else {
		z_polar_radar(2) = (px * vx + py * vy) / c2;
	}

	VectorXd y = z - z_polar_radar;
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
  
	//new estimate
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I - K * H_) * P_;
}