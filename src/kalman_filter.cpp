#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
    // state vector
    x_ = VectorXd(3);

    // state covariance matrix
    P_ = MatrixXd(4,4);

    // state transistion matrix
    F_ = MatrixXd(4, 4);

    // process covariance matrix
    Q_ = MatrixXd(4, 4);

    // measurement matrix
    H_ = MatrixXd(2, 4);

    // measurement covariance matrix
    R_ = MatrixXd(2, 2);

}

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
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    VectorXd y = z - h(x_);
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::h(const VectorXd &x) {
    VectorXd hx = VectorXd(3);

    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    // h(x')
    float rho = sqrt(px*px + py*py);

    //check division by zero
    if(fabs(rho) < 0.0001){
        std::cout << "KalmanFilter::h() - Error - Division by Zero" << std::endl;
        return hx;
    }

    float theta = atan(py/px);
    float rho_dot = (px*vx + py*vy) / rho;


    hx << rho, theta, rho_dot;
    return hx;
}


