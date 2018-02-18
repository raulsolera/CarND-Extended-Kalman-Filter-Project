#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

    //predict new state
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    
    //update
    VectorXd y = z - H_ * x_;
    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();
    
    //new estimate
    x_ = x_ + (K_ * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    
    // Calculate h(x)
    VectorXd hx(3);
    
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    
    double rho = 0;
    double phi = 0;
    double rho_dot = 0;
    
    rho = sqrt(px*px + py*py);
    // avoid division by zero
    if(fabs(px) < 0.0001){
        std::cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << std::endl;
    }
    else {
        phi = atan2(py, px);
    }
    rho_dot = (px*vx+py*vy) / rho;
    hx << rho, phi, rho_dot;
   
    //update
    VectorXd y = z - hx;
    if (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }
    if (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
    }
    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();
    
    //new estimate
    x_ = x_ + (K_ * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K_ * H_) * P_;

    
}
