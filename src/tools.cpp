#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth){
    
    //rmse delcaration
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    //check data consistency
    if(estimations.size() == 0){
        cout<< "Estimation vector size should not be zero" << endl;
    }
    if(estimations.size() != ground_truth.size()){
        cout<< "Estimation vector size should equal ground truth vector size" << endl;
    }
    
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse /= estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    //return the result
    return rmse;
    
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state){
    
    //jacobian matrix declaration
    MatrixXd Hj(3,4);
    
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //pre-compute sum of squares to facilitate calculations
    float sumPx2Py2 = px*px + py*py;
    
    //check division by zero
    if(sumPx2Py2 == 0){
        cout<<"CalculateJacobian () - Error - Division by Zero";
        return Hj;
    }
    
    //compute the Jacobian matrix
    Hj = MatrixXd::Zero(3,4);
    Hj(0, 0) = px / sqrt(sumPx2Py2);
    Hj(0, 1) = py / sqrt(sumPx2Py2);
    Hj(1, 0) = - py / sumPx2Py2;
    Hj(1, 1) = px / sumPx2Py2;
    Hj(2, 0) = py * (vx*py - vy*px) / pow(sumPx2Py2, 1.5);
    Hj(2, 1) = px * (vy*px - vx*py) / pow(sumPx2Py2, 1.5);
    Hj(2, 2) = px / sqrt(sumPx2Py2);
    Hj(2, 3) = py / sqrt(sumPx2Py2);
    
    return Hj;
    
}
