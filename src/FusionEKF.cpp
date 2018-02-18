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

    // H matrix
    // only for laser, for radar the Jacobian will be calculated each point depending on the state
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    
    // the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<  1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    // initialize state
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    meas_processed = true;
    if((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) & !useRadar){
        meas_processed = false;
        return;
    }
    if((measurement_pack.sensor_type_ == MeasurementPackage::LASER) & !useLaser){
        meas_processed = false;
        return;
    }
    
    // print the output
    cout << measurement_pack.sensor_type_;
    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR) cout<< ". Radar"<< endl;
    if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) cout<< ". Laser"<< endl;
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {

        // First measurement
        cout << "EKF: " << endl;
        
        // Initialize state (depending on type of measurement Lidar OR Radar
        // ekf_.x_ = VectorXd(4);
        // ekf_.x_ << 1, 1, 1, 1;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            //get data from raw measurement
            float ro = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            //convert from polar to cartesian cordinates
            //only position as theta is the angle for position not for velocity
            float px = ro * cos(phi);
            float py = ro * sin(phi);
            //initialize the state
            ekf_.x_ << px, py, 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            //only position measurements to initialize the state
            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];
            ekf_.x_ << px, py, 0, 0;
        }
        
        // Initialize P
        ekf_.P_ = MatrixXd::Zero(4, 4);
        ekf_.P_(0,0) = 1;
        ekf_.P_(1,1) = 1;
        ekf_.P_(2,2) = 1000;
        ekf_.P_(3,3) = 1000;
        
        // Update previous timestamp
        previous_timestamp_ = measurement_pack.timestamp_;

        // Done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    // Update F_ transition matrix to consider elapsed time
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    
    // Calculate process noise matrix Q
    MatrixXd G = MatrixXd::Zero(4, 2);
    G(0,0) = dt*dt/2.;
    G(1,1) = dt*dt/2.;
    G(2,0) = dt;
    G(3,1) = dt;
    
    MatrixXd Eaa = MatrixXd::Zero(2, 2);
    Eaa(0,0) = 9;
    Eaa(1,1) = 9;
    
    ekf_.Q_ = G * Eaa * G.transpose();
    
    // Make the prediction
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    // Update H_ and R_ depending on the type of sensor
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    
    // And make the update step

    //cout << "F_ = " << endl << ekf_.F_ << endl;
    //cout << "Q_ = " << endl << ekf_.Q_ << endl;
    //cout << "H_ = " << endl << ekf_.H_ << endl;
    //cout << "S_ = " << endl << ekf_.S_ << endl;
    //cout << "K_ = " << endl << ekf_.K_ << endl;
    //cout << "x_ = \t" << ekf_.x_.transpose() << endl;
    //cout << "P_ = " << endl << ekf_.P_ << endl;
}
