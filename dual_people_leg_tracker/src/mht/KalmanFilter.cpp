/*
 * KalmanFilter.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/KalmanFilter.h>
#include <iostream>


using namespace mht;

KalmanFilter::KalmanFilter(Eigen::Matrix<double,2,1> initialState) {

	// Set the state
	state_[0] = initialState[0];
	state_[1] = initialState[1];
	state_[2] = 0.0;
	state_[3] = 0.0;

	// Define the System matrix
	A_ = Eigen::Matrix<double,-1,-1>::Identity(4,4);


	// Define the measurement Matrix
	H_ << 1, 0, 0, 0,
		  0, 1, 0,  0;

	P_prior_ = Eigen::Matrix<double,-1,-1>::Identity(4,4);
	P_post_  = Eigen::Matrix<double,-1,-1>::Identity(4,4);

	Q_ = Eigen::Matrix<double,-1,-1>::Identity(4,4)*0.0001;

	R_ = Eigen::Matrix<double,-1,-1>::Identity(2,2)*0.001;



	std::cout << "A Kalman filter was created, current state " << std::endl << state_ <<  std::endl;
	std::cout << "System is " << std::endl << A_ <<  std::endl;
	std::cout << "Meas is " << std::endl << H_ <<  std::endl;
	std::cout << "P_prior_ is " << std::endl << P_prior_ <<  std::endl;
	std::cout << "Q_ is " << std::endl << Q_ <<  std::endl;
	std::cout << "R_ is " << std::endl << R_ <<  std::endl;

}

KalmanFilter::~KalmanFilter() {
	std::cout << "A KalmanFilter was removed " <<  std::endl;
}

void KalmanFilter::predict(double dt){
	Eigen::Matrix<double,4,4> A_dt;

	A_dt = A_;
	A_dt(0,2) = dt;
	A_dt(1,3) = dt;

	//std::cout << "Doing prediction with " << std::endl << A_dt << std::endl;

	state_predicted = A_dt * state_;

	P_prior_ = A_dt * P_post_ * A_dt.transpose() + Q_;

	//std::cout << "state_"  << std::endl << state_ << std::endl;
	//std::cout << "state_predicted"  << std::endl << state_predicted << std::endl;

	state_ =  state_predicted;
}

void KalmanFilter::update(Eigen::Matrix<double,2,1> z_k){

	// Residual
	Eigen::Matrix<double,2,1> y_k;
	y_k = z_k - H_ * state_predicted;

	// Residual covariance
	S_k = H_ * P_prior_ * H_.transpose() + R_;

	// Kalman Gain
	Eigen::Matrix<double,4,2> K_k;
	K_k = P_prior_ * H_.transpose() * S_k.inverse();

	// State estimation
	state_estimated = state_predicted + K_k * y_k;

	// Update (a posterior) state covariance
	Eigen::Matrix<double,4,4> I = Eigen::Matrix<double,4,4>::Identity();
	P_post_ = (I - K_k * H_) + P_prior_;

	state_ = state_estimated;



}

Eigen::Matrix<double,4,1> KalmanFilter::getPrediction(){
	return this->state_predicted;
}

Eigen::Matrix<double,4,1> KalmanFilter::getEstimation(){
	return this->state_estimated;
}

