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
	A_ = Eigen::Matrix<double,-1,-1>::Identity(4,4);

	state_[0] = initialState[0];
	state_[1] = initialState[1];
	state_[2] = 0.0;
	state_[3] = 0.0;

	std::cout << "A Kalman filter was created, current state " << std::endl << state_ <<  std::endl;

}

KalmanFilter::~KalmanFilter() {
	std::cout << "A KalmanFilter was removed " <<  std::endl;
}

