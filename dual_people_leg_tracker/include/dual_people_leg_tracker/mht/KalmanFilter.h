/*
 * KalmanFilter.h
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_

// Eigen includes
#include <Eigen/Dense>

namespace mht{

class KalmanFilter {
public:
	Eigen::Matrix<double,4,4> A_; // Transition Matrix

	Eigen::Matrix<double,4,4> P_; // State Vector Estimation Error

	Eigen::Matrix<double,4,4> Q_; // Process Covariance

	Eigen::Matrix<double,2,2> R_; // Process Covariance

	Eigen::Matrix<double,4,1> state_; // The current State

	Eigen::Matrix<double,4,1> state_predicted; // The current State

public:
	KalmanFilter(Eigen::Matrix<double,2,1> initialState);
	virtual ~KalmanFilter();
};
}
#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_ */
