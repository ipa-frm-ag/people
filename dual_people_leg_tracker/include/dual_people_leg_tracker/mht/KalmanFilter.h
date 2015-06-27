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

	Eigen::Matrix<double,2,4> H_; // Transition Matrix

	Eigen::Matrix<double,4,4> P_prior_; // Predicted (a priori) estimate covariance

	Eigen::Matrix<double,4,4> P_post_; // Updated (a posteriori) estimate covariance

	Eigen::Matrix<double,4,4> Q_; // Process Covariance

	Eigen::Matrix<double,2,2> R_; // Measurement Covariance

	Eigen::Matrix<double,2,2> S_k; // Residual Covariance

	Eigen::Matrix<double,4,1> state_; // The current State

	Eigen::Matrix<double,4,1> state_predicted; // The current State

	Eigen::Matrix<double,4,1> state_estimated; // The state estimation

public:
	KalmanFilter(Eigen::Matrix<double,2,1> initialState);
	virtual ~KalmanFilter();

	// Do the prediction
	void predict(double dt);

	// Get the prediction
	Eigen::Matrix<double,4,1> getPrediction();

	// Get the prediction
	Eigen::Matrix<double,4,1> getEstimation();

	// Do a update
	void update(Eigen::Matrix<double,2,1>);
};
}
#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_KALMANFILTER_H_ */
