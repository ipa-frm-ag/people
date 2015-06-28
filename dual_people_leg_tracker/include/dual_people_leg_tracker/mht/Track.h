/*
 * Tracker.h
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_

// ROS includes
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

// Boost
#include <boost/shared_ptr.hpp>

// Own includes
#include <dual_people_leg_tracker/mht/KalmanFilter.h>

class Track; // Forward

typedef boost::shared_ptr<Track> TrackPtr;

static int trackIdCounter = 0;

class Track {
public:

	enum STATE{FREE, ASSOCIATED};

	STATE state_;

private:
	int id_; // The id

	ros::Time initialTime_; // Time the Track was created
	ros::Time lastPredictTime_; // Time the last prediction was made against

	mht::KalmanFilter kalmanFilter_; // The Kalman Filter

public:
	Eigen::Vector2d initialPos_;





public:
	Track(Eigen::Vector2d initialPos, ros::Time time);
	Track(const Track &obj);
	virtual ~Track();

	// Return the id
	int getId(){ return this->id_; };

	// Predict using the Kalman Filter towards the given time
	void predict(ros::Time time);

	// Get the measurement prediction
	Eigen::Matrix<double,2,1> getMeasurementPrediction();
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_ */
