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

	bool is_occluded;

private:
	int id_; // The id

	ros::Time initialTime_; // Time the Track was created
	ros::Time lastPredictTime_; // Time the last prediction was made against
	ros::Time lastUpdateTime_; // Time towards the last update was done

	mht::KalmanFilter kalmanFilter_; // The Kalman Filter

	std::vector<ros::Time> estimation_times_; // The update and prediction times

  bool is_approved_; // Flag if this track is approved

  bool is_occluded_;

public:
	Eigen::Vector2d initialPos_;

	std::vector<Eigen::Vector4d> estimated_states_;


public:
	Track(Eigen::Vector2d initialPos, ros::Time time);
	Track(const Track &obj);
	virtual ~Track();

	// Return the id
	int getId() const{ return this->id_; };

	// Predict using the Kalman Filter towards the given time
	void predict(ros::Time time);

	// Get the measurement prediction
	Eigen::Matrix<double,2,1> getMeasurementPrediction();

	//Do a update using the lastest measurements
	void update(Eigen::Vector2d meas, ros::Time time);

	void print();

	bool isApproved() const { return this->is_approved_; };

	void setApproved();

	void unApprove();

	double getMeasurementLikelihood(Eigen::Vector2d meas);

	void setOccluded(ros::Time time);
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_ */
