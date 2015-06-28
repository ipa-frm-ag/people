/*
 * Tracker.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/Track.h>

Track::Track(Eigen::Vector2d initialPos, ros::Time initialTime):
	initialPos_(initialPos),
	kalmanFilter_(initialPos),
	initialTime_(initialTime),
	lastPredictTime_(initialTime_),
	state_(Track::FREE),
	id_(trackIdCounter++)
{
	std::cout << "A new track with id " << getId() << " was created at " << this->initialPos_ << std::endl;
}

Track::~Track() {

}

Track::Track(const Track &obj):
	initialPos_(obj.initialPos_),
	kalmanFilter_(obj.kalmanFilter_),
	initialTime_(obj.initialTime_),
	lastPredictTime_(obj.lastPredictTime_),
	state_(obj.state_),
	id_(trackIdCounter++)
{
    std::cout << "Constructing copy of the track with id" << this->getId() << std::endl;
}

void Track::predict(ros::Time time){

	// Calculate the time difference
	double dt = (time - this->lastPredictTime_).toSec();
	//std::cout << " prediction with " << dt << " s " << std::endl;

	// Assert positivness
	ROS_ASSERT(dt > 0.0);

	// Do prediction
	kalmanFilter_.predict(dt);

	// Finished -> Increase the prediction time
	this->lastPredictTime_ = time;
}

void Track::update(Eigen::Vector2d meas){
	kalmanFilter_.update(meas);
}

// Get the measurement prediction
Eigen::Matrix<double,2,1> Track::getMeasurementPrediction(){
	return  kalmanFilter_.getMeasurementPrediction();
}
