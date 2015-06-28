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
	estimated_states_.push_back(kalmanFilter_.getEstimation());
	estimation_times_.push_back(initialTime);
	std::cout << "A new track with id " << getId() << " was created at " << this->initialPos_ << std::endl;
}

Track::~Track() {
	std::cout << "Track removed" << std::endl;
}

Track::Track(const Track &obj):
	initialPos_(obj.initialPos_),
	kalmanFilter_(obj.kalmanFilter_),
	initialTime_(obj.initialTime_),
	lastPredictTime_(obj.lastPredictTime_),
	state_(obj.state_),
	id_(trackIdCounter++),
	estimation_times_(obj.estimation_times_),
	estimated_states_(obj.estimated_states_)
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

void Track::update(Eigen::Vector2d meas, ros::Time time){
	kalmanFilter_.update(meas);

	this->lastUpdateTime_ = time;
	this->estimation_times_.push_back(time);
	this->estimated_states_.push_back(kalmanFilter_.getEstimation());

	ROS_ASSERT(this->lastUpdateTime_ == this->lastPredictTime_); // Both times should be equal
}

// Get the measurement prediction
Eigen::Matrix<double,2,1> Track::getMeasurementPrediction(){
	return  kalmanFilter_.getMeasurementPrediction();
}

void Track::print(){
	for(size_t i = 0; i < this->estimated_states_.size(); i++){
		std::cout << "  " << this->estimated_states_[i].transpose() << std::endl;
	}
}
