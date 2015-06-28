/*
 * Assignment.cpp
 *
 *  Created on: Jun 28, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/TrackAssignment.h>

// Detection
TrackAssignment::TrackAssignment(TrackPtr track, int meas, TrackAssignment::LABEL label){
	ROS_ASSERT(label == TrackAssignment::DETECTED);
	this->type_ = TrackAssignment::DETECTED;
	this->track_ = track;
	this->meas_ = meas;
}

// Deletion or Occlusion
TrackAssignment::TrackAssignment(TrackPtr track, TrackAssignment::LABEL label){
	ROS_ASSERT(label == TrackAssignment::OCCLUDED || label == TrackAssignment::DELETED);
	this->type_ = label;
	this->track_ = track;
}

// New
TrackAssignment::TrackAssignment(int meas, TrackAssignment::LABEL label){
	ROS_ASSERT(label == TrackAssignment::NEW);
	this->type_ = TrackAssignment::NEW;
	this->meas_ = meas;
}

// FALSE
TrackAssignment::TrackAssignment(TrackAssignment::LABEL label){
	ROS_ASSERT(label == TrackAssignment::FALSEALARM);
	this->type_ == label;
}

TrackAssignment::~TrackAssignment() {

}

std::ostream& operator<<(std::ostream& os, const TrackAssignment& dt){
	switch(dt.type_){
		case TrackAssignment::DETECTED:
			os << "TRACK[" << dt.track_->getId() << "] is DETECTED BY MEAS[" << dt.meas_ << "]";
			break;
		case TrackAssignment::OCCLUDED:
			os << "TRACK[" << dt.track_->getId() << "] is OCCLUDED";
			break;
		case TrackAssignment::DELETED:
			os << "TRACK[" << dt.track_->getId() << "] is DELETED";
			break;
		case TrackAssignment::NEW:
			os << "NEWTRACK";
			break;
		case TrackAssignment::FALSEALARM:
			os << "MEAS[" << dt.meas_ << "] FALSE ALARM";
			break;
	}
	return os;

}
