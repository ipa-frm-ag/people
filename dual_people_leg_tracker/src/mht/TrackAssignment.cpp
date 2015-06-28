/*
 * Assignment.cpp
 *
 *  Created on: Jun 28, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/TrackAssignment.h>

// Detection
TrackAssignment::TrackAssignment():
	is_deletion_(false),
	is_occlusion_(false),
	is_falsealarm_(false),
	is_new_(false),
	is_detection_(false)
{

}

TrackAssignment::~TrackAssignment() {

}

/*std::ostream& operator<<(std::ostream& os, const TrackAssignment& dt){
	switch(dt.type_){
		case DETECTED:
			os << "TRACK[" << dt.getTrack()->getId() << "] is DETECTED BY MEAS[" << dt.meas_ << "]";
			break;
		case OCCLUDED:
			os << "TRACK[" << dt.getTrack()->getId() << "] is OCCLUDED";
			break;
		case DELETED:
			os << "TRACK[" << dt.getTrack()->getId() << "] is DELETED";
			break;
		case NEW:
			os << "NEWTRACK";
			break;
		case FALSEALARM:
			os << "MEAS[" << dt.meas_ << "] FALSE ALARM";
			break;
		default:
			std::cout << "Somethink is wrong" << std::endl;
			ROS_ASSERT(false);
			break;
	}
	return os;

}*/

void TrackAssignment::print(){

	if(isDetection())
		std::cout << "TRACK[" << getTrack()->getId() << "] is DETECTED BY MEAS[" << meas_ << "]" << std::endl;

	if(isOcclusion())
		std::cout << "TRACK[" << getTrack()->getId() << "] is OCCLUDED" << std::endl;

	if(isDeletion())
		std::cout << "TRACK[" << getTrack()->getId() << "] is DELETED" << std::endl;

	if(isNew())
		std::cout << "NEWTRACK" << std::endl;

	if(isFalseAlarm())
		std::cout << "MEAS[" << meas_ << "] FALSE ALARM" << std::endl;

}

TrackPtr TrackAssignment::getTrack() const{
	ROS_ASSERT(isDetection()  || isOcclusion() || isDeletion());
	return this->track_;
}
