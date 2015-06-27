/*
 * NewTrackAssignment.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/NewTrackAssignment.h>


NewTrackAssignment::NewTrackAssignment(tf::Stamped<tf::Vector3>& pos) {
	// TODO Auto-generated constructor stub
	pos_[0] = pos[0];
	pos_[1] = pos[1];
}

NewTrackAssignment::~NewTrackAssignment() {
	// TODO Auto-generated destructor stub
}

