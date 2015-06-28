/*
 * NewTrackAssignment.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/NewTrackAssignment.h>


NewTrackAssignment::NewTrackAssignment(Eigen::Matrix<double,2,1> meas) {
	pos_ = meas;
}

NewTrackAssignment::~NewTrackAssignment() {
}

