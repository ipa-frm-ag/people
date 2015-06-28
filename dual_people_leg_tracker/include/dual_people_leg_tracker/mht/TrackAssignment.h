/*
 * Assignment.h
 *
 *  Created on: Jun 28, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_TRACK_ASSIGNMENT_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_TRACK_ASSIGNMENT_H_

#include <dual_people_leg_tracker/mht/Track.h>

#include <iostream>

class TrackAssignment {
public:
	enum LABEL {DETECTED, OCCLUDED, DELETED, NEW, FALSEALARM};

	LABEL type_;
	int meas_;
	TrackPtr track_;

public:
	TrackAssignment(TrackPtr track, int meas, TrackAssignment::LABEL label);
	TrackAssignment(TrackPtr track, TrackAssignment::LABEL label);
	TrackAssignment(int meas, TrackAssignment::LABEL label);
	TrackAssignment(TrackAssignment::LABEL label);
	virtual ~TrackAssignment();
	friend std::ostream& operator<<(std::ostream& os, const TrackAssignment& dt);
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_TRACK_ASSIGNMENT_H_ */
