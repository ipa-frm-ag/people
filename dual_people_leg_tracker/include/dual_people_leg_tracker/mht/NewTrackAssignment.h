/*
 * NewTrackAssignment.h
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_NEWTRACKASSIGNMENT_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_NEWTRACKASSIGNMENT_H_

#include <tf/transform_datatypes.h>
#include <tf/tf.h>

// Eigen
#include <Eigen/Dense>

class NewTrackAssignment {
public:
	Eigen::Vector2d pos_;

public:
	NewTrackAssignment(tf::Stamped<tf::Vector3>& pos);
	virtual ~NewTrackAssignment();
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_NEWTRACKASSIGNMENT_H_ */
