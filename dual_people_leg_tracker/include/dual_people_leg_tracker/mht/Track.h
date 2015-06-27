/*
 * Tracker.h
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_

// Eigen
#include <Eigen/Dense>

// Boost
#include <boost/shared_ptr.hpp>

// Own includes
#include <dual_people_leg_tracker/mht/KalmanFilter.h>

class Track; // Forward

typedef boost::shared_ptr<Track> TrackPtr;

class Track {
public:
	Eigen::Vector2d initialPos_;

	mht::KalmanFilter kalmanFilter;

public:
	Track(Eigen::Vector2d initialPos);
	virtual ~Track();
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_TRACK_H_ */
