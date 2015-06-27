/*
 * Tracker.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/Track.h>

Track::Track(Eigen::Vector2d initialPos):
	initialPos_(initialPos),
	kalmanFilter(initialPos_)
{
	// TODO Auto-generated constructor stub

}

Track::~Track() {
	// TODO Auto-generated destructor stub
}

