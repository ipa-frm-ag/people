/*
 * Hypothesis.h
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESIS_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESIS_H_

#undef NDEBUG

// ROS includes
#include <ros/ros.h>

// Own includes
#include <dual_people_leg_tracker/detection/detection.h>
#include <dual_people_leg_tracker/jpda/murty.h>

// Boost includes
#include <boost/shared_ptr.hpp>

// Eigen includes
#include <eigen3/Eigen/Dense>

// System includes
#include <vector>

class Hypothesis; // Forward declaration
typedef boost::shared_ptr<Hypothesis> HypothesisPtr;


class Hypothesis {
private:
	unsigned int numberOfTracks_;

	// CostValues
	int newTrackCostValue_;
	int falseAlarmCostValue_;
	int deletionCostValue_;
	int occlusionCostValue_;

	int invalidValue_;

	// The current cost matrix
	Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> costMatrix;

	// Numbers
	unsigned int numberOfMeasurements_;

public:
	Hypothesis();
	virtual ~Hypothesis();

	/***
	 * Get the number of current Tracks hold by this hypothesis
	 */
	unsigned int getNumberOfTracks();

	// Assign the measurements (this is done in every cycle)
	bool assignMeasurements(std::vector<DetectionPtr> detections);

	// Create the cost matrix
	bool createCostMatrix();

	// Solve the cost matrix using murty
	bool solveCostMatrix();
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESIS_H_ */
