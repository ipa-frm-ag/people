#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESIS_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESIS_H_

#undef NDEBUG

// ROS includes
#include <ros/ros.h>

// Own includes
#include <dual_people_leg_tracker/detection/detection.h>
#include <dual_people_leg_tracker/jpda/murty.h>
#include <dual_people_leg_tracker/visualization/color_definitions.h>
#include <dual_people_leg_tracker/mht/Track.h>
#include <dual_people_leg_tracker/mht/NewTrackAssignment.h>

// Boost includes
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

// Eigen includes
#include <eigen3/Eigen/Dense>

// System includes
#include <vector>

class Hypothesis; // Forward declaration
typedef boost::shared_ptr<Hypothesis> HypothesisPtr;


class Hypothesis :  public boost::enable_shared_from_this<Hypothesis>{
private:
	unsigned int numberOfTracks_;

	// CostValues
	int newTrackCostValue_;
	int falseAlarmCostValue_;
	int deletionCostValue_;
	int occlusionCostValue_;

	int invalidValue_;

	int cycle_; // The cycle to which this belongs

    std::vector<Solution> solutions;

    std::vector<HypothesisPtr> children;

    double probability_; // Probability of this hypothesis

    std::vector<DetectionPtr> detections_;

    HypothesisPtr parent_;

    std::vector<TrackPtr> tracks_;

//	extern static enum LABEL{
//	    OCCLUDED,
//		DELETED,
//		DETECTED,
//		NEW_TRACK,
//		FALSE_ALARM
//	};

	// The current cost matrix
	Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> costMatrix;

	// Numbers
	unsigned int numberOfMeasurements_;

public:
	Hypothesis(int cycle);
	virtual ~Hypothesis();

	/***
	 * Get the number of current Tracks hold by this hypothesis
	 */
	unsigned int getNumberOfTracks();

	long double getProbability(){
		return this->getProbability();
	}

	long double getParentProbabilility(){
		  if(this->parent_)
			  return this->parent_->getProbability();

		  return 1.0;
	}

	// Assign the measurements (this is done in every cycle)
	bool assignMeasurements(std::vector<DetectionPtr> detections);

	// Create the cost matrix
	bool createCostMatrix();

	// Solve the cost matrix using murty
	bool solveCostMatrix();

	// Std Cout the Solutions
	bool stdCoutSolutions();

	// Create Children
	bool createChildren();

	// Set the parent
	void setParent(HypothesisPtr parent);

	// Add a track to this hypothesis
	void addTrack(TrackPtr track);
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESIS_H_ */
