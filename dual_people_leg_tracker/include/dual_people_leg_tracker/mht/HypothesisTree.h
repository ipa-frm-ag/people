#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESISTREE_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESISTREE_H_

// Boost includes
#include <boost/shared_ptr.hpp>

// Own includes
#include <dual_people_leg_tracker/mht/Hypothesis.h>

class Hypothesis; // Forward declaration
typedef boost::shared_ptr<Hypothesis> HypothesisPtr;


class HypothesisTree; // Forward declaration

typedef boost::shared_ptr<HypothesisTree> HypothesisTreePtr;

class HypothesisTree {
public:
	// The root of this tree
	HypothesisPtr rootHypothesis_;

	std::map<int, std::vector<HypothesisPtr> > hypothesis;

public:
	HypothesisTree();
	virtual ~HypothesisTree();

	// Set the root hypothesis
	void setRootHypothesis(HypothesisPtr rootHypothesis);

	// Get the root hypothesis
	HypothesisPtr getRootHypothesis(){
		return this->rootHypothesis_;
	}

	// Add Hypothesis to handle
	void addHypothesis(HypothesisPtr hypothesis);

	// Update the hypothesis(assign new detections)
	void update(int cycle, ros::Time time, std::vector<DetectionPtr> detections);


};


#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MHT_HYPOTHESISTREE_H_ */
