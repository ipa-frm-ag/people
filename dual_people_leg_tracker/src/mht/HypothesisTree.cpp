/*
 * HypothesisManager.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/HypothesisTree.h>
#include <dual_people_leg_tracker/mht/Hypothesis.h>

using namespace mht;

HypothesisTree::HypothesisTree() {
	// TODO Auto-generated constructor stub

}

HypothesisTree::~HypothesisTree() {
	// TODO Auto-generated destructor stub
}


void HypothesisTree::setRootHypothesis(HypothesisPtr rootHypothesis){
	this->rootHypothesis_ = rootHypothesis;
}

void HypothesisTree::addHypothesis(HypothesisPtr hypothesis){

	// Get the cycle
	int cycle = hypothesis->getCycle();

	// TODO Check if this really works
	this->hypothesis[cycle].push_back(hypothesis);

	std::cout << " Hypothesis from cycle " << cycle << " was added to the tree" << std::endl;

}

// Predict all hypothesis of the cycle
void HypothesisTree::update(int cycle, ros::Time time, std::vector<DetectionPtr> detections){
	std::cout << "Hypothesis Tree is doing a update" << std::endl;
	// TODO Check if this really works
	for(std::vector<HypothesisPtr>::iterator hypoIt; this->hypothesis[cycle].begin() != this->hypothesis[cycle].end(); hypoIt++){
		//(*hypoIt)->assignMeasurements(detections, time);
		//(*hypoIt)->setTime(time);
	}

	std::cout << " Hypothesis from cycle " << cycle << " are assigned the new detections" << std::endl;

}
