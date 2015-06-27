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
