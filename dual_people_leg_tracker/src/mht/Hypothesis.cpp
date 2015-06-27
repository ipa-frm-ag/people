/*
 * Hypothesis.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/Hypothesis.h>

Hypothesis::Hypothesis():
	numberOfTracks_(3),
	newTrackCostValue_(10),
	falseAlarmCostValue_(10),
	deletionCostValue_(10),
	occlusionCostValue_(10),
	invalidValue_(-1000)
{

}

Hypothesis::~Hypothesis() {
	// TODO Auto-generated destructor stub

}

unsigned int Hypothesis::getNumberOfTracks(){
	return this->numberOfTracks_;
}

bool Hypothesis::assignMeasurements(std::vector<DetectionPtr> detections){
	numberOfMeasurements_ = detections.size();
	numberOfMeasurements_ = 4;
	return true;
}

bool Hypothesis::createCostMatrix(){

    // Generate Matrix
    //std::cout << "There are currently " << nObjects << " objects and " << nMeasurements << " measurements(occlusion included)" << std::endl;

	unsigned int numberOfCols = numberOfTracks_ 	  + 2 * numberOfMeasurements_;
	unsigned int numberOfRows = numberOfMeasurements_ + 2 * numberOfTracks_;

	if(numberOfTracks_ > numberOfMeasurements_){
		// Initialize
		unsigned int size = numberOfMeasurements_ + 2 * numberOfTracks_;
	    costMatrix = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Constant(size,size,this->invalidValue_);
	}else{
		// Initialize
		unsigned int size = numberOfTracks_ 	  + 2 * numberOfMeasurements_;
	    costMatrix = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Constant(size,size,this->invalidValue_);
	}

    // Print the cost matrix
    std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;




    // Fill the new tracks
    costMatrix.block(numberOfTracks_,0,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,this->newTrackCostValue_);

    std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;

    // Fill the false alarms
    costMatrix.block(numberOfTracks_+numberOfMeasurements_,0,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,this->falseAlarmCostValue_);

    // Fill the Deletions
    costMatrix.block(0,numberOfMeasurements_,numberOfTracks_,numberOfTracks_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks_,1,this->deletionCostValue_);

    // Fill the Occlusions
    costMatrix.block(0,numberOfMeasurements_+numberOfTracks_,numberOfTracks_,numberOfTracks_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks_,1,this->occlusionCostValue_);

    costMatrix.block(numberOfTracks_,numberOfMeasurements_,2*numberOfMeasurements_,2*numberOfTracks_) = Eigen::Matrix< int, -1, -1>::Constant(2*numberOfMeasurements_,2*numberOfTracks_,-100);
    if(numberOfTracks_ > numberOfMeasurements_){
    	costMatrix.block(numberOfTracks_,numberOfMeasurements_,2*numberOfMeasurements_,2*numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(2*numberOfMeasurements_,1,0);
    }else{
    	costMatrix.block(numberOfTracks_,numberOfMeasurements_,2*numberOfTracks_,2*numberOfTracks_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(2*numberOfTracks_,1,0);
    }

    // Fill the validated measurements (using mahalanobis distance)
    for(unsigned int t=0; t < numberOfTracks_; t++){
        for(unsigned int m=0; m < numberOfMeasurements_; m++){
        	costMatrix(t, m) = (int) (((double) rand() / (RAND_MAX)) * 200);
        }
    }

    // Print the cost matrix
    std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;
	return true;
}

bool Hypothesis::solveCostMatrix(){
    //std::cout << std::endl << "Cost Matrix:" << std::endl  << costMatrix << std::endl;
    std::vector<Solution> solutions;

    // TODO depend this on the number of measurements
    int k = 20;
    solutions = murty(-costMatrix,k);

    // TODO Filter the solution regarding several parameters using the leg tracker information
    // TODO Calculate the crossing value of the solutions in order to reject obvious unrealistic solutions

    std::cout << "Solutions are:" << std::endl;
    for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      color_print_solution(costMatrix,solIt->assignmentMatrix);
      std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;
    }

    return true;
}
