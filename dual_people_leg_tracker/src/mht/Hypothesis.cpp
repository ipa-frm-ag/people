/*
 * Hypothesis.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/Hypothesis.h>
#include <dual_people_leg_tracker/mht/HypothesisTree.h>

// Parameters (from Paper)
const long double prob_detected_free = 0.3;
const long double prob_occluded_free = 0.63;
const long double prob_deleted_free = 0.07;
const long double prob_detected_approved = 0.2;
const long double prob_occluded_approved = 0.79;
const long double prob_deleted_approved = 0.01;

const long double prob_new = 0.001;
const long double prob_fal = 0.003;

Hypothesis::Hypothesis(int cycle, HypothesisTreePtr globalHypothesisTree):
	numberOfTracks_(),
	newTrackCostValue_(20),
	falseAlarmCostValue_(10),
	deletionCostValue_(50),
	occlusionCostValue_(120),
	invalidValue_(-1000),
	probability_(1.0),
	cycle_(cycle),
	globalHypothesisTree_(globalHypothesisTree)
{
	std::cout << "New Hypothesis create for cycle " << cycle << std::endl;
}

Hypothesis::~Hypothesis() {
	// TODO Auto-generated destructor stub

}

unsigned int Hypothesis::getNumberOfTracks(){
	return this->numberOfTracks_;
}

bool Hypothesis::assignMeasurements(std::vector<DetectionPtr> detections){
	this->detections_ = detections;

	numberOfMeasurements_ = detections.size();
	numberOfMeasurements_ = 10;
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
    //std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;




    // Fill the new tracks
    costMatrix.block(numberOfTracks_,0,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,this->newTrackCostValue_);

    //std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;

    // Fill the false alarms
    costMatrix.block(numberOfTracks_+numberOfMeasurements_,0,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,this->falseAlarmCostValue_);

    // Fill the Deletions
    costMatrix.block(0,numberOfMeasurements_,numberOfTracks_,numberOfTracks_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks_,1,this->deletionCostValue_);

    // Fill the Occlusions
    costMatrix.block(0,numberOfMeasurements_+numberOfTracks_,numberOfTracks_,numberOfTracks_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks_,1,this->occlusionCostValue_);

    costMatrix.block(numberOfTracks_,numberOfMeasurements_,2*numberOfMeasurements_,2*numberOfTracks_) = Eigen::Matrix< int, -1, -1>::Constant(2*numberOfMeasurements_,2*numberOfTracks_,-100);
    if(numberOfTracks_ > numberOfMeasurements_){
    	costMatrix.block(numberOfTracks_,numberOfMeasurements_,2*numberOfMeasurements_,2*numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(2*numberOfMeasurements_,1,0);
    }
    else{

    	costMatrix.block(numberOfTracks_,numberOfMeasurements_,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,-100);
    	costMatrix.block(numberOfTracks_+numberOfMeasurements_,numberOfMeasurements_,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,-100);

    	costMatrix.block(numberOfTracks_,numberOfMeasurements_,2*numberOfTracks_,2*numberOfTracks_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(2*numberOfTracks_,1,0);
    }

    // Fill the validated measurements (using mahalanobis distance)
    for(unsigned int t=0; t < numberOfTracks_; t++){
        for(unsigned int m=0; m < numberOfMeasurements_; m++){
        	costMatrix(t, m) = (int) (((double) rand() / (RAND_MAX)) * 200);
        }
    }

    // Print the cost matrix
    //std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;

    // StdCout the implications of this





	return true;
}

bool Hypothesis::solveCostMatrix(){
    //std::cout << std::endl << "Cost Matrix:" << std::endl  << costMatrix << std::endl;


    // TODO depend this on the number of measurements
    int k = 2;
    solutions = murty(-costMatrix,k);

    // TODO Filter the solution regarding several parameters using the leg tracker information
    // TODO Calculate the crossing value of the solutions in order to reject obvious unrealistic solutions

    std::cout << "Solutions are:" << std::endl;
    for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      //color_print_solution(costMatrix,solIt->assignmentMatrix);
      //std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;
    }



    return true;
}

bool Hypothesis::stdCoutSolutions(){

	Eigen::Matrix<double, -1, 1> childProbs = Eigen::Matrix<double, -1, 1>::Zero(solutions.size(),1);
	//double childProbs[solutions.size];

	size_t solCounter = 0;
    for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      color_print_solution(costMatrix,solIt->assignmentMatrix);
      std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;

      // Get the assignment matrix
      Eigen::Matrix<int, -1, -1> assignmentMatrix = solIt->assignmentMatrix;

      // Iterate the matrix
      size_t nCols = assignmentMatrix.cols();
      size_t nRows = assignmentMatrix.rows();

      //Parameters
      size_t N_det_F = 0;	// Number of detected free tracks
      size_t N_occ_F = 0;	// Number of occluded free tracks
      size_t N_del_F = 0;	// Number of deleted  free tracks

      size_t N_det_A = 0;	// Number of detected approved tracks
      size_t N_occ_A = 0;	// Number of occluded approved tracks
      size_t N_del_A = 0;	// Number of deleted  approved tracks

      size_t N_new = 0;		// New tracks
      size_t N_false = 0;   // Deleted tracks

      // Iterate through the top left rectangle
      for(size_t r=0; r < numberOfTracks_; r++){
          for(size_t c=0; c < numberOfMeasurements_; c++){

        	  // Skip if there is no assignment
        	  if(assignmentMatrix(r,c) == 0) continue;

        	  // Check if this value is on one of the four diagonals
        	  std::cout << "TRACK " << r << " --> DETECTED (MEAS " << c  << ")" << std::endl;
        	  N_det_F++;
          }
      }

      // Iterate the new Tracks
      for(size_t i=0; i < numberOfMeasurements_; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i+numberOfTracks_,i) == 0) continue;

    	  // Check if this value is on one of the four diagonals
    	  std::cout << "MEAS  " << i << " --> NEW_TRACK" << std::endl;
    	  N_new++;
      }

      // Iterate false alarms
      for(size_t i=0; i < numberOfMeasurements_; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i+numberOfMeasurements_+numberOfTracks_,i) == 0) continue;

    	  // Check if this value is on one of the four diagonals
    	  std::cout << "MEAS  " << i << " --> FALSE_ALARM" << std::endl;
    	  N_false++;
      }

      // Iterate occlusion
      for(size_t i=0; i < numberOfTracks_; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i,i+numberOfMeasurements_) == 0) continue;

    	  // Check if this value is on one of the four diagonals
    	  std::cout << "TRACK " << i << " --> OCCLUDED" << std::endl;
    	  N_occ_F++;
      }

      // Iterate deletion
      for(size_t i=0; i < numberOfTracks_; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i,i+numberOfMeasurements_+numberOfTracks_) == 0) continue;
    	  N_del_F++;

    	  // Check if this value is on one of the four diagonals
    	  std::cout << "TRACK " << i << " --> DELETED" << std::endl;
      }

      double childProb = pow(prob_detected_free,    N_det_F) *
    		  	  	  	 pow(prob_occluded_free,    N_occ_F) *
						 pow(prob_deleted_free,     N_del_F) *
						 pow(prob_detected_approved,N_det_A) *
						 pow(prob_occluded_approved,N_occ_A) *
						 pow(prob_deleted_approved, N_del_A) *
						 pow(prob_new, N_new) *
						 pow(prob_new, N_false);

      std::cout << RED << childProb << RESET << std::endl;
      childProbs[solCounter] = childProb;

      solCounter++;

    }

    childProbs.normalize();

    std::cout << "childProbs" << std::endl << childProbs << std::endl;

    return true;
}

bool Hypothesis::createChildren(){

	Eigen::Matrix<double, -1, 1> childProbs = Eigen::Matrix<double, -1, 1>::Zero(solutions.size(),1);
		//double childProbs[solutions.size];

	std::vector<std::vector<NewTrackAssignment> > trackAssignments;

	size_t solCounter = 0;
	for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){


	  // Get the assignment matrix
	  Eigen::Matrix<int, -1, -1> assignmentMatrix = solIt->assignmentMatrix;

	  // Storage for track Assignments of this solution
	  std::vector<NewTrackAssignment> solutionTrackAssignments;

	  // Iterate the matrix
	  size_t nCols = assignmentMatrix.cols();
	  size_t nRows = assignmentMatrix.rows();

	  //Parameters
	  size_t N_det_F = 0;	// Number of detected free tracks
	  size_t N_occ_F = 0;	// Number of occluded free tracks
	  size_t N_del_F = 0;	// Number of deleted  free tracks

	  size_t N_det_A = 0;	// Number of detected approved tracks
	  size_t N_occ_A = 0;	// Number of occluded approved tracks
	  size_t N_del_A = 0;	// Number of deleted  approved tracks

	  size_t N_new = 0;		// New tracks
	  size_t N_false = 0;   // Deleted tracks

	  // Iterate through the top left rectangle
	  for(size_t r=0; r < numberOfTracks_; r++){
		  for(size_t c=0; c < numberOfMeasurements_; c++){

			  // Skip if there is no assignment
			  if(assignmentMatrix(r,c) == 0) continue;

			  N_det_F++;
		  }
	  }

	  // Iterate the new Tracks
	  for(size_t i=0; i < numberOfMeasurements_; i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i+numberOfTracks_,i) == 0) continue;

		  N_new++;
		  NewTrackAssignment newTrackAssignment(detections_[i]->point_);
		  solutionTrackAssignments.push_back(newTrackAssignment);
	  }

	  // Iterate false alarms
	  for(size_t i=0; i < numberOfMeasurements_; i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i+numberOfMeasurements_+numberOfTracks_,i) == 0) continue;

		  N_false++;
	  }

	  // Iterate occlusion
	  for(size_t i=0; i < numberOfTracks_; i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i,i+numberOfMeasurements_) == 0) continue;

		  N_occ_F++;
	  }

	  // Iterate deletion
	  for(size_t i=0; i < numberOfTracks_; i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i,i+numberOfMeasurements_+numberOfTracks_) == 0) continue;
		  N_del_F++;
	  }



	  double childProb = pow(prob_detected_free,    N_det_F) *
						 pow(prob_occluded_free,    N_occ_F) *
						 pow(prob_deleted_free,     N_del_F) *
						 pow(prob_detected_approved,N_det_A) *
						 pow(prob_occluded_approved,N_occ_A) *
						 pow(prob_deleted_approved, N_del_A) *
						 pow(prob_new, N_new) *
						 pow(prob_new, N_false) *
						 this->getParentProbabilility();

	  std::cout << RED << childProb << RESET << std::endl;
	  childProbs[solCounter] = childProb;

	  trackAssignments.push_back(solutionTrackAssignments);

	  solCounter++;

	}

	childProbs.normalize();

	double maxProb = childProbs.maxCoeff();

	// Create the children
	for(size_t i = 0; i < solutions.size(); i++){

		// Ratio pruning
		if(childProbs[i]/maxProb > 0.1){

			// Create child
			HypothesisPtr childHypothesis(new Hypothesis(cycle_+1, this->globalHypothesisTree_));
			childHypothesis->setParent(shared_from_this());

			// Create the new Tracks
			std::vector<NewTrackAssignment> newTracks = trackAssignments[i];
			for(std::vector<NewTrackAssignment>::iterator assIt = newTracks.begin(); assIt != newTracks.end(); assIt++){
				std::cout << "    A New track for child " << i << " at " << std::endl << assIt->pos_ << std::endl;
				TrackPtr newTrack(new Track(assIt->pos_));

				childHypothesis->addTrack(newTrack);
			}

			ROS_ASSERT(this->globalHypothesisTree_); // ASSERT that this is set

			// Add this to the children
			this->children.push_back(childHypothesis);
			this->globalHypothesisTree_->addHypothesis(childHypothesis);

		}

	}


	//std::cout << "childProbs" << std::endl << childProbs << std::endl;



	return true;
}

void Hypothesis::setParent(HypothesisPtr parent){

	this->parent_ = parent;

	//std::cout << "I am Hypothesis for cycle " << this->cycle_ << " my parent was in cycle " << this->parent_->cycle_ << std::endl;
}

void Hypothesis::addTrack(TrackPtr track){
	std::cout << "New track added !" << std::endl;
	this->tracks_.push_back(track);
}
