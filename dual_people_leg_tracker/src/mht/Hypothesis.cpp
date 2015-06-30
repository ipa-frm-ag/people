/*
 * Hypothesis.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: alex
 */

#include <dual_people_leg_tracker/mht/Hypothesis.h>
#include <dual_people_leg_tracker/mht/HypothesisTree.h>
#include <algorithm>

// Parameters (from Paper)
const long double prob_detected_free = 0.3;
const long double prob_occluded_free = 0.63;
const long double prob_deleted_free = 0.07;
const long double prob_detected_approved = 0.2;
const long double prob_occluded_approved = 0.79;
const long double prob_deleted_approved = 0.01;

const long double prob_new = 0.001;
const long double prob_fal = 0.003;


Hypothesis::Hypothesis(int cycle):
	newTrackCostValue_(150),
	falseAlarmCostValue_(100),
	deletionCostValue_(500),
	occlusionCostValue_(800),
	invalidValue_(-9000),
	probability_(1.0),
	cycle_(cycle),
	root_cycle_(0),
	dt_(0.0),
	id_(hypothesisIdCounter++),
	is_on_most_likely_branch_(false)
{
	//std::cout << "New Hypothesis create for cycle " << cycle << std::endl;
}

Hypothesis::~Hypothesis() {
	// TODO Auto-generated destructor stub

}

unsigned int Hypothesis::getNumberOfTracks(){
	return this->tracks_.size();
}

bool Hypothesis::assignMeasurements(int cycle, Eigen::Matrix<double,2,-1> detectionsMat, ros::Time time){

	// Reset all flags
	this->recursiveResetAllFlags();

	// Recursive call if this is not the right generation
	if(cycle > this->cycle_){
		//std::cout << "Measurements are passed to the " << this->children_.size() << " children" << std::endl;
		for(std::vector<HypothesisPtr>::iterator hypoIt = this->children_.begin(); hypoIt != this->children_.end(); hypoIt++){
			//std::cout << "Pass" << std::endl;
			(*hypoIt)->assignMeasurements(cycle, detectionsMat, time);
		}

		// Done
		return true;
	}
	//std::cout << "To a hypothesis in cycle " << BOLDRED << cycle_ << RESET <<  "  "  << detectionsMat.cols() << " new measurements are assigned" << std::endl;

	// Set the time
	this->time_ = time;

	// Calculate the time difference to the step, needed for prediction
	this->dt_ = (this->time_ - this->parent_time_).toSec();

	// Predict the tracks
	for(std::vector<TrackPtr>::iterator trackIt = this->tracks_.begin(); trackIt != this->tracks_.end(); trackIt++){
		//std::cout << "Doing prediction of track " << (*trackIt)->getId() << std::endl;
		(*trackIt)->predict(this->time_);
	}

	// Assign the detections
	this->detections_ = detectionsMat;

	// Calculate most important values
	numberOfMeasurements_ = detectionsMat.cols();

	// Create the cost matrix
	this->createCostMatrix();

	// Solve the cost matrix
	this->solveCostMatrix();

	// Print the solutions
	// this->stdCoutSolutions();
	// std::cout << "Starting to create children" << std::endl;
	this->createChildren();


	//rootHypothesis->createChildren();

	return true;
}

bool Hypothesis::createCostMatrix(){

    // Generate Matrix
    //std::cout << "There are currently " << nObjects << " objects and " << nMeasurements << " measurements(occlusion included)" << std::endl;

	unsigned int numberOfCols = getNumberOfTracks() 	  + 2 * numberOfMeasurements_;
	unsigned int numberOfRows = numberOfMeasurements_ + 2 * getNumberOfTracks();

	unsigned int numberOfTracks = getNumberOfTracks();

	unsigned int size;
	if(getNumberOfTracks() > numberOfMeasurements_){
		// Initialize
		size = numberOfMeasurements_ + 2 * numberOfTracks;
	    costMatrix = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Constant(size,size,this->invalidValue_);
	}else{
		// Initialize
		size = numberOfTracks 	  + 2 * numberOfMeasurements_;
	    costMatrix = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Constant(size,size,this->invalidValue_);
	}

    // Print the cost matrix
    //std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;




    // Fill the new tracks
    costMatrix.block(getNumberOfTracks(),0,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,this->newTrackCostValue_);

    //std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;

    // Fill the false alarms
    costMatrix.block(getNumberOfTracks()+numberOfMeasurements_,0,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,this->falseAlarmCostValue_);

    // Fill the Occlusion
    costMatrix.block(0,numberOfMeasurements_,numberOfTracks,getNumberOfTracks()).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks,1,this->occlusionCostValue_);

    // Fill the Deletions
    costMatrix.block(0,numberOfMeasurements_+getNumberOfTracks(),getNumberOfTracks(),getNumberOfTracks()).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(getNumberOfTracks(),1,this->deletionCostValue_);

    costMatrix.block(getNumberOfTracks(),numberOfMeasurements_,2*numberOfMeasurements_,2*getNumberOfTracks()) = Eigen::Matrix< int, -1, -1>::Constant(2*numberOfMeasurements_,2*numberOfTracks,-600);

    // If there are more Tracks
    if(getNumberOfTracks() > numberOfMeasurements_){
    	costMatrix.block(getNumberOfTracks(),numberOfMeasurements_,2*numberOfMeasurements_,2*numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(2*numberOfMeasurements_,1,-500);

    	costMatrix.block(getNumberOfTracks(),numberOfMeasurements_,2*numberOfMeasurements_,2*numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(2*numberOfMeasurements_,1,-500);

    	//costMatrix.block(numberOfTracks,numberOfTracks+numberOfMeasurements_,numberOfTracks,numberOfTracks).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks,1,-500);

    	costMatrix.block(numberOfTracks+numberOfMeasurements_*2-1,3*numberOfMeasurements_,1,2*numberOfTracks - 2*numberOfMeasurements_) = Eigen::Matrix< int, 1, -1>::Constant(1,2*numberOfTracks - 2*numberOfMeasurements_,-500);

    }

    // If there are more measurements
    else
    {
    	//costMatrix.block(numberOfTracks,numberOfTracks+numberOfMeasurements_,numberOfTracks,numberOfTracks).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfTracks,1,-555);

    	costMatrix.block(numberOfTracks,numberOfMeasurements_,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,-500);
    	costMatrix.block(numberOfTracks+numberOfMeasurements_,numberOfMeasurements_,numberOfMeasurements_,numberOfMeasurements_).diagonal() = Eigen::Matrix< int, -1, 1>::Constant(numberOfMeasurements_,1,-500);

    }

/*    if(numberOfMeasurements_ > getNumberOfTracks() && getNumberOfTracks()>0){
    	// Fixation
    	std::cout << "numberOfCols " << numberOfCols << std::endl;
    	std::cout << "numberOfMeasurements_ " << numberOfMeasurements_ << std::endl;
    	std::cout << "getNumberOfTracks() " << getNumberOfTracks() << std::endl;
    	std::cout << "start (" << numberOfTracks+numberOfMeasurements_*2-1 << " , " << numberOfMeasurements_*3 << ")" << std::endl;
    	costMatrix.block(numberOfTracks+numberOfMeasurements_*2-1,numberOfMeasurements_*3,1,size-3*numberOfMeasurements_) = Eigen::Matrix< int, -1, -1>::Constant(1,size-3*numberOfMeasurements_,-500);

    }*/

    // Fill the validated measurements (using mahalanobis distance)
    for(unsigned int t=0; t < numberOfTracks; t++){
        for(unsigned int m=0; m < numberOfMeasurements_; m++){

        	Eigen::Vector2d measPrediction;
        	measPrediction = this->tracks_[t]->getMeasurementPrediction();

        	this->tracks_[t]->getMeasurementLikelihood(this->detections_.col(m));

        	// Use the distance between measurement and measurement prediction
        	double dist = (measPrediction - this->detections_.col(m)).norm();

        	//costMatrix(t, m) = std::min(1000,((int) (50 / dist) * 200) / 100);
        	costMatrix(t, m) = (int) (this->tracks_[t]->getMeasurementLikelihood(this->detections_.col(m)) * 1000000);
        	//costMatrix(t, m) = (int) (sigmoid(dist,1,0.5)*1000);

        }
    }

    // Twerk the deletion
    for(unsigned int t=0; t < numberOfTracks; t++){
    	int costAddition = this->tracks_[t]->timeOccludedSeconds(this->time_)*99;
    	std::cout << BOLDWHITE << "Increasing the deletion costs " << costAddition << RESET << std::endl;
    	costMatrix(0+t,numberOfMeasurements_+getNumberOfTracks()+t) += costAddition;
    }

    // Print the cost matrix
    //if(numberOfTracks > 0)
    //std::cout << "The cost matrix is now " << std::endl << costMatrix << std::endl;

    // StdCout the implications of this





	return true;
}

bool Hypothesis::solveCostMatrix(){
    //std::cout << std::endl << "Cost Matrix:" << std::endl  << costMatrix << std::endl;

    // TODO depend this on the number of measurements
    int k_pre = 15;
    int k = 2;

    std::vector<Solution> pre;
    pre = murty(-costMatrix,k_pre);

    //pre = solutions;

    int last_cost = 0;
    for(size_t i = 0; i < k_pre; i++){
    	if(pre[i].cost_total != last_cost){
    		solutions.push_back(pre[i]);
    		last_cost = pre[i].cost_total;
    		if(solutions.size() == k) break;
    	}
    }

    // TODO Filter the solution regarding several parameters using the leg tracker information
    // TODO Calculate the crossing value of the solutions in order to reject obvious unrealistic solutions

    //std::cout << "Solutions are:" << std::endl;
    //for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      //color_print_solution(costMatrix,solIt->assignmentMatrix);
      //std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;
    //}

    return true;
}

bool Hypothesis::stdCoutSolutions(){

	unsigned int numberOfTracks = getNumberOfTracks();

	Eigen::Matrix<double, -1, 1> childProbs = Eigen::Matrix<double, -1, 1>::Zero(solutions.size(),1);
	//double childProbs[solutions.size];

	std::cout << "########################################################" << std::endl;

	size_t solCounter = 0;
    for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      std::cout << "Starting printing the solutions for HYP[" << getId() << "]" << " cycle " << this->cycle_ << std::endl;
      color_print_solution(costMatrix,solIt->assignmentMatrix);
      std::cout << "Costs "<< "\033[1m\033[31m" << -solIt->cost_total << "\033[0m" << std::endl;

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
      for(size_t r=0; r < numberOfTracks; r++){
          for(size_t c=0; c < numberOfMeasurements_; c++){

        	  // Skip if there is no assignment
        	  if(assignmentMatrix(r,c) == 0) continue;

        	  // Check if this value is on one of the four diagonals
        	  // std::cout << "TRACK " << r << " --> DETECTED (MEAS " << c  << ")" << std::endl;
        	  N_det_F++;
          }
      }

      // Iterate the new Tracks
      for(size_t i=0; i < numberOfMeasurements_; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i+numberOfTracks,i) == 0) continue;

    	  // Check if this value is on one of the four diagonals
    	  // std::cout << "MEAS  " << i << " --> NEW_TRACK" << std::endl;
    	  N_new++;
      }

      // Iterate false alarms
      for(size_t i=0; i < numberOfMeasurements_; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i+numberOfMeasurements_+numberOfTracks,i) == 0) continue;

    	  // Check if this value is on one of the four diagonals
    	  // std::cout << "MEAS  " << i << " --> FALSE_ALARM" << std::endl;
    	  N_false++;
      }

      // Iterate occlusion
      for(size_t i=0; i < numberOfTracks; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i,i+numberOfMeasurements_) == 0) continue;

    	  // Check if this value is on one of the four diagonals
    	  // std::cout << "TRACK " << i << " --> OCCLUDED" << std::endl;
    	  N_occ_F++;
      }

      // Iterate deletion
      for(size_t i=0; i < numberOfTracks; i++){

    	  // Skip if there is no assignment
    	  if(assignmentMatrix(i,i+numberOfMeasurements_+numberOfTracks) == 0) continue;
    	  N_del_F++;

    	  // Check if this value is on one of the four diagonals
    	  // std::cout << "TRACK " << i << " --> DELETED" << std::endl;
      }

      std::cout << "N_new = " << N_new << std::endl;

      double childProb = pow(prob_detected_free,    N_det_F) *
    		  	  	  	 pow(prob_occluded_free,    N_occ_F) *
						 pow(prob_deleted_free,     N_del_F) *
						 pow(prob_detected_approved,N_det_A) *
						 pow(prob_occluded_approved,N_occ_A) *
						 pow(prob_deleted_approved, N_del_A) *
						 pow(prob_new, N_new) *
						 pow(prob_new, N_false);

      //std::cout << RED << childProb << RESET << std::endl;
      childProbs[solCounter] = childProb;

      solCounter++;

    }

    childProbs.normalize();

    //std::cout << "childProbs" << std::endl << childProbs << std::endl;

    return true;
}

bool Hypothesis::createChildren(){

	ROS_ASSERT(solutions.size() > 0);

	Eigen::Matrix<double, -1, 1> childProbs = Eigen::Matrix<double, -1, 1>::Zero(solutions.size(),1);
		//double childProbs[solutions.size];

	// Holds the assignments
    std::vector<std::vector<TrackAssignment> > all_assignments;


	// Iterate the solutions
	size_t solCounter = 0;
	for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
	  //std::cout << "-------Solution--------" << solCounter << std::endl;

	  // Get the assignment matrix
	  Eigen::Matrix<int, -1, -1> assignmentMatrix = solIt->assignmentMatrix;

	  // Storage for track Assignments of this solution
	  std::vector<TrackAssignment> assignments;

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

	  // Detected tracks
	  for(size_t r=0; r < getNumberOfTracks(); r++){
		  for(size_t c=0; c < numberOfMeasurements_; c++){

			  // Skip if there is no assignment
			  if(assignmentMatrix(r,c) == 0) continue;

			  TrackAssignment assignment;
			  assignment.setDetection(this->tracks_[r],c);
			  assignments.push_back(assignment);
			  N_det_F++;
		  }
	  }

	  // Iterate the new Tracks
	  for(size_t i=0; i < numberOfMeasurements_; i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i+getNumberOfTracks(),i) == 0) continue;

		  N_new++;
		  TrackAssignment assignment;
		  assignment.setNew(i);
		  assignments.push_back(assignment);
		  //std::cout << "New Track!!!" << std::endl;
	  }

	  // Iterate false alarms
	  for(size_t i=0; i < numberOfMeasurements_; i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i+numberOfMeasurements_+getNumberOfTracks(),i) == 0) continue;

		  N_false++;

		  TrackAssignment assignment;
		  assignment.isFalseAlarm();
		  assignments.push_back(assignment);
	  }

	  // Iterate occlusion
	  for(size_t i=0; i < getNumberOfTracks(); i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i,i+numberOfMeasurements_) == 0) continue;

		  N_occ_F++;
		  TrackAssignment assignment;
		  assignment.setOcclusion(this->tracks_[i]);
		  assignments.push_back(assignment);
	  }

	  // Iterate deletion
	  for(size_t i=0; i < getNumberOfTracks(); i++){

		  // Skip if there is no assignment
		  if(assignmentMatrix(i,i+numberOfMeasurements_+getNumberOfTracks()) == 0) continue;
		  N_del_F++;
		  TrackAssignment assignment;
		  assignment.setDeletion(this->tracks_[i]);
		  assignments.push_back(assignment);
	  }
//      std::cout << "N_det_F = " << N_det_F << std::endl;
//      std::cout << "N_occ_F = " << N_occ_F << std::endl;
//      std::cout << "N_del_F = " << N_del_F << std::endl;
//      std::cout << "N_det_A = " << N_det_A << std::endl;
//      std::cout << "N_occ_A = " << N_occ_A << std::endl;
//      std::cout << "N_del_A = " << N_del_A << std::endl;
//      std::cout << "N_new = " << N_new << std::endl;
//      std::cout << "N_false = " << N_false << std::endl;

      //std::cout << "prob" << pow(prob_new, N_new) << std::endl;
	  // Calulate the
	  double childProb_part1 = 1; // Needs to be calculated on updated trackers
	  double childProb_part2 = pow(prob_detected_free,    N_det_F) *
						 	   pow(prob_occluded_free,    N_occ_F) *
						       pow(prob_deleted_free,     N_del_F) *
						       pow(prob_detected_approved,N_det_A) *
						       pow(prob_occluded_approved,N_occ_A) *
						       pow(prob_deleted_approved, N_del_A) *
						       pow(prob_new, N_new) *
						       pow(prob_fal, N_false) *
						       this->getParentProbabilility();

	  // std::cout << "childProb_part2 " << childProb_part2 << std::endl;

	  //std::cout << RED << childProb << RESET << std::endl;




      // Print the assignments
      //std::cout << std::endl << "Assignments:" << std::endl;
      for(std::vector<TrackAssignment>::iterator assIt = assignments.begin(); assIt != assignments.end(); assIt++){
        assIt->print();
      }
      //std::cout << std::endl << "Assignments done" << std::endl;

      // Create child
      HypothesisPtr childHypothesis(new Hypothesis(cycle_+1));
      childHypothesis->setParentProbability(this->probability_);
      childHypothesis->setRootCycle(this->getRootCycle());
      childHypothesis->setParentTime(this->time_);

      for(std::vector<TrackAssignment>::iterator assIt = assignments.begin(); assIt != assignments.end(); assIt++){

        // New track
        if(assIt->isNew())
        {
          TrackPtr newTrack(new Track(this->detections_.col(assIt->meas_), this->time_));
          childHypothesis->addTrack(newTrack);
          //std::cout << "    A New track for child " << i << " at " << std::endl << this->detections_.col(assIt->meas_) << std::endl;

          tracksCreatedCounter++;
        }

        // Detection
        if(assIt->isDetection())
        {
          TrackPtr copyTrack(new Track( *(assIt->getTrack()) ));
          childHypothesis->addTrack(copyTrack);
          //std::cout << assIt->getTrack()->getId() << " =>copy  " << copyTrack->getId() << std::endl;

          //std::cout << "Update of track " << copyTrack->getId() << std::endl;
          //std::cout << assIt->getTrack()->getMeasurementPrediction() << std::endl;
          //std::cout << copyTrack->getMeasurementPrediction() << std::endl;
          copyTrack->update(this->detections_.col(assIt->meas_), this->time_);

          double measurementLikelihood = copyTrack->getMeasurementLikelihood(this->detections_.col(assIt->meas_));
          //std::cout << "The likelihood of the track " << copyTrack->getId() << " is " << measurementLikelihood << std::endl;

          childProb_part1 = childProb_part1*measurementLikelihood;
          // TODO, apply measurement !!! After!! making the copy!
        }


        // OCCLUDED
        if(assIt->isOcclusion())
        {
          TrackPtr copyTrack(new Track( *(assIt->getTrack()) ));
          childHypothesis->addTrack(copyTrack);
          copyTrack->setOccluded(this->time_);
          //std::cout << assIt->getTrack()->getId() << " =>copy  " << copyTrack->getId() << std::endl;

          //std::cout << "Update of track " << copyTrack->getId() << std::endl;
          //std::cout << assIt->getTrack()->getMeasurementPrediction() << std::endl;
          //std::cout << copyTrack->getMeasurementPrediction() << std::endl;
          //copyTrack->update(this->detections_.col(assIt->meas_), this->time_);
        }


        // FALSEALARM
        if(assIt->isFalseAlarm())
        {
          // TODO
        }


        // DELETED
        if(assIt->isDeletion())
        {
          // TODO
        }



      }


	  // Calculate the child Hypothesis
      //std::cout << "Probability of DetectedTracks:" << childProb_part1 << "  probability of other stuff " << childProb_part2 << std::endl;
	  childProbs[solCounter] = childProb_part1 * childProb_part2;
      this->children_.push_back(childHypothesis);
      //this->globalHypothesisTree_->addHypothesis(childHypothesis);




	  solCounter++;

	}

	double probSum = childProbs.sum();

	// Normalize the children probabilities
	for(size_t i = 0; i < childProbs.rows(); i++){
		long double childProb = (double) childProbs[i]/probSum;
	  //std::cout << "setting prob of Hypothesis" << this->children_[i]->getId() << " to " << childProb << std::endl;
	  this->children_[i]->setProbability(childProb);
	}


	// Do a ratio pruning

	//std::cout << childProbs << std::endl;

	double maxProb = childProbs.maxCoeff();

	return true;
}

void Hypothesis::setParentProbability(double probability){

	this->parent_probability_ = probability;

	//std::cout << "I am Hypothesis for cycle " << this->cycle_ << " my parent was in cycle " << this->parent_->cycle_ << std::endl;
}

void Hypothesis::addTrack(TrackPtr track){
	this->tracks_.push_back(track);
	//std::cout << "HPY[" << getId() << "] adding track" << track->getId() << "now has " << getNumberOfTracks() << " tracks" << std::endl;
}

void Hypothesis::setRootCycle(int rootCycle){
	for(std::vector<HypothesisPtr>::iterator hypoIt = this->children_.begin(); hypoIt != this->children_.end(); hypoIt++){
		(*hypoIt)->setRootCycle(rootCycle);
	}
}

// Print yourself
void Hypothesis::print(){
	int nTabs = this->cycle_ - getRootCycle();
	for(int i=0; i < nTabs; i++){
		std::cout << "  ";
	}
	if(is_on_most_likely_branch_) std::cout << YELLOW;
	std::cout << "->HYP[ID:" << getId() << " prob:" << RED << this->probability_ << RESET << " cyc: " << getCycle() << " nCh: " << this->children_.size() << " cumLi:" << getCumulativeLikelihood() << " nT: " << getNumberOfTracks() << " dt:" << this->dt_ << "]" << std::endl;
	if(is_on_most_likely_branch_) std::cout << RESET;
	for(std::vector<HypothesisPtr>::iterator hypoIt = this->children_.begin(); hypoIt != this->children_.end(); hypoIt++){
		(*hypoIt)->print();
	}
}

// Print yourself
void Hypothesis::printTracks(){
	std::cout << "Tracks of HYP[ID:" << getId() << " prob:" << RED << this->probability_ << RESET << " cyc: " << getCycle() << " nCh: " << this->children_.size() << " nT: " << getNumberOfTracks() << " dt:" << this->dt_ << "]" << std::endl;

	for(std::vector<TrackPtr>::iterator trackIt = this->tracks_.begin(); trackIt != this->tracks_.end(); trackIt++){
		std::cout << "Track:" << (*trackIt)->getId() << std::endl;
		(*trackIt)->print();
	}
}

void Hypothesis::printTracks(int cycle){
	if(this->cycle_ ==  cycle)
		this->printTracks();
	else
		for(std::vector<HypothesisPtr>::iterator hypoIt = this->children_.begin(); hypoIt != this->children_.end(); hypoIt++){
			(*hypoIt)->printTracks(cycle);
		}
}

bool Hypothesis::getTracks(std::vector<TrackPtr> &allTracks,int cycle){
	if(this->cycle_ ==  cycle){
		allTracks.reserve( allTracks.size() + this->tracks_.size() );
		allTracks.insert(allTracks.end(), this->tracks_.begin(), this->tracks_.end());
		return true;
	}
	else
		for(std::vector<HypothesisPtr>::iterator hypoIt = this->children_.begin(); hypoIt != this->children_.end(); hypoIt++){
			(*hypoIt)->getTracks(allTracks,cycle);
		}
}

void Hypothesis::coutCurrentSolutions(int cycle){
	if(this->cycle_ ==  cycle)
		this->stdCoutSolutions();
	else
		for(std::vector<HypothesisPtr>::iterator hypoIt = this->children_.begin(); hypoIt != this->children_.end(); hypoIt++){
			(*hypoIt)->coutCurrentSolutions(cycle);
		}
}

void Hypothesis::setParentTime(ros::Time time){
	this->parent_time_ = time;
}

void Hypothesis::setProbability(double probabilty){
	this->probability_ = probabilty;
}

HypothesisPtr Hypothesis::getMostLikelyHypothesis(int cycle){

	// Iterate the children
	HypothesisPtr mostlikelyHypothesis;
	long double maxProb = 0;

	//If the next cycle is the requested cycle
	if(cycle-1 > this->cycle_){
		HypothesisPtr tempHypothesis;

		for(size_t i = 0; i<this->children_.size(); i++){
			tempHypothesis = this->children_[i]->getMostLikelyHypothesis(cycle);
			long double childProb = tempHypothesis->getProbability();
			if(childProb > maxProb){
				maxProb = childProb;
				mostlikelyHypothesis = tempHypothesis;
			}
		}
	}

	// If your on top ask your children to give you your most likely hypothesis
	else
	{
		for(size_t i = 0; i<this->children_.size(); i++){
			long double childProb = this->children_[i]->getProbability();
			if(childProb > maxProb){
				maxProb = childProb;
				mostlikelyHypothesis = this->children_[i];
			}
		}
	}

	return mostlikelyHypothesis;
}

HypothesisPtr Hypothesis::getMostLikelyHypothesisCumulative(int cycle){

	// Iterate the children
	HypothesisPtr mostlikelyHypothesis;
	long double maxProb = 0;

	//If the next cycle is the requested cycle
	if(cycle-1 > this->cycle_){
		HypothesisPtr tempHypothesis;

		for(size_t i = 0; i<this->children_.size(); i++){
			tempHypothesis = this->children_[i]->getMostLikelyHypothesis(cycle);
			long double childProb = tempHypothesis->getCumulativeLikelihood();
			if(childProb > maxProb){
				maxProb = childProb;
				mostlikelyHypothesis = tempHypothesis;
			}
		}
	}

	// If your on top ask your children to give you your most likely hypothesis
	else
	{
		std::cout << "  > Iterating the " << this->children_.size() << " children of Hyp[" << this->getId() << "]"  << std::endl;
		for(size_t i = 0; i<this->children_.size(); i++){
			std::cout << "iterating child" << std::endl;
			long double childProb = this->children_[i]->getCumulativeLikelihood();
			if(childProb > maxProb){
				maxProb = childProb;
				mostlikelyHypothesis = this->children_[i];
			}
		}
	}

	if(this->cycle_ == this->root_cycle_){
		recursiveRaiseFlagOnCumulativePath(mostlikelyHypothesis->getId());
	}

	return mostlikelyHypothesis;
}

long double Hypothesis::getCumulativeLikelihood(){
	return getParentProbabilility() * getProbability();
}

bool Hypothesis::hasChildWithID(int id){
	if(getId() == id) return true;

	for(size_t i = 0; i<this->children_.size(); i++){
		if(this->children_[i]->hasChildWithID(id)) return true;
	}

	return false;
}

void Hypothesis::recursiveRaiseFlagOnCumulativePath(int id){

	for(size_t i = 0; i<this->children_.size(); i++){
		if(this->children_[i]->hasChildWithID(id)){
			this->is_on_most_likely_branch_ = true;
			this->children_[i]->recursiveRaiseFlagOnCumulativePath(id);
		}

	}
}

bool Hypothesis::recursiveResetAllFlags(){
	this->is_on_most_likely_branch_ = false;

	for(size_t i = 0; i<this->children_.size(); i++){
		this->children_[i]->recursiveResetAllFlags();
	}

}
