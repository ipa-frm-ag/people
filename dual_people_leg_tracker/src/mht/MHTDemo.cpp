#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <dual_people_leg_tracker/mht/KalmanFilter.h>
#include <dual_people_leg_tracker/mht/Hypothesis.h>

#include <libs/gnuplot-iostream/gnuplot-iostream.h>


#include <unistd.h>


int main(int argc, char **argv)
{

	Gnuplot gp;

	gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
	// Data will be sent via a temporary file.  These are erased when you call
	// gp.clearTmpfiles() or when gp goes out of scope.  If you pass a filename
	// (e.g. "gp.file1d(pts, 'mydata.dat')"), then the named file will be created
	// and won't be deleted (this is useful when creating a script).


	// Time settings
	double dt = 0.08; // Timestep
	double duration = 1;

	ros::Time time(0);

	// Number of objects
	const int Nobjects = 6;
	int NMeasInit = 6;

	// Initialize the objects
	Eigen::Matrix<double,4,Nobjects> objects;
	objects = Eigen::Matrix<double,4,-1>::Zero(4,Nobjects);

	// X pos
	objects.row(0) << Eigen::VectorXd::LinSpaced(Eigen::Sequential,Nobjects,0,5);

	// X Vel
	objects.row(2) << 0, 0, 0;

	// Y Vel
	objects.row(3) <<  Eigen::VectorXd::Constant(Nobjects,1);

	std::cout << "objects" << std::endl << objects << std::endl;

	// System settings
	Eigen::Matrix<double,4,4> A_; // Transition Matrix

	Eigen::Matrix<double,4,4> Q_; // System Noise
	double posNoise = 0.2;
	double velNoise = 0.4;
	Q_ <<    posNoise, 0,        0,        0,
		       0,        posNoise, 0,        0,
			     0,        0,        velNoise, 0,
			     0,        0,        0,        velNoise;

  Eigen::Matrix<double,2,2> R_; // System Noise
  double measNoise = 0.01;
  R_ <<    measNoise, 0,
           0,         measNoise;

	A_ = Eigen::Matrix<double,-1,-1>::Identity(4,4);
	A_(0,2) = dt;
	A_(1,3) = dt;

	Eigen::Matrix<double,4,1> x; // Transition Matrix
	x << 0,0,0.1,1;

	Eigen::Matrix<double,2,1> z; // Measurement

	// Define the measurement Matrix
	Eigen::Matrix<double,2,4> H_; // Measurement
	H_ << 1, 0, 0, 0,
		  0, 1, 0,  0;

	z = H_*x;

	// Create detections Mat
	Eigen::Matrix<double,2, -1> detectionsMat;
	detectionsMat = Eigen::Matrix<double,2, -1>::Zero(2,NMeasInit+1);

	for(size_t i = 0; i < NMeasInit; i++){
		detectionsMat.col(i) = H_ * objects.col(i);
	}

	int cycle_ = 0;

	// Create Hypothesis
	HypothesisPtr rootHypothesis;
	rootHypothesis = HypothesisPtr(new Hypothesis(0));
	rootHypothesis->setParentProbability(1.0);
	rootHypothesis->setParentTime(time);
	rootHypothesis->assignMeasurements(cycle_, detectionsMat, time);
  rootHypothesis->coutCurrentSolutions(cycle_);

	std::vector<std::pair<double, double> > objectsHistory[Nobjects];



	for(double t=0; t<duration; t=t+dt){
		cycle_++;
		time = time + ros::Duration(dt);




		//std::cout << BOLDWHITE << time << " ----------------------------------------------------- " << cycle_ << RESET << std::endl;
		// Move
		for(int i = 0; i < Nobjects; i++){
      objectsHistory[i].push_back( std::pair<double,double>(objects(0,i), objects(1,i)));
			objects.col(i) = A_ * objects.col(i) + Q_*Eigen::Vector4d::Random();
		}

    for(int i = 0; i < Nobjects; i++){
      detectionsMat.col(i) = H_ * objects.col(i);
    }
    detectionsMat.col(Nobjects) = Eigen::Vector2d::Random()*5;

		//std::cout << "detectionsMat" << detectionsMat << std::endl;



		// Here the new children are generated
		rootHypothesis->assignMeasurements(cycle_, detectionsMat, time);

    //rootHypothesis->print();
		//rootHypothesis->coutCurrentSolutions(cycle_);

		//std::cout << "first call on get most likely" << std::endl;
		HypothesisPtr mostlikelyHypothesis = rootHypothesis->getMostLikelyHypothesis(cycle_+1);
		HypothesisPtr mostCumLikelyHypothesis = rootHypothesis->getMostLikelyHypothesisCumulative(cycle_+1);
		//std::cout << BOLDGREEN << "Most likely hypothesis for cycle " << cycle_ << " has id " << mostlikelyHypothesis->getId() << RESET << std::endl;
		//std::cout << BOLDGREEN << "Most cumulative likeli in cycle" << cycle_ << " is " << mostCumLikelyHypothesis->getId() << RESET << std::endl;

		HypothesisPtr newRoot;
		if(rootHypothesis->getNewRootByPruning(newRoot,cycle_,3)){
		  rootHypothesis = newRoot;
		}
		  //std::cout << BOLDRED << "Successful pruning!" << std::endl;
		//rootHypothesis->printTracks(cycle_+1);




		//std::cout << "objects" << std::endl << objects << std::endl;







//		std::cout << "Time       " << t << std::endl;
//		std::cout << "State      " << std::setw(3) << x.transpose() << std::endl;
//		std::cout << "Measurement" << std::setw(3) << z.transpose() << std::endl;
//		std::cout << "Kalman Prediction " << kalmanFilter.getPrediction().transpose() << std::endl;
//		std::cout << "Kalman Estimation " << kalmanFilter.getEstimation().transpose() << std::endl;
//
//		// Calculate the Mahalanobis distance
//		Eigen::Matrix<double,2,1> estimated_pos;
//		Eigen::Matrix<double,2,1> diff;
//		estimated_pos = kalmanFilter.getEstimation().block(0,0,2,1);
//
//		diff = estimated_pos - z;
//		double value = diff.transpose()*kalmanFilter.S_k.inverse()*diff;
//
//		std::cout << "Mahalanobis Distance " << sqrt(value) << std::endl;
//
//		std::cout << "Residual " << (kalmanFilter.getEstimation() - x).norm() << std::endl << "#########################" << std::endl;
//
//		x = A_*x;
//		z = H_*x;
//
//		kalmanFilter.predict(dt);
//		kalmanFilter.update(z);
	  gp << "set xrange [-4:12]\nset yrange [-0.1:1.4]\n";
	  for(size_t i = 0; i < Nobjects; i++){
	    if(i == 0)
	      gp << "plot";
	    else
	      gp << "replot";
	    gp << gp.file1d(objectsHistory[i]) << " with linespoints title 'obj'"<< std::endl;
	  }

	  std::vector<TrackPtr> allTracks;
	  rootHypothesis->getMostLikelyHypothesis(cycle_)->getTracks(allTracks, cycle_);

	  std::cout << allTracks.size() << std::endl;

	  for(size_t i = 0; i < allTracks.size(); i++){
	    std::vector<std::pair<double, double> > track;

	      std::cout << "Number of Tracks" << allTracks.size();

	      for(std::vector<Eigen::Vector4d>::iterator stateIt = allTracks[i]->estimated_states_.begin(); stateIt != allTracks[i]->estimated_states_.end(); stateIt++){
	        std::cout << (*stateIt).transpose() << std::endl;
	        track.push_back(std::pair<double,double>((*stateIt)[0], (*stateIt)[1]));
	      }
	      gp << "replot" << gp.file1d(track) << " with linespoints title 'est' pt 6 ps 2" << std::endl;
	      //

	  }

	  gp << "set label 'FA' at " << detectionsMat(0,Nobjects) << "," << detectionsMat(1,Nobjects) << std::endl;

		usleep(300000);

	}

	// Gnuplot positions
  ///////////////////////////////////////////////////////////////////////////



  //
  /////////////////////////////////////////////////////////////////////////



	//rootHypothesis->print();




//	gp << "replot" << gp.file1d(obj1Points_Est) << " with linepoints title 'obj1_Est' pt 6 ps 2" << std::endl;
//	gp << "replot" << gp.file1d(obj2Points_Est) << " with linepoints title 'obj2_Est' pt 6 ps 2" << std::endl;







}
