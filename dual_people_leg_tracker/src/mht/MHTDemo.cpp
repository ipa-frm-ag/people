#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <dual_people_leg_tracker/mht/KalmanFilter.h>
#include <dual_people_leg_tracker/mht/Hypothesis.h>

#include <libs/gnuplot-iostream/gnuplot-iostream.h>

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
	double duration = 0.40;

	ros::Time time(0);

	// Number of objects
	const int N = 3;
	int NMeasInit = 3;

	// Initialize the objects
	Eigen::Matrix<double,4,N> objects;
	objects = Eigen::Matrix<double,4,-1>::Zero(4,N);

	// X pos
	objects.row(0) << -1, 4, 6;

	// X Vel
	objects.row(2) << 0, 0, 0;

	// Y Vel
	objects.row(3) << 2, 0.25, 0.25;

	std::cout << "objects" << std::endl << objects << std::endl;

	// System settings
	Eigen::Matrix<double,4,4> A_; // Transition Matrix

	Eigen::Matrix<double,4,4> Q_; // System Noise
	double posNoise = 0.01;
	double velNoise = 0.02;
	Q_ <<    posNoise, 0,        0,        0,
		     0,        posNoise, 0,        0,
			 0,        0,        velNoise, 0,
			 0,        0,        0,        velNoise;

  Eigen::Matrix<double,2,2> R_; // System Noise
  double measNoise = 0.01;
  R_ <<    measNoise, 0,
         0,        measNoise;

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
	detectionsMat = Eigen::Matrix<double,2, -1>::Zero(2,NMeasInit);

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


	// Plotting helper
	std::vector<std::pair<double, double> > obj0Points;
	std::vector<std::pair<double, double> > obj1Points;
	std::vector<std::pair<double, double> > obj2Points;
	// Set initial values
	obj0Points.push_back(std::make_pair(objects(0,0), objects(1,0)));
	obj1Points.push_back(std::make_pair(objects(0,1), objects(1,1)));
	obj2Points.push_back(std::make_pair(objects(0,2), objects(1,2)));

	std::vector<std::pair<double, double> > obj0Points_Est;
	std::vector<std::pair<double, double> > obj1Points_Est;
	std::vector<std::pair<double, double> > obj2Points_Est;


	for(double t=0; t<duration; t=t+dt){
		cycle_++;
		time = time + ros::Duration(dt);




		std::cout << BOLDWHITE << time << " ----------------------------------------------------- " << cycle_ << RESET << std::endl;
		// Move
		for(int i = 0; i < N; i++){
			objects.col(i) = A_ * objects.col(i) + Q_*Eigen::Vector4d::Random();
			objects(0,0) += objects(0,0) * sin(t*8)*0.1;
		}

		if(t==0.16){
		  detectionsMat.resize(2,2);
		  detectionsMat.col(0) = H_ * objects.col(0);
		  detectionsMat.col(1) = H_ * objects.col(1);
		}else{
	    // Measure
	    for(size_t i = 0; i < NMeasInit; i++){
	      detectionsMat.resize(2,NMeasInit+1);
	      detectionsMat.col(i) = H_ * objects.col(i) + R_*Eigen::Vector2d::Random();
	    }

	    detectionsMat.col(NMeasInit) = Eigen::Vector2d::Random();
		}



		// Here the new children are generated
		rootHypothesis->assignMeasurements(cycle_, detectionsMat, time);

    rootHypothesis->print();
		rootHypothesis->coutCurrentSolutions(cycle_);

		//std::cout << "first call on get most likely" << std::endl;
		HypothesisPtr mostlikelyHypothesis = rootHypothesis->getMostLikelyHypothesis(cycle_+1);
		HypothesisPtr mostCumLikelyHypothesis = rootHypothesis->getMostLikelyHypothesisCumulative(cycle_+1);
		//std::cout << BOLDGREEN << "Most likely hypothesis for cycle " << cycle_ << " has id " << mostlikelyHypothesis->getId() << RESET << std::endl;
		//std::cout << BOLDGREEN << "Most cumulative likeli in cycle" << cycle_ << " is " << mostCumLikelyHypothesis->getId() << RESET << std::endl;

		HypothesisPtr newRoot;
		if(rootHypothesis->getNewRootByPruning(newRoot,cycle_,2)){
		  rootHypothesis = newRoot;
		}
		  std::cout << BOLDRED << "Successful pruning!" << std::endl;
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

		///////////////////////////////////////////////////////////////////////////
		// GNUPLOT
		obj0Points.push_back(std::make_pair(objects(0,0), objects(1,0)));
		obj1Points.push_back(std::make_pair(objects(0,1), objects(1,1)));
		obj2Points.push_back(std::make_pair(objects(0,2), objects(1,2)));

		gp << "set xrange [-4:12]\nset yrange [-0.1:0.8]\n";
		gp << "plot" << gp.file1d(obj0Points) << " with linespoints title 'obj0',"
				     << gp.file1d(obj1Points) << " with linespoints title 'obj1',"
					 << gp.file1d(obj2Points) << " with linespoints title 'obj2'"<< std::endl;

		//
		/////////////////////////////////////////////////////////////////////////

	}

	rootHypothesis->print();

	std::vector<TrackPtr> allTracks;
	rootHypothesis->getTracks(allTracks, cycle_+1);

	for(size_t i = 0; i < allTracks.size(); i++){
		switch(i){
			case 0:
	    		for(std::vector<Eigen::Vector4d>::iterator stateIt = allTracks[i]->estimated_states_.begin(); stateIt != allTracks[i]->estimated_states_.end(); stateIt++){
	    			std::cout << (*stateIt).transpose() << std::endl;
	    			obj0Points_Est.push_back(std::make_pair((*stateIt)[0], (*stateIt)[1]));
	    		}
	    		break;
			case 1:
	    		for(std::vector<Eigen::Vector4d>::iterator stateIt = allTracks[i]->estimated_states_.begin(); stateIt != allTracks[i]->estimated_states_.end(); stateIt++){
	    			std::cout << (*stateIt).transpose() << std::endl;
	    			obj1Points_Est.push_back(std::make_pair((*stateIt)[0], (*stateIt)[1]));
	    		}
	    		break;
			case 2:
	    		for(std::vector<Eigen::Vector4d>::iterator stateIt = allTracks[i]->estimated_states_.begin(); stateIt != allTracks[i]->estimated_states_.end(); stateIt++){
	    			std::cout << (*stateIt).transpose() << std::endl;
	    			obj2Points_Est.push_back(std::make_pair((*stateIt)[0], (*stateIt)[1]));
	    		}
	    		break;

		}

	}

	gp << "replot" << gp.file1d(obj0Points_Est) << " with points title 'obj0_Est' pt 6 ps 1" << std::endl;
	gp << "replot" << gp.file1d(obj1Points_Est) << " with points title 'obj1_Est' pt 6 ps 1" << std::endl;
	gp << "replot" << gp.file1d(obj2Points_Est) << " with points title 'obj2_Est' pt 6 ps 1" << std::endl;







}
