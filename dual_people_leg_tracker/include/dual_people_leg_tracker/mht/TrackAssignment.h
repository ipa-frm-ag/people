/*
 * Assignment.h
 *
 *  Created on: Jun 28, 2015
 *      Author: alex
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_TRACK_ASSIGNMENT_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_TRACK_ASSIGNMENT_H_

#include <dual_people_leg_tracker/mht/Track.h>

#include <iostream>



class TrackAssignment {
public:



	int meas_;
private:

	TrackPtr track_;

	bool is_new_;
	bool is_detection_;
	bool is_deletion_;
	bool is_falsealarm_;
	bool is_occlusion_;

  size_t N_det_F; // Number of detected free tracks
  size_t N_occ_F; // Number of occluded free tracks
  size_t N_del_F; // Number of deleted  free tracks

  size_t N_det_A; // Number of detected approved tracks
  size_t N_occ_A; // Number of occluded approved tracks
  size_t N_del_A; // Number of deleted  approved tracks

  size_t N_new;   // New tracks
  size_t N_false;   // Deleted tracks

public:
	TrackAssignment();

	virtual ~TrackAssignment();
	//friend std::ostream& operator<<(std::ostream& os, const TrackAssignment& dt);

	TrackPtr getTrack() const;

	void print();

	void setNew(int measIdx){
		this->meas_ = measIdx;
		this->is_new_ = true;

		N_new++;
	}
	void setDetection(TrackPtr track, int measIdx){
		this->track_ = track;
		this->meas_ = measIdx;
		this->is_detection_ = true;
	}
	void setDeletion(TrackPtr track){
		this->track_ = track;
		this->is_deletion_ = true;
	}
	void setFalseAlarm(int measIdx){
		this->meas_ = measIdx;
		this->is_falsealarm_ = true;
	}
	void setOcclusion(TrackPtr track){
		this->track_ = track;
		this->is_occlusion_ = true;
	}

	bool isNew() const { return this->is_new_; }
	bool isDetection() const { return this->is_detection_; };
	bool isDeletion() const { return this->is_deletion_; };
	bool isFalseAlarm() const { return this->is_falsealarm_; };
	bool isOcclusion() const { return this->is_occlusion_; };
};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_MHT_TRACK_ASSIGNMENT_H_ */
