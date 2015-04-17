/*
 * leg_detector.h
 *
 *  Created on: Mar 19, 2015
 *      Author: frm-ag
 */

#ifndef DUAL_TRACKER_H_
#define DUAL_TRACKER_H_

#include <ros/ros.h>

// Own includes
#include <leg_detector/DualTrackerConfig.h>
#include <leg_detector/laser_processor.h>

// OpenCV includes
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

// Messages
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// People tracking
#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>

// Configuration
#include <dynamic_reconfigure/server.h>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

static double no_observation_timeout_s = 0.0;
static double max_second_leg_age_s     = 2.0;
static double max_track_jump_m         = 1.0; //Maximale jump distance for a track
static double max_meas_jump_m          = 0.75; // 1.0
static double leg_pair_separation_m    = 1.0;

#endif /* DUAL_TRACKER_H_ */
