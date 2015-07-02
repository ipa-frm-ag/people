/*
 * SensorModel.cpp
 *
 *  Created on: Jul 1, 2015
 *      Author: frm-ag
 */
#include <ros/ros.h>
#include <dual_people_leg_tracker/models/SensorModel.h>
#include <ros/console.h>

SensorModel::SensorModel(tf::TransformListener& tfl):
tfl_(tfl)
{

}

SensorModel::~SensorModel() {

}

bool SensorModel::isWithinScanArea(tf::Vector3 pos, const sensor_msgs::LaserScan& scan, std::string fixed_frame){
  //ROS_DEBUG_COND(DEBUG_SENSOR_MODEL,"SensorModel::%s",__func__);
  tf::Stamped<tf::Point> point(pos, scan.header.stamp, fixed_frame);

  // Transform the point into the coordinate system of the laserscan
  try
  {
    tfl_.transformPoint(scan.header.frame_id, point, point); //Transform using odometry information into the fixed frame
    //ROS_DEBUG_COND(DEBUG_SENSOR_MODEL,"SensorModel::%s - Transforming point from %s to %s",__func__, point.frame_id_.c_str(), scan.header.frame_id.c_str());
  }
  catch (...)
  {
    ROS_WARN("TF exception spot 3.");
  }

  double angle  = atan2(point[1],point[0]);
  double r      = point.length();

  if(angle > scan.angle_min && angle < scan.angle_max && r > 0){
    return true;
  }

  // This is returned if the particle is not within range
  return false;
}
