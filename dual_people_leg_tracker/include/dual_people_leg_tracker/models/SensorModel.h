#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MODELS_SENSORMODEL_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MODELS_SENSORMODEL_H_

// ROS includes
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

// ROS Messages
#include <sensor_msgs/LaserScan.h>

#define DEBUG_SENSOR_MODEL


class SensorModel {

  public:
    /**
     * The used transformation listener
     */
    tf::TransformListener& tfl_;

  public:
    SensorModel(tf::TransformListener& tfl);
    virtual ~SensorModel();


    /***
     * Check if a given point is inside the scan area
     * @param point
     * @param scan
     * @return
     */
    bool isWithinScanArea(tf::Vector3 point, const sensor_msgs::LaserScan& scan, std::string fixedFrame);

};

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_SRC_MODELS_SENSORMODEL_H_ */
