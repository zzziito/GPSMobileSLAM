#ifndef ORBSLAM2_ROS_MONONODE_H_
#define ORBSLAM2_ROS_MONONODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "System.h"
#include "Node.h"


class MonoNode : public Node
{
  public:
    MonoNode (const GPS_OFF_SLAM::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~MonoNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msg);
    void GpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& msgG);

  private:
    image_transport::Subscriber image_subscriber;
    message_filters::Subscriber<sensor_msgs::NavSatFix> *gps_subscriber_;
    double latitude;
    double longitude;
};

#endif //ORBSLAM2_ROS_MONONODE_H_
