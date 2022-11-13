#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (GPS_OFF_SLAM::System::MONOCULAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoNode::MonoNode (GPS_OFF_SLAM::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 100, &MonoNode::ImageCallback, this);
  gps_subscriber_ = new message_filters::Subscriber<sensor_msgs::NavSatFix> (node_handle, "/gps_data", 100); 
  camera_info_topic_ = "/videofile/camera_info";
  gps_subscriber_->registerCallback(boost::bind(&MonoNode::GpsCallBack, this, _1));
}


MonoNode::~MonoNode () {
    delete gps_subscriber_;
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  gps_off_slam->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec(),latitude,longitude);

  Update ();
}

void MonoNode::GpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& msgG)
{
  latitude = static_cast<double>(msgG->latitude);
  longitude = static_cast<double>(msgG->longitude);
}
