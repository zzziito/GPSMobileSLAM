#ifndef GPS_OFFROAD_SLAM_ROS_NODE_H_
#define GPS_OFFROAD_SLAM_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <gps_offroad_slam/dynamic_reconfigureConfig.h>

#include "gps_offroad_slam/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

#include "System.h"



class Node
{
  public:
    Node (GPS_OFF_SLAM::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();
    void Init ();

  protected:
    void Update ();
    GPS_OFF_SLAM::System* gps_off_slam;
    ros::Time current_frame_time_;

    std::string camera_info_topic_;

  private:
    void PublishMapPoints (std::vector<GPS_OFF_SLAM::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishGBAStatus (bool gba_status);
    void PublishRenderedImage (cv::Mat image);
    void ParamsChangedCallback(gps_offroad_slam::dynamic_reconfigureConfig &config, uint32_t level);
    bool SaveMapSrv (gps_offroad_slam::SaveMap::Request &req, gps_offroad_slam::SaveMap::Response &res);
    void LoadOrbParameters (GPS_OFF_SLAM::ORBParameters& parameters);

    // initialization Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    tf2::Transform TransformFromMat (cv::Mat position_mat);
    tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<GPS_OFF_SLAM::MapPoint*> map_points);

    dynamic_reconfigure::Server<gps_offroad_slam::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher status_gba_publisher_;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    GPS_OFF_SLAM::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string gps_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};

#endif //GPS_OFFROAD_SLAM_ROS_NODE_H_
