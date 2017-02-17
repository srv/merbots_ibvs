// Copyright (c) 2017 Universitat de les Illes Balears

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <merbots_tracking/TargetPoints.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <mutex>

namespace merbots_ibvs {

class TargetPosePublisher {
 public:
  TargetPosePublisher() : nh_(), nhp_("~") {
    nh_.param("camera_angle", cam_angle_, 0.0);
    ROS_INFO("[Params] Camera angle regarding vertical axis: %f", cam_angle_);
    // Distance subscriber
    dist_sub_ = nhp_.subscribe("dist", 1, &TargetPosePublisher::dist_cb, this);
    // Region of Interest subscriber
    roi_sub_ = nhp_.subscribe("roi", 1, &TargetPosePublisher::roi_cb, this);
    // Get camera info
    ROS_INFO("Waiting for calibration information (10s) ...");
    boost::shared_ptr<sensor_msgs::CameraInfo const> sp;
    sp = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info",
                                                             nhp_,
                                                             ros::Duration(10));
    if (sp == NULL) {
      // No calibration received
      ROS_FATAL("No calibration received");
      ros::shutdown();
    } else {
      // Processing the received calibration
      sensor_msgs::CameraInfo cinfo = *sp;
      // Get the binning factors
      double bx = static_cast<double>(cinfo.binning_x);
      double by = static_cast<double>(cinfo.binning_y);
      if (bx < 1.0) bx = 1.0;
      if (by < 1.0) by = 1.0;
      fx_ =  static_cast<double>(cinfo.K[0]) / bx;
      fy_ =  static_cast<double>(cinfo.K[4]) / by;
      cx_ =  static_cast<double>(cinfo.K[2]) / bx;
      cy_ =  static_cast<double>(cinfo.K[5]) / by;
      frame_id_ = cinfo.header.frame_id;
    }
  }

 private:
  void setDist(const double& data) {
      std::lock_guard<std::mutex> lock(mutex_zdist);
      zdist_ = data;
  }

  double getDist() {
      std::lock_guard<std::mutex> lock(mutex_zdist);
      return zdist_;
  }

  void dist_cb(const sensor_msgs::RangeConstPtr& msg) {
    // Receiving new distance
    setDist(msg->range);
  }

  void roi_cb(const merbots_tracking::TargetPointsConstPtr& roi_msg) {
    cv::Point2i tl, tr, bl, br;
    {
      // Computing the feature points from the ROI
      std::lock_guard<std::mutex> lock(mutex_roi);
      tl.x = static_cast<int>(roi_msg->point_tl.x);
      tl.y = static_cast<int>(roi_msg->point_tl.y);
      tr.x = static_cast<int>(roi_msg->point_tr.x);
      tr.y = static_cast<int>(roi_msg->point_tr.y);
      bl.x = static_cast<int>(roi_msg->point_bl.x);
      bl.y = static_cast<int>(roi_msg->point_bl.y);
      br.x = static_cast<int>(roi_msg->point_br.x);
      br.y = static_cast<int>(roi_msg->point_br.y);
    }

    double H = getDist();
    if (H > 0.0) {
      double mid_x = (tl.x + tr.x + bl.x + br.x) / 4.0;
      double mid_y = (tl.y + tr.y + bl.y + br.y) / 4.0;

      // Pixel to 3D ray
      double ray_x = (mid_x - cx_) / fx_;
      double ray_y = (mid_y - cy_) / fy_;

      // Ax + By + Cz + D = 0
      double ground_plane[4] = { 0,
                                sin(cam_angle_),
                                cos(cam_angle_),
                                -H};

      double t = (-ground_plane[3]) /
                  (ground_plane[0] * ray_x +
                   ground_plane[1] * ray_y +
                   ground_plane[2]);
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(ray_x * t, ray_y * t, t));
      std::string frame_id = frame_id_;
      transform.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf::StampedTransform st(transform, ros::Time::now(), frame_id, "target");
      br_.sendTransform(st);

      std::cout << "TF: (" << ray_x * t
                   << ", " << ray_y * t
                   << ", " << t << ")" << std::endl;
    }
  }

  double fx_, fy_, cx_, cy_;
  double zdist_;
  double cam_angle_;
  std::string frame_id_;
  ros::NodeHandle nh_, nhp_;
  ros::Subscriber dist_sub_;
  ros::Subscriber roi_sub_;
  tf::TransformBroadcaster br_;

  std::mutex mutex_zdist;
  std::mutex mutex_roi;
};  //  class TargetPosePublisher
}   //  namespace merbots_ibvs

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_pose_publisher_node");

  merbots_ibvs::TargetPosePublisher tpp;
  ros::spin();
  return 0;
}






