#ifndef IBVS_H
#define IBVS_H

#include <boost/thread.hpp>

#include <auv_msgs/BodyVelocityReq.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <merbots_ibvs/IBVSConfig.h>
#include <merbots_ibvs/IBVSInfo.h>
#include <merbots_tracking/TargetPoints.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <merbots_ibvs/pid/PID.h>

namespace merbots_ibvs
{
  class IBVS
  {
  public:
    IBVS(const ros::NodeHandle& nh_);

  private:

    // Methods
    bool restart(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void dist_cb(const sensor_msgs::RangeConstPtr& msg);
    void image_cb(const sensor_msgs::ImageConstPtr& msg);
    void target_cb(const sensor_msgs::ImageConstPtr& msg);
    void dynreconf_cb(merbots_ibvs::IBVSConfig& config, uint32_t level);
    void roi_cb(const merbots_tracking::TargetPointsConstPtr& roi_msg);
    void ctrltimer_cb(const ros::TimerEvent& event);

    // Variables
    // ROS
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber roi_sub;
    ros::Subscriber dist_sub;
    ros::Publisher twist_pub;
    ros::Publisher twist_debug_pub;
    ros::Publisher ibvsinfo_pub;
    image_transport::Subscriber target_sub;
    image_transport::Subscriber img_sub;
    ros::Timer control_timer;
    tf::TransformListener tf_listener;
    ros::ServiceServer restart_srv;
    dynamic_reconfigure::Server<merbots_ibvs::IBVSConfig> server;

    // Calibration info
    unsigned width;
    unsigned height;
    double fs;
    double fs_2;
    double u0;
    double v0;

    // Points used to perform IBVS
    boost::mutex mutex_roi;
    cv::Point2i last_pt_tl, last_pt_tr, last_pt_bl;
    cv::Rect last_roi;
    int last_status;
    bool last_roi_valid;
    boost::mutex mutex_target;
    cv::Point2i des_pt_tl, des_pt_tr, des_pt_bl;
    cv::Rect des_roi;

    // Control parameters
    boost::mutex mutex_zdist;
    double z_dist;
    double control_freq;
    cv::Mat_<double> L;
    cv::Mat_<double> J;
    cv::Mat_<double> s;
    double max_vx, max_vy, max_wz;
    double cam_angle, cos_ang, tan_ang;
    PIDController pid_x, pid_y, pid_z;

    // Last received image
    boost::mutex mutex_img;
    cv::Mat last_img;

    // Remaining parameters
    bool init_roi;
    bool init_target;
    bool enable_vely;
    bool debug;
  };
}

#endif
