#ifndef PID_H
#define PID_H

#include <boost/thread.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

class PIDController
{
public:

    PIDController();
    PIDController(double& kp_, double& kd_, double& ki_);

    void reset();
    void setKp(const double& kp_);
    void setKd(const double& kd_);
    void setKi(const double& ki_);
    void getCommand(const ros::Time& t, const cv::Mat_<double>& s_p, cv::Mat_<double>& res);

private:

    double computeDeltaTime(const ros::Time& t);
    void saturate(cv::Mat_<double>& error_sum, double npix = 5.0);

    // PID constants
    boost::mutex mutex_cts;
    double kp, kd, ki;

    ros::Time last_time;
    cv::Mat_<double> last_error;
    cv::Mat_<double> error_sum;
};

#endif // PID_H
