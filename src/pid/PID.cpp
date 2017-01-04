#include "PID.h"

PIDController::PIDController()
{
    kp = 0.0;
    kd = 0.0;
    ki = 0.0;
    reset();
}

PIDController::PIDController(double& kp_, double& kd_, double& ki_)
{
    kp = kp_;
    kd = kd_;
    ki = ki_;
    reset();
}

void PIDController::reset()
{
    last_time = ros::Time();

    last_error = cv::Mat::zeros(6, 1, CV_64F);
    last_error.setTo(std::numeric_limits<double>::max());

    error_sum = cv::Mat::zeros(6, 1, CV_64F);
}

void PIDController::setKp(const double& kp_)
{
    mutex_cts.lock();
    kp = kp_;
    mutex_cts.unlock();
}

void PIDController::setKd(const double& kd_)
{
    mutex_cts.lock();
    kd = kd_;
    mutex_cts.unlock();
}

void PIDController::setKi(const double& ki_)
{
    mutex_cts.lock();
    ki = ki_;
    mutex_cts.unlock();
}

double PIDController::computeDeltaTime(const ros::Time& t)
{
    double dt = 0.0;

    if(!last_time.isZero())
    {
        dt = (t - last_time).toSec();
    }

    last_time = t;

    return dt;
}

void PIDController::saturate(cv::Mat_<double>& error_sum, double npix)
{
    for (int i = 0; i < error_sum.rows; i++)
    {
        if (error_sum(i, 0) > npix)
        {
            error_sum(i, 0) = npix;
        }
        else if (error_sum(i, 0) < npix)
        {
            error_sum(i, 0) = -npix;
        }
    }
}

void PIDController::getCommand(const ros::Time& t, const cv::Mat_<double>& s_p, cv::Mat_<double>& res)
{
    double dt = computeDeltaTime(t);

    // Derivative
    cv::Mat_<double> derror = cv::Mat::zeros(6, 1, CV_64F);
    if (last_error(0, 0) != std::numeric_limits<double>::max())
    {
        derror = (s_p - last_error) / dt;
    }

    // Computing PID output
    // Trapezoidal integration
    error_sum = error_sum + (((s_p + last_error) / 2.0) * dt);
    saturate(error_sum, 2);

    // Updating the last error
    s_p.copyTo(last_error);

    // Getting current constants
    double Kp, Kd, Ki;
    mutex_cts.lock();
    Kp = kp;
    Kd = kd;
    Ki = ki;
    mutex_cts.unlock();

    // PID output
    cv::Mat_<double> pid_out = (Kp * s_p) + (Kd * derror) + (Ki * error_sum);

    pid_out.copyTo(res);
}
