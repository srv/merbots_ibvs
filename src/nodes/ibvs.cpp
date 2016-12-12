#include <boost/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <merbots_ibvs/IBVSConfig.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>

class IBVS
{
public:
	IBVS() :
			nh(),
			it(nh),
            control_freq(10),
            L(6, 6, 0.0),
            e(6, 1, 0.0),
            lambda(1.0),
            init(false)
	{
        // Reading calibration information
        ROS_INFO("Waiting for calibration information (10s) ...");

        boost::shared_ptr<sensor_msgs::CameraInfo const> sp;
        sp = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", ros::Duration(10));

        if (sp == NULL)
        {
            // No calibration received
            ROS_FATAL("No calibration received");
            ros::shutdown();
        }
        else
        {
            // Processing the received calibration
            sensor_msgs::CameraInfo cinfo = *sp;

            ROS_INFO("Calibration data:");

            width = cinfo.width;
            ROS_INFO("Image width: %u", width);

            height = cinfo.height;
            ROS_INFO("Image height: %u", height);

            fs = cinfo.K[0];
            fs_sq = fs * fs;
            ROS_INFO("Focal length: %.2f", fs);

            u0 = cinfo.K[2];
            v0 = cinfo.K[5];
            ROS_INFO("Principal point: (%.2f, %.2f)", u0, v0);

            // Computing the desired coordinates of the ROI
            int factor_w = static_cast<int>(width / 4.0);
            int factor_h = static_cast<int>(height / 4.0);

            dpt_tl.x = factor_w;
            dpt_tl.y = factor_h;

            dpt_tr.x = width - factor_w;
            dpt_tr.y = factor_h;

            dpt_bl.x = factor_w;
            dpt_bl.y = height - factor_h;

            ROS_INFO("Desired coordinates: (%i, %i), (%i, %i), (%i, %i)",
                     dpt_tl.x, dpt_tl.y,
                     dpt_tr.x, dpt_tr.y,
                     dpt_bl.x, dpt_bl.y
            );

            // Reading the remaining parameters
            nh.param("control_freq", control_freq, 10.0);
            ROS_INFO("[Params] Control frequency: %f", control_freq);

            nh.param("lambda", lambda, 0.2);
            ROS_INFO("[Params] Lambda: %f", lambda);
        }

        // Subscribing to the topic used to receive ROI's
        roi_sub = nh.subscribe("roi", 1, &IBVS::roi_cb, this);

        // Publishing twist messages
        twist_pub = nh.advertise<geometry_msgs::Twist>("twist", 1);

        // Control timer
        control_timer = nh.createTimer(ros::Duration(1.0 / control_freq), &IBVS::ctrltimer_cb, this);

        // Dynamic reconfigure
        server.setCallback(boost::bind(&IBVS::dynreconf_cb, this, _1, _2));
	}

    void dynreconf_cb(merbots_ibvs::IBVSConfig& config, uint32_t level)
    {
        // Adapting the correspondent parameters dynamically
        lambda = config.lambda;
    }

    void roi_cb(const sensor_msgs::RegionOfInterestConstPtr& roi_msg)
    {
        mutex_roi.lock();

        // Computing the feature points from the ROI
        last_pt_tl.x = roi_msg->x_offset;
        last_pt_tl.y = roi_msg->y_offset;

        last_pt_tr.x = roi_msg->x_offset + roi_msg->width;
        last_pt_tr.y = roi_msg->y_offset;

        last_pt_bl.x = roi_msg->x_offset;
        last_pt_bl.y = roi_msg->y_offset + roi_msg->height;

        if (!init)
        {
            init = true;
        }

        mutex_roi.unlock();
    }

    void ctrltimer_cb(const ros::TimerEvent&)
    {
        bool do_control = false;

        // Copying the current points
        mutex_roi.lock();

        pt_tl = last_pt_tl;
        pt_tr = last_pt_tr;
        pt_bl = last_pt_bl;

        if (init)
        {
            do_control = true;
        }

        mutex_roi.unlock();

        if (do_control)
        {
            // TODO Point depth estimation. Currently, we use a fixed value.
            double z1 = 1.0;
            double z2 = 1.0;
            double z3 = 1.0;

            // --- Interaction matrix L ---
            // Primes
            double up1 = u0 - pt_tl.x;
            double vp1 = v0 - pt_tl.y;
            double up2 = u0 - pt_tr.x;
            double vp2 = v0 - pt_tr.y;
            double up3 = u0 - pt_bl.x;
            double vp3 = v0 - pt_bl.y;

            // Squared values
            double up1_sq = up1 * up1;
            double vp1_sq = vp1 * vp1;
            double up2_sq = up2 * up2;
            double vp2_sq = vp2 * vp2;
            double up3_sq = up3 * up3;
            double vp3_sq = vp3 * vp3;

            // L Column 0
            L(0, 0) = -(fs / z1);
            L(2, 0) = -(fs / z2);
            L(4, 0) = -(fs / z3);

            // L Column 1
            L(1, 1) = -(fs / z1);
            L(3, 1) = -(fs / z2);
            L(5, 1) = -(fs / z3);

            // L Column 2
            L(0, 2) = up1 / z1;
            L(1, 2) = vp1 / z1;
            L(2, 2) = up2 / z2;
            L(3, 2) = vp2 / z2;
            L(4, 2) = up3 / z3;
            L(5, 2) = vp3 / z3;

            // L Column 3
            L(0, 3) = (up1 * vp1) / fs;
            L(1, 3) = (fs_sq * vp1_sq) / fs;
            L(2, 3) = (up2 * vp2) / fs;
            L(3, 3) = (fs_sq * vp2_sq) / fs;
            L(4, 3) = (up3 * vp3) / fs;
            L(5, 3) = (fs_sq * vp3_sq) / fs;

            // L Column 4
            L(0, 4) = -(fs_sq + up1_sq) / fs;
            L(1, 4) = -(up1 * vp1) / fs;
            L(2, 4) = -(fs_sq + up2_sq) / fs;
            L(3, 4) = -(up2 * vp2) / fs;
            L(4, 4) = -(fs_sq + up3_sq) / fs;
            L(5, 4) = -(up3 * vp3) / fs;

            // L Column 5
            L(0, 5) = vp1;
            L(1, 5) = -up1;
            L(2, 5) = vp2;
            L(3, 5) = -up2;
            L(4, 5) = vp3;
            L(5, 5) = -up3;

            // --- Computing the motion command ---
            // Filling the error vector
            e(0,0) = pt_tl.x - dpt_tl.x;
            e(1,0) = -(pt_tl.y - dpt_tl.y);
            e(2,0) = pt_tr.x - dpt_tr.x;
            e(3,0) = -(pt_tr.y - dpt_tr.y);
            e(4,0) = pt_bl.x - dpt_bl.x;
            e(5,0) = -(pt_bl.y - dpt_bl.y);

            // Computing the velocities
            cv::Mat_<double> vels = -lambda * (L.inv() * e);

            // Filling the message
            curr_twist.linear.x = vels(0, 0);
            curr_twist.linear.y = vels(1, 0);
            curr_twist.linear.z = vels(2, 0);
            curr_twist.angular.x = vels(3, 0);
            curr_twist.angular.y = vels(4, 0);
            curr_twist.angular.z = vels(5, 0);
            twist_pub.publish(curr_twist);
        }
    }

private:
    // ROS
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
    ros::Subscriber roi_sub;
    ros::Publisher twist_pub;
    ros::Timer control_timer;
    geometry_msgs::Twist curr_twist;
    dynamic_reconfigure::Server<merbots_ibvs::IBVSConfig> server;

    // Calibration info
    unsigned width;
    unsigned height;
    double fs;
    double fs_sq;
    double u0;
    double v0;

    // Points used to perform IBVS
    boost::mutex mutex_roi;
    cv::Point2i last_pt_tl, last_pt_tr, last_pt_bl;
    cv::Point2i pt_tl, pt_tr, pt_bl;
    cv::Point2i dpt_tl, dpt_tr, dpt_bl;

    // Control parameters
    double control_freq;
    cv::Mat_<double> L;
    cv::Mat_<double> e;
    double lambda;

    // Remaining parameters
    bool init;
};

int main(int argc, char** argv)
{
	// ROS
	ros::init(argc, argv, "ibvs");
    ROS_INFO("MERBOTS Image-Based Visual Servoing");
	IBVS ibvs;
	ros::spin();

	return 0;
}
