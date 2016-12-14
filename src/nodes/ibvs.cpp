#include <boost/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <merbots_ibvs/IBVSConfig.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RegionOfInterest.h>

class IBVS
{
public:
	IBVS() :
			nh("~"),
			it(nh),
            control_freq(10),
            L(6, 6, 0.0),
            e(6, 1, 0.0),
            lambda(1.0),
            z_dist(1.0),
            init(false),
            debug(false)
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

            // Debug
            nh.param("debug", debug, true);
            ROS_INFO("[Params] Debug: %s", debug ? "Yes":"No");
        }

        // Subscribing to the topic used to receive ROI's
        roi_sub = nh.subscribe("roi", 1, &IBVS::roi_cb, this);

        // Publishing twist messages
        twist_pub = nh.advertise<geometry_msgs::Twist>("twist", 1);

        // Dynamic reconfigure
        server.setCallback(boost::bind(&IBVS::dynreconf_cb, this, _1, _2));

        // Target callback
        target_sub = it.subscribe("target", 0, &IBVS::target_cb, this);

        // Distance subscriber
        dist_sub = nh.subscribe("dist", 0, &IBVS::dist_cb, this);

        // Control timer
        control_timer = nh.createTimer(ros::Duration(1.0 / control_freq), &IBVS::ctrltimer_cb, this);
	}

    void dist_cb(const sensor_msgs::RangeConstPtr& msg)
    {
        mutex_zdist.lock();

        // Receiving new distance
        z_dist = msg->range;

        mutex_zdist.unlock();
    }

    // Target callback
    void target_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        // Converting the image message to OpenCV format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Error converting image to OpenCV format: %s", e.what());
            return;
        }

        int mid_w = static_cast<int>(width / 2.0);
        int mid_h = static_cast<int>(height / 2.0);

        int mid_w_roi = static_cast<int>(cv_ptr->image.cols / 2.0);
        int mid_h_roi = static_cast<int>(cv_ptr->image.rows / 2.0);

        mutex_dpts.lock();
        dpt_tl.x = mid_w - mid_w_roi;
        dpt_tl.y = mid_h - mid_h_roi;
        dpt_tr.x = mid_w + mid_w_roi;
        dpt_tr.y = mid_h - mid_h_roi;
        dpt_bl.x = mid_w - mid_w_roi;
        dpt_bl.y = mid_h + mid_h_roi;
        mutex_dpts.unlock();
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
            // Considering all the points at the same distance
            double z1, z2, z3;
            mutex_zdist.lock();
            z1 = z_dist;
            z2 = z_dist;
            z3 = z_dist;
            mutex_zdist.unlock();

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

            cv::Point2i dpoint_tl, dpoint_tr, dpoint_bl;
            // Getting the desired positions
            mutex_dpts.lock();

            dpoint_tl = dpt_tl;
            dpoint_tr = dpt_tr;
            dpoint_bl = dpt_bl;

            mutex_dpts.unlock();

            // Filling the error vector
            e(0,0) = pt_tl.x - dpoint_tl.x;
            e(1,0) = -(pt_tl.y - dpoint_tl.y);
            e(2,0) = pt_tr.x - dpoint_tr.x;
            e(3,0) = -(pt_tr.y - dpoint_tr.y);
            e(4,0) = pt_bl.x - dpoint_bl.x;
            e(5,0) = -(pt_bl.y - dpoint_bl.y);

            // Computing the velocities
            cv::Mat_<double> vels = -lambda * (L.inv() * e);

            // Filling and publishing the message
            curr_twist.linear.x = vels(0, 0);
            curr_twist.linear.y = vels(1, 0);
            curr_twist.linear.z = vels(2, 0);
            curr_twist.angular.x = vels(3, 0);
            curr_twist.angular.y = vels(4, 0);
            curr_twist.angular.z = vels(5, 0);
            twist_pub.publish(curr_twist);

            // Showing debug image if needed
            if (debug)
            {
                cv::Mat img = cv::Mat::zeros(height, width, CV_8UC(3));

                // Lines
                cv::line(img, pt_tl, dpoint_tl, cv::Scalar(255, 255, 255), 2);
                cv::line(img, pt_tr, dpoint_tr, cv::Scalar(255, 255, 255), 2);
                cv::line(img, pt_bl, dpoint_bl, cv::Scalar(255, 255, 255), 2);

                // Printing desired coordinates
                cv::circle(img, dpoint_tl, 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, dpoint_tr, 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, dpoint_bl, 3, cv::Scalar(0, 0, 255), -1);

                // Printing current coordinates
                cv::circle(img, pt_tl, 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(img, pt_tr, 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(img, pt_bl, 3, cv::Scalar(0, 255, 0), -1);

                cv::imshow("IBVS", img);
                cv::waitKey(5);
            }
        }
    }

private:
    // ROS
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
    ros::Subscriber roi_sub;
    ros::Subscriber dist_sub;
    ros::Publisher twist_pub;
    image_transport::Subscriber target_sub;
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
    boost::mutex mutex_dpts;
    cv::Point2i last_pt_tl, last_pt_tr, last_pt_bl;
    cv::Point2i pt_tl, pt_tr, pt_bl;
    cv::Point2i dpt_tl, dpt_tr, dpt_bl;

    // Control parameters
    boost::mutex mutex_zdist;
    double control_freq;
    cv::Mat_<double> L;
    cv::Mat_<double> e;
    double lambda;
    double z_dist;

    // Remaining parameters
    bool init;
    bool debug;
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
