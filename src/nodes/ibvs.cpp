#include <boost/thread.hpp>

#include <auv_msgs/BodyVelocityReq.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <merbots_ibvs/IBVSConfig.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class IBVS
{
public:
	IBVS() :
			nh("~"),
			it(nh),
            control_freq(10),
            L(6, 6, 0.0),
            J(6, 6, 0.0),
            s(6, 1, 0.0),
            lambda_x(1.0),
            lambda_y(1.0),
            lambda_z(1.0),
            z_dist(1.0),
            init_roi(false),
            init_target(false),
            enable_vely(true),
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
            fs_2 = fs * fs;
            ROS_INFO("Focal length: %.2f", fs);

            u0 = cinfo.K[2];
            v0 = cinfo.K[5];
            ROS_INFO("Principal point: (%.2f, %.2f)", u0, v0);
        }

        // Reading the remaining parameters
        nh.param("control_freq", control_freq, 10.0);
        ROS_INFO("[Params] Control frequency: %f", control_freq);

        nh.param("lambda_x", lambda_x, 1.0);
        ROS_INFO("[Params] Lambda X: %f", lambda_x);

        nh.param("lambda_y", lambda_y, 1.0);
        ROS_INFO("[Params] Lambda Y: %f", lambda_y);

        nh.param("lambda_z", lambda_z, 1.0);
        ROS_INFO("[Params] Lambda Z: %f", lambda_z);

        nh.param("enable_vely", enable_vely, true);
        ROS_INFO("[Params] Enable linear Y velocity: %s", enable_vely ? "Yes":"No");

        nh.param("debug", debug, true);
        ROS_INFO("[Params] Debug: %s", debug ? "Yes":"No");

        std::string robot_frame;
        nh.param<std::string>("robot_frame", robot_frame, "base_link");

        std::string camera_frame;
        nh.param<std::string>("camera_frame", camera_frame, "camera");

        // Computing the transformation between coordinate systems
        tf::StampedTransform Trc;
        try
        {
            // Waiting for the static transform
            tf_listener.waitForTransform(robot_frame, camera_frame, ros::Time(0), ros::Duration(5.0));
            tf_listener.lookupTransform(robot_frame, camera_frame, ros::Time(0), Trc);

            // Getting and transforming the corresponding components
            tf::Matrix3x3 rot = Trc.getBasis();
            cv::Mat_<double> Rrc = cv::Mat::zeros(3, 3, CV_64F);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Rrc(i, j) = rot[i][j];
                }
            }

            tf::Vector3 trans = Trc.getOrigin();
            double x = trans.getX();
            double y = trans.getY();
            double z = trans.getZ();
            cv::Mat_<double> Skew_trc = cv::Mat::zeros(3, 3, CV_64F);
            Skew_trc(0, 0) = 0.0; Skew_trc(0, 1) = -z;   Skew_trc(0, 2) = y;
            Skew_trc(1, 0) = z;   Skew_trc(1, 1) = 0.0;  Skew_trc(1, 2) = -x;
            Skew_trc(2, 0) = -y;  Skew_trc(2, 1) = x;    Skew_trc(2, 2) = 0.0;

            // Computing the jacobian
            cv::Mat_<double> Rcr = Rrc.inv();
            Rcr.copyTo(J.rowRange(0, 3).colRange(0, 3));
            Rcr.copyTo(J.rowRange(3, 6).colRange(3, 6));
            cv::Mat_<double> RS = -Rcr * Skew_trc;
            RS.copyTo(J.rowRange(0, 3).colRange(3, 6));
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Could not get initial transform from camera to robot frame, %s", ex.what());
            ros::shutdown();
        }

        // Subscribing to the topic used to receive ROI's
        roi_sub = nh.subscribe("roi", 0, &IBVS::roi_cb, this);

        // Publishing twist messages
        twist_pub = nh.advertise<auv_msgs::BodyVelocityReq>("twist", 1);

        // Dynamic reconfigure
        server.setCallback(boost::bind(&IBVS::dynreconf_cb, this, _1, _2));

        // Target callback
        target_sub = it.subscribe("target", 0, &IBVS::target_cb, this);

        // Distance subscriber
        dist_sub = nh.subscribe("dist", 0, &IBVS::dist_cb, this);

        // Image Subscriber for debugging purposes
        if (debug)
        {
            img_sub = it.subscribe("image", 0, &IBVS::image_cb, this);
        }

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

    // Image callback
    void image_cb(const sensor_msgs::ImageConstPtr& msg)
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

        mutex_img.lock();

        cv_ptr->image.copyTo(last_img);

        mutex_img.unlock();
    }

    // Target callback
    void target_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        int mid_w = static_cast<int>(width / 2.0);
        int mid_h = static_cast<int>(height / 2.0);

        int mid_w_roi = static_cast<int>(msg->width / 2.0);
        int mid_h_roi = static_cast<int>(msg->height / 2.0);

        mutex_target.lock();
        // Updating the points
        des_pt_tl.x = mid_w - mid_w_roi;
        des_pt_tl.y = mid_h - mid_h_roi;
        des_pt_tr.x = mid_w + mid_w_roi;
        des_pt_tr.y = mid_h - mid_h_roi;
        des_pt_bl.x = mid_w - mid_w_roi;
        des_pt_bl.y = mid_h + mid_h_roi;

        // Updating the current desired ROI
        des_roi.x = des_pt_tl.x;
        des_roi.y = des_pt_tl.y;
        des_roi.width = msg->width;
        des_roi.height = msg->height;

        if (!init_target)
        {
            init_target = true;
        }

        mutex_target.unlock();
    }

    void dynreconf_cb(merbots_ibvs::IBVSConfig& config, uint32_t level)
    {
        mutex_lambdas.lock();

        // Adapting the correspondent parameters dynamically
        lambda_x = config.lambda_x;
        lambda_y = config.lambda_y;
        lambda_z = config.lambda_z;

        mutex_lambdas.unlock();
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

        // Updating the ROI
        last_roi.x = roi_msg->x_offset;
        last_roi.y = roi_msg->y_offset;
        last_roi.width = roi_msg->width;
        last_roi.height = roi_msg->height;

        if (!init_roi)
        {
            init_roi = true;
        }

        mutex_roi.unlock();
    }

    void ctrltimer_cb(const ros::TimerEvent& event)
    {
        // Getting current features and assessing if the control should be done
        int u1, v1;
        int u2, v2;
        int u3, v3;
        cv::Rect roi;
        bool in_roi = false;
        mutex_roi.lock();
        u1 = last_pt_tl.x;
        v1 = last_pt_tl.y;
        u2 = last_pt_tr.x;
        v2 = last_pt_tr.y;
        u3 = last_pt_bl.x;
        v3 = last_pt_bl.y;
        roi = last_roi;
        if (init_roi)
        {
            in_roi = true;
        }
        mutex_roi.unlock();

        // Getting desired features
        int u1_d, v1_d;
        int u2_d, v2_d;
        int u3_d, v3_d;
        cv::Rect roi_d;
        bool in_target = false;
        mutex_target.lock();
        u1_d = des_pt_tl.x;
        v1_d = des_pt_tl.y;
        u2_d = des_pt_tr.x;
        v2_d = des_pt_tr.y;
        u3_d = des_pt_bl.x;
        v3_d = des_pt_bl.y;
        roi_d = des_roi;
        if (init_target)
        {
            in_target = true;
        }
        mutex_target.unlock();

        // Assessing if the control should be done
        if (in_roi && in_target)
        {
            // Controlling the vehicle
            // Getting distance to the points
            // FIXME We assume all the point are at the same distance
            double z1, z2, z3;
            mutex_zdist.lock();
            z1 = z_dist;
            z2 = z_dist;
            z3 = z_dist;
            mutex_zdist.unlock();

            // Getting lambdas
            double lamb_x, lamb_y, lamb_z;
            mutex_lambdas.lock();
            lamb_x = lambda_x;
            lamb_y = lambda_y;
            lamb_z = lambda_z;
            mutex_lambdas.unlock();

            // --- Interaction matrix L ---
            // Primes
            double up1 = u1 - u0;
            double vp1 = v1 - v0;
            double up2 = u2 - u0;
            double vp2 = v2 - v0;
            double up3 = u3 - u0;
            double vp3 = v3 - v0;

            // Squared values
            double up1_2 = up1 * up1;
            double vp1_2 = vp1 * vp1;
            double up2_2 = up2 * up2;
            double vp2_2 = vp2 * vp2;
            double up3_2 = up3 * up3;
            double vp3_2 = vp3 * vp3;

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
            L(1, 3) = (fs_2 + vp1_2) / fs;
            L(2, 3) = (up2 * vp2) / fs;
            L(3, 3) = (fs_2 + vp2_2) / fs;
            L(4, 3) = (up3 * vp3) / fs;
            L(5, 3) = (fs_2 + vp3_2) / fs;

            // L Column 4
            L(0, 4) = -(fs_2 + up1_2) / fs;
            L(1, 4) = -(up1 * vp1) / fs;
            L(2, 4) = -(fs_2 + up2_2) / fs;
            L(3, 4) = -(up2 * vp2) / fs;
            L(4, 4) = -(fs_2 + up3_2) / fs;
            L(5, 4) = -(up3 * vp3) / fs;

            // L Column 5
            L(0, 5) = vp1;
            L(1, 5) = -up1;
            L(2, 5) = vp2;
            L(3, 5) = -up2;
            L(4, 5) = vp3;
            L(5, 5) = -up3;

            // Transforming interaction matrix L into L'
            cv::Mat_<double> Lp = L * J;

            // --- Filling the error vector ---
            s(0,0) = u1 - u1_d;
            s(1,0) = v1 - v1_d;
            s(2,0) = u2 - u2_d;
            s(3,0) = v2 - v2_d;
            s(4,0) = u3 - u3_d;
            s(5,0) = v3 - v3_d;

            // Selecting the corresponding columns of Lp
            cv::Mat_<double> Lp_s;
            if (enable_vely)
            {
                // Y velocities can be generated
                Lp_s = cv::Mat::zeros(6, 3, CV_64F);
                Lp.col(0).copyTo(Lp_s.col(0));
                Lp.col(1).copyTo(Lp_s.col(1));
                Lp.col(5).copyTo(Lp_s.col(2));
            }
            else
            {
                Lp_s = cv::Mat::zeros(6, 2, CV_64F);
                Lp.col(0).copyTo(Lp_s.col(0));
                Lp.col(5).copyTo(Lp_s.col(1));
            }

            // We compute the pseudoinverse matrix
            cv::Mat_<double> Lp_s_inv = (Lp_s.t() * Lp_s).inv() * Lp_s.t();

            // --- Computing the motion command ---
            cv::Mat_<double> vels = Lp_s_inv * s;

            // --- Filling and publishing the corresponding message ---
            auv_msgs::BodyVelocityReq curr_twist;
            curr_twist.header.stamp = ros::Time::now();
            curr_twist.header.frame_id = "ibvs";

            curr_twist.goal.requester = "ibvs";
            curr_twist.goal.id = 0;
            curr_twist.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_TELEOPERATION_HIGH; // FIXME

            curr_twist.twist.linear.x = -lamb_x * vels(0, 0);
            curr_twist.disable_axis.x = 0;
            if (enable_vely)
            {
                curr_twist.twist.linear.y = -lamb_y * vels(1, 0);
                curr_twist.disable_axis.y = 0;
            }
            else
            {
                curr_twist.twist.linear.y = 0.0;
                curr_twist.disable_axis.y = 1;
            }
            curr_twist.twist.linear.z = 0.0;
            curr_twist.disable_axis.z = 1;
            curr_twist.twist.angular.x = 0.0;
            curr_twist.disable_axis.roll = 1;
            curr_twist.twist.angular.y = 0.0;
            curr_twist.disable_axis.pitch = 1;
            if (enable_vely)
            {
                curr_twist.twist.angular.z = -lamb_z * vels(2, 0);
            }
            else
            {
                curr_twist.twist.angular.z = -lamb_z * vels(1, 0);
            }
            curr_twist.disable_axis.yaw = 0;
            twist_pub.publish(curr_twist);

            // Showing debug image if needed
            if (debug)
            {
                cv::Mat img;
                mutex_img.lock();
                last_img.copyTo(img);
                mutex_img.unlock();

                // Lines
                cv::line(img, cv::Point(u1, v1), cv::Point(u1_d, v1_d), cv::Scalar(255, 255, 255), 2);
                cv::line(img, cv::Point(u2, v2), cv::Point(u2_d, v2_d), cv::Scalar(255, 255, 255), 2);
                cv::line(img, cv::Point(u3, v3), cv::Point(u3_d, v3_d), cv::Scalar(255, 255, 255), 2);

                // Printing current roi
                cv::rectangle(img, roi, cv::Scalar(0, 255, 0), 2);

                // Printing desired coordinates
                cv::circle(img, cv::Point(u1_d, v1_d), 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, cv::Point(u2_d, v2_d), 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, cv::Point(u3_d, v3_d), 3, cv::Scalar(0, 0, 255), -1);

                // Printing current coordinates
                cv::circle(img, cv::Point(u1, v1), 3, cv::Scalar(255, 0, 0), -1);
                cv::circle(img, cv::Point(u2, v2), 3, cv::Scalar(255, 0, 0), -1);
                cv::circle(img, cv::Point(u3, v3), 3, cv::Scalar(255, 0, 0), -1);

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
    image_transport::Subscriber img_sub;
    ros::Timer control_timer;
    tf::TransformListener tf_listener;
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
    boost::mutex mutex_lambdas;
    double lambda_x, lambda_y, lambda_z;

    // Last received image
    boost::mutex mutex_img;
    cv::Mat last_img;

    // Remaining parameters
    bool init_roi;
    bool init_target;
    bool enable_vely;
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
