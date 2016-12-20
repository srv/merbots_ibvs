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
            L(2, 6, 0.0),
            J(6, 6, 0.0),
            s(2, 1, 0.0),
            lambda_x(1.0),
            lambda_y(1.0),
            lambda_z(1.0),
            z_dist(1.0),
            init_roi(false),
            init_target(false),
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

            // Computing the desired coordinates of the features
            des_pt.x = static_cast<int>(width / 2.0);
            des_pt.y = static_cast<int>(height / 2.0);
            des_area = 1;
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
        mutex_target.lock();

        // Updating the current desired area
        des_area = msg->width * msg->height;

        // Updating the current desired ROI
        int mid_x = static_cast<int>(msg->width / 2.0);
        int mid_y = static_cast<int>(msg->height / 2.0);
        des_roi.x = des_pt.x - mid_x;
        des_roi.y = des_pt.y - mid_y;
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

        // Mid sizes of the ROI
        int mid_x = static_cast<int>(roi_msg->width / 2.0);
        int mid_y = static_cast<int>(roi_msg->height / 2.0);

        // Computing the feature points from the ROI
        last_pt.x = roi_msg->x_offset + mid_x;
        last_pt.y = roi_msg->y_offset + mid_y;
        last_area = roi_msg->width * roi_msg->height;
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

    void ctrltimer_cb(const ros::TimerEvent&)
    {
        // Getting current features and assessing if the control should be done
        int u, v, a;
        cv::Rect roi;
        bool in_roi = false;
        mutex_roi.lock();
        u = last_pt.x;
        v = last_pt.y;
        a = last_area;
        roi = last_roi;
        if (init_roi)
        {
            in_roi = true;
        }
        mutex_roi.unlock();

        // Getting desired features
        int u_d, v_d, a_d;
        cv::Rect roi_d;
        bool in_target = false;
        mutex_target.lock();
        u_d = des_pt.x;
        v_d = des_pt.y;
        a_d = des_area;
        roi_d = des_roi;
        if (init_target)
        {
            in_target = true;
        }
        mutex_target.unlock();

        // Assessing if the control should be done
        bool do_control = false;
        if (in_roi && in_target)
        {
            do_control = true;
        }

        // Controlling the vehicle
        if (do_control)
        {
            // Getting distance to the point
            double z;
            mutex_zdist.lock();
            z = z_dist;
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
            double up = u - u0;
            double vp = v - v0;

            // Squared values
            double up_2 = up * up;
            double vp_2 = vp * vp;

            // L Column 0
            L(0, 0) = -(fs / z);

            // L Column 1
            L(1, 1) = -(fs / z);

            // L Column 2
            L(0, 2) = up / z;
            L(1, 2) = vp / z;

            // L Column 3
            L(0, 3) = (up * vp) / fs;
            L(1, 3) = (fs_2 + vp_2) / fs;

            // L Column 4
            L(0, 4) = -(fs_2 + up_2) / fs;
            L(1, 4) = -(up * vp) / fs;

            // L Column 5
            L(0, 5) = vp;
            L(1, 5) = -up;

            // Transforming interaction matrix L into L'
            cv::Mat_<double> Lp = L * J;

            // Obtaining the required columns from Lp
            cv::Mat_<double> L_yz(2, 2);
            Lp.col(1).copyTo(L_yz.col(0));
            Lp.col(5).copyTo(L_yz.col(1));

            cv::Mat_<double> L_x(2, 1);
            Lp.col(0).copyTo(L_x.col(0));

            // --- Computing the motion command ---

            // Computing linear velocity in x (forward axis)
            cv::Mat_<double> vx(1, 1, 0.0);
            if (a > 0 && a_d > 0)
            {
                vx(0, 0) = -lamb_x * (log(a) - log(a_d));
            }

            // Preparing the error vector
            s(0, 0) = lamb_y * (u - u_d);
            s(1, 0) = lamb_z * (v - v_d);

            // Computing the yz velocities
            cv::Mat_<double> vels = L_yz.inv() * (s - (L_x * vx));

            // Filling and publishing the corresponding message
            auv_msgs::BodyVelocityReq curr_twist;
            curr_twist.header.stamp = ros::Time::now();
            curr_twist.header.frame_id = "ibvs";
            curr_twist.goal.requester = "ibvs";
            curr_twist.goal.id = 0;
            curr_twist.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_TELEOPERATION_HIGH; // FIXME
            curr_twist.twist.linear.x = vx(0, 0);
            curr_twist.twist.linear.y = vels(0, 0);
            curr_twist.twist.linear.z = 0.0;
            curr_twist.twist.angular.x = 0.0;
            curr_twist.twist.angular.y = 0.0;
            curr_twist.twist.angular.z = vels(1, 0);
            curr_twist.disable_axis.x = 0;
            curr_twist.disable_axis.y = 0;
            curr_twist.disable_axis.z = 1;
            curr_twist.disable_axis.roll = 1;
            curr_twist.disable_axis.pitch = 1;
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
                cv::line(img, cv::Point(u, v), cv::Point(u_d, v_d), cv::Scalar(255, 255, 255), 2);

                // Printing desired coordinates
                cv::circle(img, cv::Point(u_d, v_d), 3, cv::Scalar(0, 0, 255), -1);

                // Printing current coordinates
                cv::circle(img, cv::Point(u, v), 3, cv::Scalar(0, 255, 0), -1);

                cv::rectangle(img, roi, cv::Scalar(0, 255, 0), 2);
                cv::rectangle(img, roi_d, cv::Scalar(0, 0, 255), 2);

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
    cv::Point2i last_pt;
    int last_area;
    cv::Rect last_roi;
    boost::mutex mutex_target;
    cv::Point2i des_pt;
    int des_area;
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
