#include <boost/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>

class IBVS
{
public:
	IBVS() :
			nh(),
			it(nh),
            control_freq(10)
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
        }

        // Subscribing to the topic used to receive ROI's
        roi_sub = nh.subscribe("roi", 1, &IBVS::roi_cb, this);

        // Control timer
        control_timer = nh.createTimer(ros::Duration(1.0 / control_freq), &IBVS::ctrltimer_cb, this);
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

        mutex_roi.unlock();
    }

    void ctrltimer_cb(const ros::TimerEvent&)
    {
        // Copying the current points
        mutex_roi.lock();

        pt_tl = last_pt_tl;
        pt_tr = last_pt_tr;
        pt_bl = last_pt_bl;

        mutex_roi.unlock();

//        ROS_INFO("Current coordinates: (%i, %i), (%i, %i), (%i, %i)",
//                 pt_tl.x, pt_tl.y,
//                 pt_tr.x, pt_tr.y,
//                 pt_bl.x, pt_bl.y
//        );
    }

private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
    ros::Subscriber roi_sub;
    ros::Timer control_timer;

    // Calibration info
    unsigned width;
    unsigned height;
    double fs;
    double u0;
    double v0;

    // Points used to perform IBVS
    boost::mutex mutex_roi;
    cv::Point2i last_pt_tl, last_pt_tr, last_pt_bl;
    cv::Point2i pt_tl, pt_tr, pt_bl;
    cv::Point2i dpt_tl, dpt_tr, dpt_bl;

    // Remaining parameters
    double control_freq;
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
