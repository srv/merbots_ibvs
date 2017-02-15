#include <merbots_ibvs/ibvs/IBVS.h>

#define SQ(x) ((x)*(x))

namespace merbots_ibvs
{
	IBVS::IBVS(const ros::NodeHandle& nh_) :
	nh(nh_),
	it(nh_),
	control_freq(10),
	L(6, 6, 0.0),
	J(6, 6, 0.0),
	s(6, 1, 0.0),
	max_vx(0.6),
	max_vy(0.3),
	max_wz(0.25),
	cam_angle(0.0),
	z_dist(1.0),
	init_roi(false),
	last_status(0),
	last_roi_valid(false),
	init_target(false),
	enable_vely(true),
	debug(false),
	resize_debug_img(0.5)
	{
		// Reading calibration information
		ROS_INFO("Waiting for calibration information (10s) ...");

		boost::shared_ptr<sensor_msgs::CameraInfo const> sp;
		sp = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", nh, ros::Duration(10));

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

			// Get the binning factors
			int binning_x = cinfo.binning_x;
			int binning_y = cinfo.binning_y;

			width = cinfo.width;
			if (binning_x > 1)
			{
				width /= binning_x;
			}
			ROS_INFO("Image width: %u", width);

			height = cinfo.height;
			if (binning_y > 1)
			{
				height /= binning_y;
			}
			ROS_INFO("Image height: %u", height);

			fs = cinfo.K[0];
			if (binning_x > 1)
			{
				fs /= binning_x;
			}
			fs_2 = fs * fs;
			ROS_INFO("Focal length: %.2f", fs);

			u0 = cinfo.K[2];
			if (binning_x > 1)
			{
				u0 /= binning_x;
			}

			v0 = cinfo.K[5];
			if (binning_y > 1)
			{
				v0 /= binning_y;
			}
			ROS_INFO("Principal point: (%.2f, %.2f)", u0, v0);
		}

		// Reading the remaining parameters
		nh.param("control_freq", control_freq, 10.0);
		ROS_INFO("[Params] Control frequency: %f", control_freq);

		// PID parameters
		double kp_x, kd_x, ki_x;
		nh.param("kp_x", kp_x, 1.0);
		ROS_INFO("[Params] Kp x: %f", kp_x);
		pid_x.setKp(kp_x);
		nh.param("kd_x", kd_x, 0.0);
		ROS_INFO("[Params] Kd x: %f", kd_x);
		pid_x.setKd(kd_x);
		nh.param("ki_x", ki_x, 0.0);
		ROS_INFO("[Params] Ki x: %f", ki_x);
		pid_x.setKi(ki_x);

		double kp_y, kd_y, ki_y;
		nh.param("kp_y", kp_y, 1.0);
		ROS_INFO("[Params] Kp y: %f", kp_y);
		pid_y.setKp(kp_y);
		nh.param("kd_y", kd_y, 0.0);
		ROS_INFO("[Params] Kd y: %f", kd_y);
		pid_y.setKd(kd_y);
		nh.param("ki_y", ki_y, 0.0);
		ROS_INFO("[Params] Ki y: %f", ki_y);
		pid_y.setKi(ki_y);

		double kp_z, kd_z, ki_z;
		nh.param("kp_z", kp_z, 1.0);
		ROS_INFO("[Params] Kp z: %f", kp_z);
		pid_z.setKp(kp_z);
		nh.param("kd_z", kd_z, 0.0);
		ROS_INFO("[Params] Kd z: %f", kd_z);
		pid_z.setKd(kd_z);
		nh.param("ki_z", ki_z, 0.0);
		ROS_INFO("[Params] Ki z: %f", ki_z);
		pid_z.setKi(ki_z);

		nh.param("max_vx", max_vx, 0.6);
		ROS_INFO("[Params] Max linear velocity in X axis: %f", max_vx);

		nh.param("max_vy", max_vy, 0.3);
		ROS_INFO("[Params] Max linear velocity in Y axis: %f", max_vy);

		nh.param("max_wz", max_wz, 0.25);
		ROS_INFO("[Params] Max angular velocity around Z axis: %f", max_wz);

		nh.param("camera_angle", cam_angle, 0.0);
		ROS_INFO("[Params] Camera angle regarding vertical axis: %f", cam_angle);

		// Computing cosine and tangent of the angle
		tan_ang = tan(cam_angle);
		cos_ang = cos(cam_angle);

		nh.param("enable_vely", enable_vely, true);
		ROS_INFO("[Params] Enable linear Y velocity: %s", enable_vely ? "Yes":"No");

		std::string target_file;
		nh.param<std::string>("target_from_file", target_file, "");

		if (target_file != "")
		{
			cv::Mat image = cv::imread(target_file);

			int mid_w = static_cast<int>(width / 2.0);
			int mid_h = static_cast<int>(height / 2.0);

			int mid_w_roi = static_cast<int>(image.cols / 2.0);
			int mid_h_roi = static_cast<int>(image.rows / 2.0);

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
			des_roi.width = image.cols;
			des_roi.height = image.rows;

			if (!init_target)
			{
				init_target = true;
			}

			mutex_target.unlock();
		}

		nh.param("debug", debug, true);
		ROS_INFO("[Params] Debug: %s", debug ? "Yes":"No");

		nh.param("resize_debug_img", resize_debug_img, 0.5);
		ROS_INFO("[Params] Debug image resize: %f", resize_debug_img);

		std::string robot_frame;
		nh.param<std::string>("robot_frame", robot_frame, "base_link");

		std::string camera_frame;
		nh.param<std::string>("camera_frame", camera_frame, "camera");

		// Computing the transformation between coordinate systems
		tf::StampedTransform Tcr;
		try
		{
			// Waiting for the static transform
			//            tf_listener.waitForTransform(robot_frame, camera_frame, ros::Time(0), ros::Duration(5.0));
			//            tf_listener.lookupTransform(robot_frame, camera_frame, ros::Time(0), Trc);
			tf_listener.waitForTransform(camera_frame, robot_frame, ros::Time(0), ros::Duration(5.0));
			tf_listener.lookupTransform(camera_frame, robot_frame, ros::Time(0), Tcr);

			// Getting and transforming the corresponding components
			tf::Matrix3x3 rot = Tcr.getBasis();
			cv::Mat_<double> Rcr = cv::Mat::zeros(3, 3, CV_64F);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Rcr(i, j) = rot[i][j];
				}
			}

			tf::Vector3 trans = Tcr.getOrigin();
			double x = trans.getX();
			double y = trans.getY();
			double z = trans.getZ();
			cv::Mat_<double> Skew_tcr = cv::Mat::zeros(3, 3, CV_64F);
			Skew_tcr(0, 0) = 0.0; Skew_tcr(0, 1) = -z;   Skew_tcr(0, 2) = y;
			Skew_tcr(1, 0) = z;   Skew_tcr(1, 1) = 0.0;  Skew_tcr(1, 2) = -x;
			Skew_tcr(2, 0) = -y;  Skew_tcr(2, 1) = x;    Skew_tcr(2, 2) = 0.0;

			// Computing the jacobian
			//            cv::Mat_<double> Rcr = Rrc.inv();
			Rcr.copyTo(J.rowRange(0, 3).colRange(0, 3));
			Rcr.copyTo(J.rowRange(3, 6).colRange(3, 6));
			cv::Mat_<double> SR = Skew_tcr * Rcr;
			SR.copyTo(J.rowRange(0, 3).colRange(3, 6));
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
    	twist_debug_pub = nh.advertise<auv_msgs::BodyVelocityReq>("twist_debug", 1);

		// Publishing IBVS info
		ibvsinfo_pub = nh.advertise<merbots_ibvs::IBVSInfo>("ibvs_info", 1);

		// Dynamic reconfigure
		server.setCallback(boost::bind(&IBVS::dynreconf_cb, this, _1, _2));

		// Target callback
		target_sub = it.subscribe("target", 0, &IBVS::target_cb, this);

		// Distance subscriber
		dist_sub = nh.subscribe("dist", 0, &IBVS::dist_cb, this);

		// Image Subscriber for debugging purposes
		img_sub = it.subscribe("image", 0, &IBVS::image_cb, this);

		// Debug image publisher
		debug_img_pub = it.advertise("debug_img", 1);

		// Service to restart IBVS
    restart_srv = nh.advertiseService("restart", &IBVS::restart, this);

    // Rotate target TODO
    rotate_cw_srv = nh.advertiseService("rotate_clockwise", &IBVS::rotateClockwise, this);
		rotate_ccw_srv = nh.advertiseService("rotate_counterclockwise", &IBVS::rotateCounterclockwise, this);

		// Control timer
		control_timer = nh.createTimer(ros::Duration(1.0 / control_freq), &IBVS::ctrltimer_cb, this);
	}

	bool IBVS::restart(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
	{
		mutex_roi.lock();
		init_roi = false;
		mutex_roi.unlock();

		mutex_target.lock();
		init_target = false;
		mutex_target.unlock();

		mutex_img.lock();
		if (!last_img.empty())
		{
			last_img = cv::Mat::zeros(last_img.rows, last_img.cols, CV_64F);
		}
		mutex_img.unlock();

		return true;
	}


  bool IBVS::rotateClockwise(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res)
  {
    mutex_target.lock();

    // Updating the current desired ROI
    int mid_w = static_cast<int>(width / 2.0);
    int mid_h = static_cast<int>(height / 2.0);
    int mid_w_roi = static_cast<int>(des_roi.height / 2.0);
    int mid_h_roi = static_cast<int>(des_roi.width / 2.0);

    des_roi.x = mid_w - mid_w_roi;
    des_roi.y = mid_h - mid_h_roi;
    des_roi.width = mid_w_roi*2;
    des_roi.height = mid_h_roi*2;

    // Updating the points
    des_pt_tl.x = mid_w - mid_w_roi;
    des_pt_tl.y = mid_h + mid_h_roi;
    des_pt_tr.x = mid_w - mid_w_roi;
    des_pt_tr.y = mid_h - mid_h_roi;
    des_pt_bl.x = mid_w + mid_w_roi;
    des_pt_bl.y = mid_h + mid_h_roi;
    des_pt_br.x = mid_w + mid_w_roi;
    des_pt_br.y = mid_h - mid_h_roi;

    mutex_target.unlock();
    return true;
  }

  bool IBVS::rotateCounterclockwise(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res)
  {
    mutex_target.lock();

    cv::Point2i temp(des_pt_br);
    des_pt_br = des_pt_bl;
    des_pt_bl = des_pt_tl;
    des_pt_tl = des_pt_tr;
    des_pt_tr = temp;

    // Updating the current desired ROI
    int mid_w = static_cast<int>(width / 2.0);
    int mid_h = static_cast<int>(height / 2.0);
    int mid_w_roi = static_cast<int>(des_roi.height / 2.0);
    int mid_h_roi = static_cast<int>(des_roi.width / 2.0);

    des_roi.x = mid_w - mid_w_roi;
    des_roi.y = mid_h - mid_h_roi;
    des_roi.width = mid_w_roi*2;
    des_roi.height = mid_h_roi*2;

    // Updating the points
    des_pt_tl.x = mid_w + mid_w_roi;
    des_pt_tl.y = mid_h - mid_h_roi;
    des_pt_tr.x = mid_w + mid_w_roi;
    des_pt_tr.y = mid_h + mid_h_roi;
    des_pt_bl.x = mid_w - mid_w_roi;
    des_pt_bl.y = mid_h - mid_h_roi;
    des_pt_br.x = mid_w - mid_w_roi;
    des_pt_br.y = mid_h + mid_h_roi;

    mutex_target.unlock();
    return true;
  }

	void IBVS::dist_cb(const sensor_msgs::RangeConstPtr& msg)
	{
		mutex_zdist.lock();

		// Receiving new distance
		z_dist = msg->range;

		mutex_zdist.unlock();
	}

	// Image callback
	void IBVS::image_cb(const sensor_msgs::ImageConstPtr& msg)
	{
		// Converting the image message to OpenCV format
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
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
	void IBVS::target_cb(const sensor_msgs::ImageConstPtr& msg)
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
    des_pt_br.x = mid_w + mid_w_roi;
    des_pt_br.y = mid_h + mid_h_roi;

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

	void IBVS::dynreconf_cb(merbots_ibvs::IBVSConfig& config, uint32_t level)
	{
		// Adapting the correspondent parameters dynamically
		pid_x.setKp(config.kp_x);
		pid_x.setKd(config.kd_x);
		pid_x.setKi(config.ki_x);
		pid_y.setKp(config.kp_y);
		pid_y.setKd(config.kd_y);
		pid_y.setKi(config.ki_y);
		pid_z.setKp(config.kp_z);
		pid_z.setKd(config.kd_z);
		pid_z.setKi(config.ki_z);
	}

	void IBVS::roi_cb(const merbots_tracking::TargetPointsConstPtr& roi_msg)
	{
		mutex_roi.lock();

		// Computing the feature points from the ROI
		last_pt_tl.x = static_cast<int>(roi_msg->point_tl.x);
		last_pt_tl.y = static_cast<int>(roi_msg->point_tl.y);
		last_pt_tr.x = static_cast<int>(roi_msg->point_tr.x);
		last_pt_tr.y = static_cast<int>(roi_msg->point_tr.y);
		last_pt_bl.x = static_cast<int>(roi_msg->point_bl.x);
		last_pt_bl.y = static_cast<int>(roi_msg->point_bl.y);
		last_pt_br.x = static_cast<int>(roi_msg->point_br.x);
		last_pt_br.y = static_cast<int>(roi_msg->point_br.y);

		// Updating the ROI
		last_roi.x = last_pt_tl.x;
		last_roi.y = last_pt_tl.y;
		last_roi.width = last_pt_tr.x - last_pt_tl.x;
		last_roi.height = last_pt_bl.y - last_pt_tl.y;

		// Updating the status
		last_status = roi_msg->status;

		// Assessing if it is a valid ROI
		last_roi_valid = roi_msg->exists_roi;

		if (!last_roi_valid)
		{
			init_roi = false;
		}
		else if (last_roi_valid && !init_roi)
		{
			init_roi = true;
		}

		mutex_roi.unlock();
	}

	void IBVS::ctrltimer_cb(const ros::TimerEvent& event)
	{
		// Getting current features and assessing if the control should be done
		int u1, v1;
		int u2, v2;
		int u3, v3;
		int u4, v4;
		cv::Rect roi;
		int status;
		bool in_roi = false;
		bool valid_roi = false;
		mutex_roi.lock();
		u1 = last_pt_tl.x;
		v1 = last_pt_tl.y;
		u2 = last_pt_tr.x;
		v2 = last_pt_tr.y;
		u3 = last_pt_bl.x;
		v3 = last_pt_bl.y;
		u4 = last_pt_br.x;
		v4 = last_pt_br.y;
		roi = last_roi;
		if (init_roi)
		{
			in_roi = true;
		}
		status = last_status;
		valid_roi = last_roi_valid;
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
			// Getting height of the vehicle
			double h;
			mutex_zdist.lock();
			h = z_dist;
			mutex_zdist.unlock();

			// Computing the distance of each point
			double z1, z2, z3;
			z1 = (h / cos_ang) * (1.0 - (tan_ang * (v1 - v0)) / (tan_ang * (v1 - v0) + fs));
			z2 = (h / cos_ang) * (1.0 - (tan_ang * (v2 - v0)) / (tan_ang * (v2 - v0) + fs));
			z3 = (h / cos_ang) * (1.0 - (tan_ang * (v3 - v0)) / (tan_ang * (v3 - v0) + fs));

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

			// --- Filling the error vector ---
			s(0,0) = u1 - u1_d;
			s(1,0) = v1 - v1_d;
			s(2,0) = u2 - u2_d;
			s(3,0) = v2 - v2_d;
			s(4,0) = u3 - u3_d;
			s(5,0) = v3 - v3_d;

			// --- Computing the motion command ---
			ros::Time curr_time = ros::Time::now();
			cv::Mat_<double> res_pidx;
			pid_x.getCommand(curr_time, s, res_pidx);
			cv::Mat_<double> res_pidy;
			pid_y.getCommand(curr_time, s, res_pidy);
			cv::Mat_<double> res_pidz;
			pid_z.getCommand(curr_time, s, res_pidz);

			cv::Mat_<double> vx = -(Lp_s_inv.row(0) * res_pidx);
			cv::Mat_<double> vy, wz;
			if (enable_vely)
			{
				vy = -(Lp_s_inv.row(1) * res_pidy);
				wz = -(Lp_s_inv.row(2) * res_pidz);
			}
			else
			{
				wz = -(Lp_s_inv.row(1) * res_pidz);
			}

			// --- Filling and publishing the corresponding message ---

			auv_msgs::BodyVelocityReqPtr curr_twist(new auv_msgs::BodyVelocityReq);
			curr_twist->header.stamp = ros::Time::now();
			curr_twist->header.frame_id = "ibvs";

			curr_twist->goal.requester = "ibvs";
			curr_twist->goal.id = 0;
			curr_twist->goal.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL; // FIXME

			// VX
			curr_twist->twist.linear.x = vx(0,0);
			curr_twist->disable_axis.x = 0;
			if (std::abs(curr_twist->twist.linear.x) > max_vx)
			{
				// Limiting velocity
				if (curr_twist->twist.linear.x > 0)
				{
					curr_twist->twist.linear.x = max_vx;
				}
				else
				{
					curr_twist->twist.linear.x = -max_vx;
				}
			}

			// VY
			if (enable_vely)
			{
				curr_twist->twist.linear.y = vy(0,0);
				curr_twist->disable_axis.y = 0;

				if (std::abs(curr_twist->twist.linear.y) > max_vy)
				{
					// Limiting velocity
					if (curr_twist->twist.linear.y > 0)
					{
						curr_twist->twist.linear.y = max_vy;
					}
					else
					{
						curr_twist->twist.linear.y = -max_vy;
					}
				}
			}
			else
			{
				curr_twist->twist.linear.y = 0.0;
				curr_twist->disable_axis.y = 1;
			}

			curr_twist->twist.linear.z = 0.0;
			curr_twist->disable_axis.z = 1;
			curr_twist->twist.angular.x = 0.0;
			curr_twist->disable_axis.roll = 1;
			curr_twist->twist.angular.y = 0.0;
			curr_twist->disable_axis.pitch = 1;

			// WZ
			curr_twist->twist.angular.z = wz(0,0);
			curr_twist->disable_axis.yaw = 0;
			if (std::abs(curr_twist->twist.angular.z) > max_wz)
			{
				// Limiting velocity
				if (curr_twist->twist.angular.z > 0)
				{
					curr_twist->twist.angular.z = max_wz;
				}
				else
				{
					curr_twist->twist.angular.z = -max_wz;
				}
			}

			twist_pub.publish(curr_twist);

      		if (twist_debug_pub.getNumSubscribers() > 0)
			{
				twist_debug_pub.publish(curr_twist);
			}

			// Showing debug image if needed
			if (debug_img_pub.getNumSubscribers() > 0 || debug)
			{
				cv::Mat img;
				mutex_img.lock();
				last_img.copyTo(img);
				mutex_img.unlock();

				// Lines
				cv::line(img, cv::Point(u1, v1), cv::Point(u1_d, v1_d), cv::Scalar(0, 255, 255), 3);
				cv::line(img, cv::Point(u2, v2), cv::Point(u2_d, v2_d), cv::Scalar(0, 255, 255), 3);
				cv::line(img, cv::Point(u3, v3), cv::Point(u3_d, v3_d), cv::Scalar(0, 255, 255), 3);

				// Printing the current rectangle
				cv::circle(img, cv::Point(u1_d, v1_d), 4, cv::Scalar(0, 255, 0), -1);
    		cv::circle(img, cv::Point(u2_d, v2_d), 4, cv::Scalar(0, 255, 0), -1);
    		cv::circle(img, cv::Point(u3_d, v3_d), 4, cv::Scalar(0, 255, 0), -1);
    		cv::line(img, cv::Point(u1, v1), cv::Point(u2, v2), cv::Scalar(0, 255, 0), 2);
    		cv::line(img, cv::Point(u2, v2), cv::Point(u4, v4), cv::Scalar(0, 255, 0), 2);
    		cv::line(img, cv::Point(u4, v4), cv::Point(u3, v3), cv::Scalar(0, 255, 0), 2);
    		cv::line(img, cv::Point(u3, v3), cv::Point(u1, v1), cv::Scalar(0, 255, 0), 2);

    		// Printing desired roi
				cv::rectangle(img, roi_d, cv::Scalar(0, 0, 255), 2);

				// Plotting the working mode in the image
				if (status == 0)
				{
					cv::putText(img, "D", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
				}
				else
				{
					cv::putText(img, "T", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
				}

				cv::resize(img, img, cv::Size(), resize_debug_img, resize_debug_img);

				if (debug_img_pub.getNumSubscribers() > 0)
				{
					sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
					debug_img_pub.publish(msg);
				}

				if (debug)
				{
					cv::imshow("IBVS", img);
					cv::waitKey(5);
				}
			}
		}

		// Publishing information about the visual servoing process
		merbots_ibvs::IBVSInfoPtr ibvsinfo_msg(new merbots_ibvs::IBVSInfo);
		ibvsinfo_msg->header.stamp = ros::Time::now();
		ibvsinfo_msg->target_found = valid_roi;
		if (valid_roi)
		{
			ibvsinfo_msg->error = SQ(u1 - u1_d) + SQ(v1 - v1_d) +
								  SQ(u2 - u2_d) + SQ(v2 - v2_d) +
								  SQ(u3 - u3_d) + SQ(v3 - v3_d);
		}
		else
		{
			ibvsinfo_msg->error = 0.0;
		}
		ibvsinfo_pub.publish(ibvsinfo_msg);
	}
}
