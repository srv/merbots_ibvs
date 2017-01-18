#include <merbots_ibvs/ibvs/IBVS.h>

int main(int argc, char** argv)
{
	// ROS
	ros::init(argc, argv, "ibvs");
	ROS_INFO("MERBOTS Image-Based Visual Servoing");

	ros::NodeHandle nh("~");
	merbots_ibvs::IBVS ibvs(nh);
	ros::spin();
	return 0;
}
