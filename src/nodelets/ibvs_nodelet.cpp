#include <merbots_ibvs/nodelets/ibvs_nodelet.h>

PLUGINLIB_DECLARE_CLASS(
  merbots_ibvs,
  merbots_ibvs::IBVSNodelet,
  merbots_ibvs::IBVSNodelet,
  nodelet::Nodelet);

  namespace merbots_ibvs
  {

    void IBVSNodelet::onInit()
    {
      NODELET_INFO("Initializing IBVS Nodelet");
      ros::NodeHandle nh = getPrivateNodeHandle();
      ibvs_ = new IBVS(nh);
    }

  }
