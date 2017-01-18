#ifndef IBVS_NODELET
#define IBVS_NODELET

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <merbots_ibvs/ibvs/IBVS.h>

namespace merbots_ibvs
{

  class IBVSNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit ();

  private:
    IBVS* ibvs_;
  };

}

#endif
