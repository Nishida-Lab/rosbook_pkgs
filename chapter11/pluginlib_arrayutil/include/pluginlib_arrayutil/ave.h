#ifndef PLUGINLIB_ARRAYUTIL_AVE_H_
#define PLUGINLIB_ARRAYUTIL_AVE_H_

#include <pluginlib_arrayutil/arrayutil_base.h>

namespace arrayutil_plugins_ave
{
  class Ave : public arrayutil_base::ArrayUtil
  {
    public:
      Ave();
      double operate();
  };
}

#endif
