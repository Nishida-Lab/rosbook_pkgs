#ifndef PLUGINLIB_ARRAYUTIL_MIN_H_
#define PLUGINLIB_ARRAYUTIL_MIN_H_

#include <pluginlib_arrayutil/arrayutil_base.h>

namespace arrayutil_plugins_min 
{
  class Min : public arrayutil_base::ArrayUtil
  {
    public:
      Min();
      void loadArray();
      double operate();
  };
}

#endif
