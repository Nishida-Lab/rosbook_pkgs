#ifndef PLUGINLIB_ARRAYUTIL_MAX_H_
#define PLUGINLIB_ARRAYUTIL_MAX_H_

#include <pluginlib_arrayutil/arrayutil_base.h>

namespace arrayutil_plugins_max 
{
  class Max : public arrayutil_base::ArrayUtil
  {
    public:
      Max();
      void loadArray();
      double operate();
  };
}

#endif
