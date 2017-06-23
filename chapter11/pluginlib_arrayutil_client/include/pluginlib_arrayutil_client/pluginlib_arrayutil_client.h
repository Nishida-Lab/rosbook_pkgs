#ifndef PLUGINLIB_ARRAYUTIL_CLIENT_PLUGINLIB_ARRAYUTIL_CLIENT_H_
#define PLUGINLIB_ARRAYUTIL_CLIENT_PLUGINLIB_ARRAYUTIL_CLIENT_H_

#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib_arrayutil/arrayutil_base.h>

namespace pluginlib_arrayutil_client
{
  const double ARRAY[] = {1.7, 2.3, 3.2, 4.8};
  const int ARRAY_SIZE = 4;
  const std::string PLUGIN_NAME[] = {
    "pluginlib_arrayutil/Sum", "pluginlib_arrayutil/Ave",
    "pluginlib_arrayutil/Min", "pluginlib_arrayutil/Max"
  };
  const int PLUGIN_SIZE = 4;

  class PluginlibArrayutilClient
  {
    typedef boost::shared_ptr<arrayutil_base::ArrayUtil> LoaderPtr;

    public:
    PluginlibArrayutilClient() :
      arrayutil_loader_("pluginlib_arrayutil", "arrayutil_base::ArrayUtil"),
      vec_(ARRAY, ARRAY + ARRAY_SIZE)
    { };

    void run();

    private:

    void loadAllPlugins();
    void operateAllPlugins();

    std::vector<double> vec_;
    pluginlib::ClassLoader<arrayutil_base::ArrayUtil> arrayutil_loader_;
    std::vector<LoaderPtr> pluginInstances_;
  };
}

#endif
