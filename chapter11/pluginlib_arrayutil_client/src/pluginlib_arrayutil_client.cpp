#include <pluginlib_arrayutil_client/pluginlib_arrayutil_client.h>

namespace pluginlib_arrayutil_client
{
  void PluginlibArrayutilClient::run()
  {
    loadAllPlugins();
    operateAllPlugins();
  }

  void PluginlibArrayutilClient::loadAllPlugins()
  {
    // create plugin instances
    for(int i = 0; i < PLUGIN_SIZE; ++i)
      pluginInstances_.push_back(arrayutil_loader_.createInstance(PLUGIN_NAME[i]));

    // set array values for each plugin
    for (std::vector<LoaderPtr>::iterator it = pluginInstances_.begin(); it != pluginInstances_.end(); ++it)
      (*it)->setArray(vec_);
  }

  void PluginlibArrayutilClient::operateAllPlugins()
  {
    for (std::vector<LoaderPtr>::iterator it = pluginInstances_.begin(); it != pluginInstances_.end(); ++it)
    {
      // compute index just for grabbing the current loaded plugin name
      int index = std::distance(pluginInstances_.begin(), it);

      // execute the operation according to the current loaded plugin (Sum, Ave, Min, Max)
      double result = (*it)->operate();
      ROS_INFO_STREAM(PLUGIN_NAME[index] << ": " << std::fixed << std::setprecision(2) << std::setw(5) << result);
    }
  }
}

