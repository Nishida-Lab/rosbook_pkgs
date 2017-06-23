#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "pluginlib_arrayutil/arrayutil_base.h"

const double array[] = {1.7, 2.3, 3.2, 4.8};
const int ARRAY_SIZE = 4;
const double ANSWER_SUM = 12.0;
const double ANSWER_AVE = 3.0;
const double ANSWER_MIN = 1.7;
const double ANSWER_MAX = 4.8;
const std::vector<double> vec(array, array + ARRAY_SIZE);

// TEST CASES
TEST(PluginlibArrayutilSubTest, testSum)
{
  pluginlib::ClassLoader<arrayutil_base::ArrayUtil> arrayutil_loader("pluginlib_arrayutil", "arrayutil_base::ArrayUtil");
  boost::shared_ptr<arrayutil_base::ArrayUtil> sum = arrayutil_loader.createInstance("pluginlib_arrayutil/Sum");

  sum->setArray(vec);
  double result = sum->operate();

  EXPECT_EQ(result, ANSWER_SUM);
}

TEST(PluginlibArrayutilSubTest, testAve)
{
  pluginlib::ClassLoader<arrayutil_base::ArrayUtil> arrayutil_loader("pluginlib_arrayutil", "arrayutil_base::ArrayUtil");
  boost::shared_ptr<arrayutil_base::ArrayUtil> ave = arrayutil_loader.createInstance("pluginlib_arrayutil/Ave");

  ave->setArray(vec);
  double result = ave->operate();

  EXPECT_EQ(result, ANSWER_AVE);
}

TEST(PluginlibArrayutilSubTest, testMin)
{
  pluginlib::ClassLoader<arrayutil_base::ArrayUtil> arrayutil_loader("pluginlib_arrayutil", "arrayutil_base::ArrayUtil");
  boost::shared_ptr<arrayutil_base::ArrayUtil> min = arrayutil_loader.createInstance("pluginlib_arrayutil/Min");

  min->setArray(vec);
  double result = min->operate();

  EXPECT_EQ(result, ANSWER_MIN);
}

TEST(PluginlibArrayutilSubTest, testMax)
{
  pluginlib::ClassLoader<arrayutil_base::ArrayUtil> arrayutil_loader("pluginlib_arrayutil", "arrayutil_base::ArrayUtil");
  boost::shared_ptr<arrayutil_base::ArrayUtil> max = arrayutil_loader.createInstance("pluginlib_arrayutil/Max");

  max->setArray(vec);
  double result = max->operate();

  EXPECT_EQ(result, ANSWER_MAX);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pluginlib_arrayutil_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return ret;
}
