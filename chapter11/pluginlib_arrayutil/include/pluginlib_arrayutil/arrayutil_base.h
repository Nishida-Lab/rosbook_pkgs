#ifndef PLUGINLIB_ARRAYUTIL_ARRAYUTIL_BASE_H_
#define PLUGINLIB_ARRAYUTIL_ARRAYUTIL_BASE_H_

#include <vector>

namespace arrayutil_base 
{
  class ArrayUtil
  {
    public:
      void setArray(const std::vector<double> array);
      void setArray(const double *array, const int size);
      virtual double operate() = 0;
      virtual ~ArrayUtil(){}

    protected:
      ArrayUtil(){}

    //private:
      std::vector<double> vec_;
  };
};

#endif
