#include "test_opencv/opencv_ros.hpp"

int main(int argc , char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    ImageConverter ic(nh);

    ros::Rate looprate (5);   // read image at 5Hz
    while(ros::ok())
    {
        ic.publishReadImage();
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
