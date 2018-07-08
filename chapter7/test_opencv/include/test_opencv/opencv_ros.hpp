#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "ImageWindow";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_ori_;
    image_transport::Publisher image_pub_drawn_;

public:
    ImageConverter(ros::NodeHandle& nh);
    ~ImageConverter();

    void publishReadImage(void);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};
