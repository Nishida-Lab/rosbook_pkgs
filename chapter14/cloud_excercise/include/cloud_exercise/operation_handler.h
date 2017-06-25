#include <sensor_msgs/PointCloud2.h>

class CloudOperator
{
public:
    void setInputCloud(const sensor_msgs::PointCloud2 &cloud_input_ros)
    {
        cloud_input_ros_ = cloud_input_ros;
    }
    virtual void operate() = 0;
    virtual void publish() = 0;

protected:
    sensor_msgs::PointCloud2 cloud_input_ros_;
};

class CloudOperationHandler
{
public:
    CloudOperationHandler(ros::NodeHandle &nh,
                             CloudOperator *cloud_operator,
                             const std::string &sub_topic_name) :
        cloud_operator_(cloud_operator),
        cloud_sub_(nh.subscribe(sub_topic_name, 10, &CloudOperationHandler::operateCB, this))
    {}

    void operateCB(const sensor_msgs::PointCloud2 &cloud_input_ros)
    {
        cloud_operator_->setInputCloud(cloud_input_ros);
        cloud_operator_->operate();
        cloud_operator_->publish();
    }

protected:
    CloudOperator *cloud_operator_;
    ros::Subscriber cloud_sub_;
};
