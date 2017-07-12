#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

class CloudLoader
{
public:
    CloudLoader(){}

    CloudLoader(ros::NodeHandle &nh, const std::string &pub_topic_name, const std::string &pcd_file_name = "") :
        nh_(nh),
        cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1))
    {
        if(pcd_file_name == "")
            return;

        file_path_ = ros::package::getPath("cloud_exercise") + "/data/" + pcd_file_name;
    }

    void load()
    {
        if(file_path_ == "")
            return;

        pcl::io::loadPCDFile(file_path_, cloud_pcl_);
    }

    void convertPCLtoROS()
    {
        pcl::toROSMsg(cloud_pcl_, cloud_ros_);
        cloud_ros_.header.frame_id = "base_link";
    }

    void publish()
    {
        cloud_pub_.publish(cloud_ros_);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_pcl_;

private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_ros_;
    std::string file_path_;
};


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
