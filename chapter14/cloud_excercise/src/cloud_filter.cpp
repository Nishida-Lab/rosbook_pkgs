#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <cloud_exercise/operation_handler.h>

class CloudFilter : public CloudOperator
{
public:
    CloudFilter(ros::NodeHandle &nh) :
        cloud_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1))
    {}

    void operate()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;

        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(cloud_input_pcl.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered_pcl);

        pcl::toROSMsg(cloud_filtered_pcl, cloud_filterd_ros_);
    }

    void publish()
    {
        cloud_pub_.publish(cloud_filterd_ros_);
    }

protected:
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_filterd_ros_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_filter");
    ros::NodeHandle nh;

    CloudOperationHandler handler(nh, new CloudFilter(nh), "cloud_raw");

    ros::spin();

    return 0;
}
