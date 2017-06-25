#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>

#include <cloud_exercise/operation_handler.h>

class CloudDownsamper : public CloudOperator
{
public:
    CloudDownsamper(ros::NodeHandle &nh) :
        cloud_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_downsampled", 1))
    {}

    void operate()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled_pcl;

        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud_input_pcl.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(cloud_downsampled_pcl);

        pcl::toROSMsg(cloud_downsampled_pcl, cloud_filterd_ros_);
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
    ros::init(argc, argv, "cloud_downsampler");
    ros::NodeHandle nh;

    CloudOperationHandler handler(nh, new CloudDownsamper(nh), "cloud_filtered");

    ros::spin();

    return 0;
}
