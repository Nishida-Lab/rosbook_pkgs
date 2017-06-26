#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <cloud_exercise/cloud_common.h>

class CloudPlanarSegmenter : public CloudOperator
{
public:
    CloudPlanarSegmenter(ros::NodeHandle &nh) :
        cloud_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1)),
        cloud_without_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_without_segmented", 1)),
        coefficients_pub_(nh.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1))
    {}

    void operate()
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        segment(inliers);
        extract(inliers);
    }

    void publish()
    {
        cloud_segmented_pub_.publish(cloud_segmented_ros_);
        cloud_without_segmented_pub_.publish(cloud_without_segmented_ros_);
        coefficients_pub_.publish(coefficients_ros_);
    }

private:
    void segment(pcl::PointIndices::Ptr inliers)
    {
        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl_);

        pcl::ModelCoefficients coefficients_pcl;
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;

        // Create the segmentation object
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.01);
        segmentation.setInputCloud(cloud_input_pcl_.makeShared());
        segmentation.segment(*inliers, coefficients_pcl);

        pcl_conversions::fromPCL(coefficients_pcl, coefficients_ros_);
    }

    void extract(pcl::PointIndices::Ptr inliers)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_without_segmented_pcl;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Create the filtering object
        extract.setInputCloud(cloud_input_pcl_.makeShared());
        extract.setIndices(inliers);
        // Extract the plannar inlier pointcloud from indices
        extract.setNegative(false);
        extract.filter(cloud_segmented_pcl);
        // Remove the plannar inlier pointcloud, extract the rest
        extract.setNegative(true);
        extract.filter(cloud_without_segmented_pcl);

        // Convert to ROS msg
        pcl::toROSMsg(cloud_segmented_pcl, cloud_segmented_ros_);
        pcl::toROSMsg(cloud_without_segmented_pcl, cloud_without_segmented_ros_);
    }

protected:
    ros::Publisher cloud_segmented_pub_;
    ros::Publisher cloud_without_segmented_pub_;
    ros::Publisher coefficients_pub_;
    sensor_msgs::PointCloud2 cloud_segmented_ros_;
    sensor_msgs::PointCloud2 cloud_without_segmented_ros_;
    pcl_msgs::ModelCoefficients coefficients_ros_;
    pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl_;
};


main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_planar_segmenter");
    ros::NodeHandle nh;

    CloudOperationHandler handler(nh, new CloudPlanarSegmenter(nh), "cloud_downsampled");

    ros::spin();

    return 0;
}
