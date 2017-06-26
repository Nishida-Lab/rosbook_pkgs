#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/icp.h>

#include <cloud_exercise/cloud_common.h>

enum INDICES_CONVERTER
{
  INDEX_NOT_ROTATED = 0, INDEX_ROTATED = 1, INDEX_ALIGNED = 2
};

class CloudMatchingHandler
{
public:
    CloudMatchingHandler(ros::NodeHandle &nh) :
        transform_pub_(nh.advertise<geometry_msgs::Transform>("icp_transformation", 1))
    {
        loaders_.push_back(new CloudLoader(nh, "cloud_not_rotated", "not_rotated.pcd"));
        loaders_.push_back(new CloudLoader(nh, "cloud_rotated", "rotated.pcd"));
        loaders_.push_back(new CloudLoader(nh, "cloud_aligned"));
    }

    void operate()
    {
        for(std::vector<CloudLoader*>::iterator it = loaders_.begin(); it != loaders_.end(); ++it)
            (*it)->load();

        match();
        convertTransformEiganToROS();

        for(std::vector<CloudLoader*>::iterator it = loaders_.begin(); it != loaders_.end(); ++it)
            (*it)->convertPCLtoROS();
    }

    void publish()
    {
        // Publish pointcloud
        for(std::vector<CloudLoader*>::iterator it = loaders_.begin(); it != loaders_.end(); ++it)
            (*it)->publish();

        // Publish transformation in Translation and Quaternion
        transform_pub_.publish(transform_ros_);
    }

private:
    void match()
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        icp.setInputSource(loaders_[INDEX_NOT_ROTATED]->cloud_pcl_.makeShared());
        icp.setInputTarget(loaders_[INDEX_ROTATED]->cloud_pcl_.makeShared());

        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon (1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(loaders_[INDEX_ALIGNED]->cloud_pcl_);
        transform_eigen_ = icp.getFinalTransformation().cast<double>();
    }

    void convertTransformEiganToROS()
    {
        // convert Matrix4d to Affine3d for ROS conversion
        Eigen::Affine3d transform_affine_eigen;
        transform_affine_eigen = transform_eigen_;

        tf::transformEigenToMsg(transform_affine_eigen, transform_ros_);
        ROS_INFO_STREAM(std::endl << transform_ros_);
    }

    std::vector<CloudLoader*> loaders_;
    ros::Publisher transform_pub_;
    Eigen::Matrix4d transform_eigen_;
    geometry_msgs::Transform transform_ros_;
};


main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_matcher");
    ros::NodeHandle nh;

    CloudMatchingHandler handler(nh);
    handler.operate();

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        handler.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
