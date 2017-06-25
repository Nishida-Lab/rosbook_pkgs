#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/icp.h>

enum INDEX_CONVERTER
{
  INDEX_NOT_ROTATED = 0, INDEX_ROTATED = 1, INDEX_ALIGNED = 2
};

class CloudReader
{
public:
    CloudReader(){}

    CloudReader(ros::NodeHandle &nh, const std::string &pub_topic_name, const std::string &pcd_file_name = "") :
        nh_(nh),
        cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1))
    {
        if(pcd_file_name == "")
            return;

        file_path_ = ros::package::getPath("cloud_exercise") + "/data/" + pcd_file_name;
    }

    void read()
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

class CloudMatchingHandler
{
public:
    CloudMatchingHandler(ros::NodeHandle &nh)
    {
        readers_.push_back(new CloudReader(nh, "cloud_not_rotated", "not_rotated.pcd"));
        readers_.push_back(new CloudReader(nh, "cloud_rotated", "rotated.pcd"));
        readers_.push_back(new CloudReader(nh, "cloud_aligned"));
    }

    void operate()
    {
        for(std::vector<CloudReader*>::iterator it = readers_.begin(); it != readers_.end(); ++it)
            (*it)->read();

        match();

        for(std::vector<CloudReader*>::iterator it = readers_.begin(); it != readers_.end(); ++it)
            (*it)->convertPCLtoROS();
    }

    void publish()
    {
        for(std::vector<CloudReader*>::iterator it = readers_.begin(); it != readers_.end(); ++it)
            (*it)->publish();
    }

private:
    void match()
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(readers_[INDEX_NOT_ROTATED]->cloud_pcl_.makeShared());
        icp.setInputTarget(readers_[INDEX_ROTATED]->cloud_pcl_.makeShared());

        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon (1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(readers_[INDEX_ALIGNED]->cloud_pcl_);
    }

    std::vector<CloudReader*> readers_;
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
