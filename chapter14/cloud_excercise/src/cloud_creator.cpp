#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudCreator
{
public:
    CloudCreator(ros::NodeHandle &nh, const std::string &topic_name, const std::string &file_name)
    {
        nh_ = nh;

        cloud_pcl_.width  = 1000;
        cloud_pcl_.height = 1;
        cloud_pcl_.points.resize(cloud_pcl_.width * cloud_pcl_.height);

        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

        file_path_ = ros::package::getPath("cloud_exercise") + "/data/" + file_name;
    }

    void operate()
    {
        cloud_pcl_.width  = 5000;
        cloud_pcl_.height = 1;
        cloud_pcl_.points.resize(cloud_pcl_.width * cloud_pcl_.height);

        create();

        pcl::toROSMsg(cloud_pcl_, cloud_ros_);
        cloud_ros_.header.frame_id = "base_link";

        pcl::io::savePCDFile(file_path_, cloud_pcl_);
    }

    void publish()
    {
        cloud_pub_.publish(cloud_ros_);
    }

    virtual void create() = 0;

protected:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl_;
    sensor_msgs::PointCloud2 cloud_ros_;
    std::string file_path_;
};


class NotRotatedCloudCreator : public CloudCreator
{
public:
    NotRotatedCloudCreator(ros::NodeHandle &nh, const std::string &topic_name, const std::string &file_name)
        : CloudCreator(nh, topic_name, file_name){}

    virtual void create()
    {
        for(size_t i = 0; i < cloud_pcl_.points.size(); ++i)
        {
            cloud_pcl_.points[i].x = 3.0 * rand () / (RAND_MAX + 1.0f);
            cloud_pcl_.points[i].y = 1.0 * rand () / (RAND_MAX + 1.0f);
            cloud_pcl_.points[i].z = 0.5 * rand () / (RAND_MAX + 1.0f);
        }
    }
};


class RotatedCloudCreator : public CloudCreator
{
public:
    RotatedCloudCreator(ros::NodeHandle &nh, const std::string &topic_name, const std::string &file_name)
        : CloudCreator(nh, topic_name, file_name){}

    virtual void create()
    {
        double theta = M_PI / 4;

        for(size_t i = 0; i < cloud_pcl_.points.size(); ++i)
        {
            double x_tmp = 3.0 * rand () / (RAND_MAX + 1.0f);
            double y_tmp = 1.0 * rand () / (RAND_MAX + 1.0f);

            cloud_pcl_.points[i].x = cos(theta) * x_tmp - sin(theta) * y_tmp;
            cloud_pcl_.points[i].y = sin(theta) * x_tmp + cos(theta) * y_tmp + 2.0f;
            cloud_pcl_.points[i].z = 0.5 * rand () / (RAND_MAX + 1.0f) ;
        }
    }
};

class CloudCreatingHandler
{
public:
    CloudCreatingHandler(ros::NodeHandle &nh)
    {
      creators_.push_back(new NotRotatedCloudCreator(nh, "cloud_not_rotated", "not_rotated.pcd"));
      creators_.push_back(new RotatedCloudCreator(nh, "cloud_rotated", "rotated.pcd"));
    }

    void operate()
    {
        for(std::vector<CloudCreator*>::iterator it = creators_.begin(); it != creators_.end(); ++it)
            (*it)->operate();
    }

    void publish()
    {
        for(std::vector<CloudCreator*>::iterator it = creators_.begin(); it != creators_.end(); ++it)
            (*it)->publish();
    }

private:
    std::vector<CloudCreator*> creators_;
};


main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_creator");
    ros::NodeHandle nh;

    CloudCreatingHandler handler(nh);
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
