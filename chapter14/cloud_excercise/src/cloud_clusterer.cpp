#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cloud_exercise/operation_handler.h>

class CloudClusterer : public CloudOperator
{
public:
    CloudClusterer(ros::NodeHandle &nh)
    {
        pcl_pubs_.resize(CLUSTER_NUM);
        cloud_clusters_ros_.resize(CLUSTER_NUM);

        for(std::vector<ros::Publisher>::iterator it = pcl_pubs_.begin(); it != pcl_pubs_.end(); ++it)
        {
          std::stringstream ss;
          int index = it - pcl_pubs_.begin();
          ss << "cloud_clustered" << index + 1;

          (*it) = nh.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);
        }
    }

    void operate()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::fromROSMsg(cloud_input_ros_, *cloud_input_pcl_ptr);

        clusterKdTree(cloud_input_pcl_ptr, cluster_indices);
        extractCluster(cloud_input_pcl_ptr, cluster_indices);
    }

    void publish()
    {
        for(std::vector<ros::Publisher>::iterator it = pcl_pubs_.begin(); it != pcl_pubs_.end(); ++it)
        {
            int index = it - pcl_pubs_.begin();
            it->publish(cloud_clusters_ros_[index]);
        }
    }

private:
    void clusterKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr, std::vector<pcl::PointIndices> &cluster_indices)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered_ptr);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered_ptr);
        ec.extract(cluster_indices);
    }

    void extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr, std::vector<pcl::PointIndices> cluster_indices)
    {
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered_ptr->points[*pit]);
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          sensor_msgs::PointCloud2 cloud_cluster_ros;
          pcl::toROSMsg(*cloud_cluster, cloud_cluster_ros);
          cloud_cluster_ros.header.frame_id = "base_link";

          int index = it - cluster_indices.begin();
          cloud_clusters_ros_[index] = cloud_cluster_ros;
        }
    }

protected:
    static const int CLUSTER_NUM = 3;
    ros::NodeHandle nh_;
    std::vector<ros::Publisher> pcl_pubs_;
    std::vector<sensor_msgs::PointCloud2> cloud_clusters_ros_;
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_filterd_ros_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_clusterer");
    ros::NodeHandle nh;

    CloudOperationHandler handler(nh, new CloudClusterer(nh), "cloud_without_segmented");

    ros::spin();

    return 0;
}
