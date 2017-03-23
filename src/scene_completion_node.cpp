#include <ros/ros.h>
#include <scene_completion_node.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

SceneCompletionNode::SceneCompletionNode(ros::NodeHandle nh) :
    nh_(nh),
    reconfigure_server_(nh),
    n_clouds_per_recognition(5),
    as_(nh_, "SceneCompletion", boost::bind(&SceneCompletionNode::executeCB, this, _1), false)
{
    nh.getParam("filtered_cloud_topic", filtered_cloud_topic);// /filter_pc

    // Construct subscribers and publishers
    cloud_sub_ = nh.subscribe(filtered_cloud_topic.c_str(), 1, &SceneCompletionNode::pcl_cloud_cb, this);

    //publish object clusters
    foreground_points_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("foreground_points",10);

    // Set up dynamic reconfigure
    reconfigure_server_.setCallback(boost::bind(&SceneCompletionNode::reconfigure_cb, this, _1, _2));

    as_.start();

    ROS_INFO("ready to find objects");
    ROS_INFO_STREAM("Constructed SceneCompletionNode.");

}


void SceneCompletionNode::pcl_cloud_cb(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
    // Lock the buffer mutex while we're capturing a new point cloud
    boost::mutex::scoped_lock buffer_lock(buffer_mutex_);

    // Convert to PCL cloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*points_msg, *cloud);

    // Store the cloud
    clouds_.push_back(cloud);

    // Increment the cloud index
    while(clouds_.size() > (unsigned)n_clouds_per_recognition) {
        clouds_.pop_front();
    }
}


//update variables from config server
void SceneCompletionNode::reconfigure_cb(scene_completion::SceneCompletionConfig &config, uint32_t level)
{
    //change any member vars here
    n_clouds_per_recognition = config.n_clouds_per_recognition;
}


void SceneCompletionNode::executeCB(const scene_completion::CompleteSceneGoalConstPtr & goal)
{
    //this is the merged set of captured pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZRGB>());
    //these are the clusters of partial objects
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters;

     //We need to wait until n_clouds_per_recognition clouds have been received
     ros::Rate warn_rate(1.0);
     if(clouds_.empty()) {
         ROS_WARN("Pointcloud buffer is empty!");
         warn_rate.sleep();
    }

     // grab mutex so noone else touches cloud list
    boost::mutex::scoped_lock buffer_lock(buffer_mutex_);

    //merge all clouds together
    for(std::list<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > >::const_iterator it = clouds_.begin();
                    it != clouds_.end();
                    ++it)
            {
                *cloud_full += *(*it);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //extract clusters from cloud
    //////////////////////////////////////////////////////////////////////////////////////
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_full);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm  //TODO MAKE DYNAMIC  RECONFIGURE
    ec.setMinClusterSize (100);//TODO MAKE DYNAMIC  RECONFIGURE
    ec.setMaxClusterSize (25000000);//TODO MAKE DYNAMIC  RECONFIGURE
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_full);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_full->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cloudClusters.push_back(cloud_cluster);


    }

    ;

}
