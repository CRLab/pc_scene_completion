#ifndef __SCENE_COMPLETION_NODE_H
#define __SCENE_COMPLETION_NODE_H

#include <list>
#include <queue>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pc_scene_completion/SceneCompletionConfig.h>
#include <pc_pipeline_msgs/CompleteSceneAction.h>
#include <pc_pipeline_msgs/CompletePartialCloudAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


  class SceneCompletionNode {
  public:
    SceneCompletionNode(ros::NodeHandle nh = ros::NodeHandle(""));
    ~SceneCompletionNode();

    //action server callback
    void executeCB(const pc_pipeline_msgs::CompleteSceneGoalConstPtr & goal);

  private:

    //dyanmic reconfigure callback
    void reconfigure_cb(pc_scene_completion::SceneCompletionConfig &config, uint32_t level);

    //new pointcloud received
    void pcl_cloud_cb(const sensor_msgs::PointCloud2ConstPtr &points_msg);

    // ROS Structures
    ros::NodeHandle &nh_;
    //subscriber to filtered pointcloud
    ros::Subscriber cloud_sub_;

    actionlib::SimpleActionServer<pc_pipeline_msgs::CompleteSceneAction> as_;
    
    actionlib::SimpleActionClient<pc_pipeline_msgs::CompletePartialCloudAction> depth_cnn_client;
    actionlib::SimpleActionClient<pc_pipeline_msgs::CompletePartialCloudAction> depth_tactile_cnn_client;
    actionlib::SimpleActionClient<pc_pipeline_msgs::CompletePartialCloudAction> partial_client;
      
    //topic in which we listen for filtered pointclouds from
    //this is grabbed from the ros param server.
    std::string filtered_cloud_topic;

    //counter to keep track of how many objects we have detected so that we can
    //give them each a unique name "mesh_" + str(partial_mesh_count++)
    int partial_mesh_count;

    int n_clouds_per_recognition;
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    std::string world_frame;
    std::string camera_frame;

    // ROS Dynamic Reconfigure
    dynamic_reconfigure::Server<pc_scene_completion::SceneCompletionConfig> reconfigure_server_;

    // Mutex for managing buffery synchronization
    boost::mutex buffer_mutex_;
    std::list<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > > clouds_queue_;
    void point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                             Eigen::Matrix4f transformEigen,
			     std::string object_completion_topic,
                             shape_msgs::Mesh &meshMsg,
                             geometry_msgs::PoseStamped  &poseStampedMsg,
                             sensor_msgs::PointCloud2 &partialCloudMsg);
  };

#endif // ifndef __SCENE_COMPLETION_NODE_H
