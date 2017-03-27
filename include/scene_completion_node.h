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
#include <objrec_ros_integration/FindObjects.h>
#include <scene_completion/SceneCompletionConfig.h>
#include <scene_completion/CompleteSceneAction.h>
#include <scene_completion/CompletePartialCloudAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


  class SceneCompletionNode {
  public:
    SceneCompletionNode(ros::NodeHandle nh = ros::NodeHandle("~"));
    ~SceneCompletionNode();

    //action server callback
    bool recognizeObjectsCB(objrec_ros_integration::FindObjects::Request &req, objrec_ros_integration::FindObjects::Response &res);


    void executeCB(const scene_completion::CompleteSceneGoalConstPtr & goal);

  private:

    //dyanmic reconfigure callback
    void reconfigure_cb(scene_completion::SceneCompletionConfig &config, uint32_t level);

    //new pointcloud received
    void pcl_cloud_cb(const sensor_msgs::PointCloud2ConstPtr &points_msg);

    // ROS Structures
    ros::NodeHandle &nh_;
    //subscriber to filtered pointcloud
    ros::Subscriber cloud_sub_;
    ros::Publisher objects_pub_;
    ros::Publisher markers_pub_;
    ros::Publisher foreground_points_pub_;
    //tf::TransformListener listener_;

    actionlib::SimpleActionServer<scene_completion::CompleteSceneAction> as_;
    actionlib::SimpleActionClient<scene_completion::CompletePartialCloudAction> client;

    std::string filtered_cloud_topic;
    int n_clouds_per_recognition;
    int partial_mesh_count;

    // ROS Dynamic Reconfigure
    dynamic_reconfigure::Server<scene_completion::SceneCompletionConfig> reconfigure_server_;


    // Mutex for managing buffery synchronization
    boost::mutex buffer_mutex_;
    std::list<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > > clouds_;
    void point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_cluster,
                                         shape_msgs::Mesh &mesh, geometry_msgs::PoseStamped &pose_stamped );
  };

#endif // ifndef __SCENE_COMPLETION_NODE_H
