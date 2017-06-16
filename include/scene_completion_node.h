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
#include <scene_completion/SceneCompletionConfig.h>
#include <scene_completion/CompleteSceneAction.h>
#include <scene_completion/CompletePartialCloudAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <memory>


  class SceneCompletionNode {
  public:
    SceneCompletionNode();
    ~SceneCompletionNode();

    //action server callback
    void executeCB(const scene_completion::CompleteSceneGoalConstPtr & goal);

  private:

    //dyanmic reconfigure callback
    void reconfigure_cb(scene_completion::SceneCompletionConfig &config, uint32_t level);

    //new pointcloud received
    void pcl_cloud_cb(const sensor_msgs::PointCloud2ConstPtr &points_msg);

    // ROS Structures
    std::shared_ptr<ros::NodeHandle> nh_;

    //subscriber to filtered pointcloud
    ros::Subscriber cloud_sub_;
    ros::Publisher objects_pub_;
    ros::Publisher markers_pub_;
    ros::Publisher foreground_points_pub_;
    //tf::TransformListener listener_;


    std::shared_ptr<actionlib::SimpleActionServer<scene_completion::CompleteSceneAction>> as_;
    std::shared_ptr< actionlib::SimpleActionClient<scene_completion::CompletePartialCloudAction> > client;
    scene_completion::CompleteSceneResult result;


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
    std::shared_ptr< dynamic_reconfigure::Server<scene_completion::SceneCompletionConfig> > reconfigure_server_;

    // Mutex for managing buffery synchronization
    boost::mutex buffer_mutex_;
    std::list<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > > clouds_;
    void point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_cluster,
                                         shape_msgs::Mesh &mesh, geometry_msgs::PoseStamped &pose_stamped );
  };

#endif // ifndef __SCENE_COMPLETION_NODE_H
