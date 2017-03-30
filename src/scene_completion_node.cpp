#include <ros/ros.h>
#include <scene_completion_node.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

SceneCompletionNode::SceneCompletionNode(ros::NodeHandle nh) :
    nh_(nh),
    reconfigure_server_(nh),
    as_(nh_, "SceneCompletion", boost::bind(&SceneCompletionNode::executeCB, this, _1), false),
    client("object_completion", true),
    partial_mesh_count(0)

{
    nh.getParam("filtered_cloud_topic", filtered_cloud_topic);// /filter_pc

    // Set up dynamic reconfigure
    reconfigure_server_.setCallback(boost::bind(&SceneCompletionNode::reconfigure_cb, this, _1, _2));

    // Construct subscribers and publishers
    cloud_sub_ = nh.subscribe("/filtered_pc", 1, &SceneCompletionNode::pcl_cloud_cb, this);
    //cluster_tolerance = scene_completion::SceneCompletionConfig.n_clouds_per_recognition

    //publish object clusters
    foreground_points_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("foreground_points",10);

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
    cluster_tolerance = config.cluster_tolerance;
    min_cluster_size = config.min_cluster_size;
    max_cluster_size = config.max_cluster_size;

    world_frame = config.world_frame; // std::string("/world"); //TODO MAKE DYNAMIC  RECONFIGURE
    camera_frame = config.camera_frame; // std::string("/head_camera_depth_optical_frame"); //TODO MAKE DYNAMIC  RECONFIGURE


}


void SceneCompletionNode::executeCB(const scene_completion::CompleteSceneGoalConstPtr & goal)
{

    scene_completion::CompleteSceneResult result;


    //this is the merged set of captured pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZRGB>());

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
    ec.setClusterTolerance (cluster_tolerance); // 2cm  
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_full);
    ec.extract (cluster_indices);

//    tf::TransformListener listener;
//    tf::StampedTransform transform;

//    std::string world_frame= "/world"; //TODO MAKE DYNAMIC  RECONFIGURE
//    std::string camera_frame= "/world"; //TODO MAKE DYNAMIC  RECONFIGURE

//    try{
//      listener.lookupTransform(world_frame.c_str(), camera_frame.c_str(),
//                               ros::Time(0), transform);
//    }
//    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
//      ros::Duration(1.0).sleep();
//    }

//    ROS_DEBUG("Waiting for world Transform");
//    ROS_DEBUG("Looking up Transform");
//    ros::Time now = ros::Time::now();
//    tf_listener->waitForTransform (output_frame_id, frame_id, now, ros::Duration(2.0));
//    tf_listener->lookupTransform (output_frame_id, frame_id, now, transform);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        //this is the set of points corresponding to the visible region of a single object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_full->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        //TODO MAKE DYNAMIC  RECONFIGURE
        //if we are running shape completion, then we
        //This is a simple partial view to mesh
        //RUN each cloud_cluster through marching cubes
        //pointcloud -> mesh
        geometry_msgs::PoseStamped pose_stamped;
        shape_msgs::Mesh mesh;

        point_cloud_to_mesh(cloud_cluster, mesh, pose_stamped);

        result.meshes.push_back(mesh);
        result.poses.push_back(pose_stamped);


    }

    as_.setSucceeded(result);


}


void SceneCompletionNode::point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                          shape_msgs::Mesh &mesh, geometry_msgs::PoseStamped  &pose_stamped ) {
    // TODO RUN cluster through marching cubes



    client.waitForServer();
    scene_completion::CompletePartialCloudGoal goal;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::PCLPointCloud2 cloud_pc2;
    pcl::toPCLPointCloud2(*cloud, cloud_pc2);

    pcl_conversions::fromPCL(cloud_pc2, cloud_msg);

    goal.partial_cloud = cloud_msg;


    client.sendGoalAndWait(goal);
    scene_completion::CompletePartialCloudResultConstPtr result = client.getResult();

    //We have a mesh with an points in the camera frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_vertices_in_camera_frame(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i =0; i < result->mesh.vertices.size(); i++)
    {
        geometry_msgs::Point p = result->mesh.vertices.at(i);
        pcl::PointXYZRGB pcl_point; // DO I need to do new for every point, or is += on a pointcloud a copy constructor
        pcl_point.x = p.x;
        pcl_point.y = p.y;
        pcl_point.z = p.z;
        partial_vertices_in_camera_frame->points.push_back(pcl_point);
    }

    //now we have a pointcloud in camera frame
    //lets put it in world frame, so that we can find the lowest point in the z world direction
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_vertices_in_world_frame(new pcl::PointCloud<pcl::PointXYZRGB>());

    // std::string world_frame= std::string("/world"); //TODO MAKE DYNAMIC  RECONFIGURE
    // std::string camera_frame= std::string("/head_camera_depth_optical_frame"); //TODO MAKE DYNAMIC  RECONFIGURE
    tf::TransformListener tf_listener;

    partial_vertices_in_camera_frame->header.frame_id = camera_frame.c_str();
    ros::Time now = ros::Time::now();
    tf_listener.waitForTransform (world_frame, camera_frame, now, ros::Duration(2.0));

    pcl_ros::transformPointCloud(world_frame,
                                 *partial_vertices_in_camera_frame,
                                 *partial_vertices_in_world_frame,
                                 tf_listener);


    // Now lets find the centroid in the world frame of reference
    //and make a new pointcloud with that as the origin
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_in_object_frame (new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
    pcl::compute3DCentroid (*partial_vertices_in_world_frame, centroid); centroid.w () = 1.f;

    std::cout << "centroid.x(): " << centroid.x() << std::endl;
    std::cout << "centroid.y(): " << centroid.y() << std::endl;
    std::cout << "centroid.z(): " << centroid.z() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::iterator p;
    int num_pts = 0;
    for (p = partial_vertices_in_world_frame->points.begin(); p < partial_vertices_in_world_frame->points.end(); p++)
    {
        pcl::PointXYZRGB *point = new pcl::PointXYZRGB;

        point->x = p->x - centroid.x();
        point->y = p->y - centroid.y();
        point->z = p->z;
        point->r = p->r;
        point->g = p->g;
        point->b = p->b;
        point->a = p->a;

        partial_in_object_frame->points.push_back(*point);
        num_pts ++;
    }
    partial_in_object_frame->width = num_pts;
    partial_in_object_frame->height = 1;

    //now we need to repackage the pointcloud as a mesh
        for (int i =0 ; i < partial_in_object_frame->size(); i++) {
            geometry_msgs::Point geom_p_msg;

            geom_p_msg.x = partial_in_object_frame->at(i).x;
            geom_p_msg.y = partial_in_object_frame->at(i).y;
            geom_p_msg.z = partial_in_object_frame->at(i).z;

            mesh.vertices.push_back(geom_p_msg);

//            result->mesh.vertices.at(i).x = cloud->at(i).x;
//            result->mesh.vertices.at(i).y = cloud->at(i).y;
//            result->mesh.vertices.at(i).z = cloud->at(i).z;
        }

        mesh.triangles = result->mesh.triangles;

    //now we have a mesh with the origin at the center of the base in the world frame of reference
    //lets get the transform from world to mesh center
     pose_stamped.header.frame_id = world_frame.c_str();
     pose_stamped.pose.orientation.w = 1.0;
     pose_stamped.pose.position.x = centroid.x();
     pose_stamped.pose.position.y = centroid.y();
     pose_stamped.pose.position.z = 0; // Todo This will be slightly off due to the foam, feel free to find min point in mesh and use that

}
