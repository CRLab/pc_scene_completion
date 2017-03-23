#include <ros/ros.h>
#include <scene_completion_node.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

SceneCompletionNode::SceneCompletionNode(ros::NodeHandle nh) :
    nh_(nh),
    reconfigure_server_(nh),
    n_clouds_per_recognition(5),
    as_(nh_, "SceneCompletion", boost::bind(&SceneCompletionNode::executeCB, this, _1), false),
    client("object_completion", true)

{
    nh.getParam("filtered_cloud_topic", filtered_cloud_topic);// /filter_pc

    // Construct subscribers and publishers
    cloud_sub_ = nh.subscribe("/filtered_pc", 1, &SceneCompletionNode::pcl_cloud_cb, this);

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
    ec.setClusterTolerance (0.02); // 2cm  //TODO MAKE DYNAMIC  RECONFIGURE
    ec.setMinClusterSize (100);//TODO MAKE DYNAMIC  RECONFIGURE
    ec.setMaxClusterSize (25000000);//TODO MAKE DYNAMIC  RECONFIGURE
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_full);
    ec.extract (cluster_indices);



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
        result.meshes.push_back(point_cloud_to_mesh(cloud_cluster));


    }

    as_.setSucceeded(result);


}


shape_msgs::Mesh SceneCompletionNode::point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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



    return result->mesh;

//    // pointcloud -> mesh
//    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
//    tree1->setInputCloud (cloud);
//    ne.setInputCloud (cloud);
//    ne.setSearchMethod (tree1);
//    ne.setKSearch (20);
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    ne.compute (*normals);

//    // Concatenate the XYZ and normal fields
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

//    // Create search tree*
//    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
//    tree->setInputCloud (cloud_with_normals);



//    std::cout << "begin marching cubes reconstruction" << std::endl;

//    shape_msgs::Mesh mesh_msg;

//    pcl::io::savePCDFileASCII("/home/bo/before_marching_cubes_cloud.pcd", *cloud_with_normals);
//    return mesh_msg;

//    pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal> mc;
//    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
//    mc.setInputCloud (cloud_with_normals);
//    mc.setSearchMethod (tree);
//    mc.reconstruct (*triangles);



//    // add each vertices + triangles to result

//    for (int i =0 ; i < cloud->size(); i++) {
//        geometry_msgs::Point geom_p_msg;

//        geom_p_msg.x = cloud->at(i).x;

//        geom_p_msg.y = cloud->at(i).y;
//        geom_p_msg.z = cloud->at(i).z;

//        mesh_msg.vertices.push_back(geom_p_msg);

//    }

//    for (int i =0 ; i < triangles->polygons.size(); i++) {
//        shape_msgs::MeshTriangle t_msg;

//        pcl::Vertices pcl_vertices = triangles->polygons.at(i);

//        t_msg.vertex_indices[0] = pcl_vertices.vertices.at(0);
//        t_msg.vertex_indices[1] = pcl_vertices.vertices.at(1);
//        t_msg.vertex_indices[2] = pcl_vertices.vertices.at(2);

//        mesh_msg.triangles.push_back(t_msg);

//    }

//    return mesh_msg;

}
