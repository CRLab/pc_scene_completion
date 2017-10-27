#include <ros/ros.h>
#include <scene_completion_node.h>


int main(int argc, char** argv) {

  ros::init(argc,argv,"scene_completion");
  
  SceneCompletionNode *node = new SceneCompletionNode();

  ros::Rate r(10); //10hz
  while(ros::ok())
  {
    ros::spin();
  }

  return 0;
}
