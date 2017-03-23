#include <ros/ros.h>
#include <scene_completion_node.h>

int main(int argc, char** argv) {

  ros::init(argc,argv,"objrec_node");

  SceneCompletionNode *node = new SceneCompletionNode();

  while(ros::ok())
  {
      ros::spin();
  }

  return 0;
}
