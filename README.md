# scene_completion
ROS node for segmenting and completing a scene

This node is constantly listening to a filtered pointcloud topic, and keeps a running list of the 5 most recent clouds.  
It also has an action server that when triggered will concatenate the 5 recent clouds, run them through euclidean cluster extraction, 
and then pass each cluster through a helper node to convert each segmented cloud to a mesh.  

The simplest object completion node that this can run with is:
https://github.com/CURG/object_completion
