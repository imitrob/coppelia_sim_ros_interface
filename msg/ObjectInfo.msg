string name # obj name
float32[3] forces # x,y,z [N]
float32[3] torques # x,y,z [Nm]
geometry_msgs/Pose pose
float32[6] bounding_box # min x, max x, min y, max y, min z, max z positions.
bool collision # check with any object in the scene
