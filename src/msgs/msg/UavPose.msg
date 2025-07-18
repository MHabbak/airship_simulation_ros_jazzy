# UAV Pose message - created to replace missing uav_msgs/uav_pose
std_msgs/Header header

geometry_msgs/Point position         # Position in world frame
geometry_msgs/Quaternion orientation # Orientation in world frame
geometry_msgs/Vector3 velocity       # Velocity in body frame
geometry_msgs/Vector3 angular_velocity # Angular velocity in body frame