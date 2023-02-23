#include "sdpo_ros_odom/OdomWhROS.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_ros_odom");

  sdpo_ros_odom::OdomWhROS odom_ros;
  ros::spin();

  return 0;
}
