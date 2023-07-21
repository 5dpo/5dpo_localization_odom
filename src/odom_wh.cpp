#include "sdpo_localization_odom/OdomWhROS.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_localization_odom");

  sdpo_localization_odom::OdomWhROS odom_ros;
  ros::spin();

  return 0;
}
