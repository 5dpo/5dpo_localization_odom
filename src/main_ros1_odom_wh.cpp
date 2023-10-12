#include "sdpo_localization_odom/OdomWhROS1.h"

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "sdpo_localization_odom");

  sdpo_localization_odom::OdomWhROS1 node;

  ros::spin();

  ros::shutdown();

  return 0;

}
