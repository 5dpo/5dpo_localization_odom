#include "sdpo_localization_odom/OdomWhROS.h"

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_localization_odom::OdomWhROS>());

  rclcpp::shutdown();

  return 0;

}
