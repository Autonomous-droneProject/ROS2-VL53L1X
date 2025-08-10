#include "tca9548a/GenericI2CManager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tca9548a::GenericI2CManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}