#include "vl53l1x/vl53l1x_node.hpp"

namespace vl53l1x {
Vl53l1xNode::Vl53l1xNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("vl53l1x_node", options) {
  // --- 1. Declare and get params ---
  RCLCPP_INFO(this->get_logger(), "Starting VL53L1X Ros node...");

  i2c_bus_param_ =
      this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");

  tca_manager_node_name_param_ = this->declare_parameter<std::string>(
      "tca_manager_node_name", "tca9548a_manager");

  int tca_address_int = this->declare_parameter<int>("tca_address", 0x70);
  tca_address_param_ = static_cast<uint8_t>(tca_address_int);

  int vl53l1x_channel_int = this->declare_parameter<int>("vl53l1x_channel", 0);
  vl53l1x_channel_param_ = static_cast<uint8_t>(vl53l1x_channel_int);

  timeout_ = this->declare_parameter<int>("timeout", 500);
  timing_budget_ =
      this->declare_parameter<int>("timing_budget", 50000);
  freq_ = this->declare_parameter<double>("publish_rate", 10.0);

  // --- 2. Create the service client to the central TCA node ---
  std::string service_name = "/" + tca_manager_node_name_param_ + "/select_channel";
  select_channel_client_ = this->create_client<tca9548a::srv::SelectChannel>(service_name);

  // --- 3. Wait for the service to be available before proceeding with initialization ---
  if (!select_channel_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_FATAL(
        this->get_logger(),
        "TCA select_channel service not available. Node cannot initialize.");
    return;
  }

  // --- 4. Perform the INITIAL channel selection and sensor setup ---
  auto request = std::make_shared<tca9548a::srv::SelectChannel::Request>();
  request->address = tca_address_param_;
  request->channel = vl53l1x_channel_param_;
  auto result_future = select_channel_client_->async_send_request(request);

  // Wait for the response to complete. This is safe in the constructor.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_FATAL(
        this->get_logger(),
        "Failed to call TCA select_channel service for initial setup.");
    return;
  }

  auto result = result_future.get();
  if (!result->success) {
    RCLCPP_FATAL(this->get_logger(),
                 "TCA failed to select channel %d during initialization. Node "
                 "cannot proceed.",
                 vl53l1x_channel_param_);
    return;
  }

  // Now that the channel is selected, we can initialize the sensor driver
  sensor = Vl53l1x(i2c_bus_param_);

  if (!sensor.init()) {
    RCLCPP_FATAL(this->get_logger(),
                 "Failed to initialize VL53L1X sensor on channel %d",
                 vl53l1x_channel_param_);
    return;
  }

  auto deselect_request =
      std::make_shared<tca9548a::srv::SelectChannel::Request>();
  deselect_request->channel = 0x00; // A special channel 0x00 to deselect all
  deselect_request->address = tca_address_param_;
  select_channel_client_->async_send_request(deselect_request);

  // --- 5. Set up ROS 2 communication ---
  publisher_ = this->create_publisher<sensor_msgs::msg::Range>("distance", 10);

  auto publish_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / freq_));
  timer_ = this->create_wall_timer(
      publish_period, std::bind(&Vl53l1xNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Node initialized. Publishing at %.2f Hz.",
              freq_);
}

void Vl53l1xNode::timer_callback() {
  auto message = sensor_msgs::msg::Range();

  if (!select_channel_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(
        this->get_logger(),
        "TCA select_channel service not available. Cannot read sensor.");
    message.range = std::numeric_limits<float>::infinity();
    publisher_->publish(message);
    return;
  }

  auto request = std::make_shared<tca9548a::srv::SelectChannel::Request>();
  request->channel = vl53l1x_channel_param_;
    request->address = tca_address_param_;
  auto result_future = select_channel_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call TCA select_channel service.");
    message.range = std::numeric_limits<float>::infinity();
    publisher_->publish(message);
    return;
  }

  auto result = result_future.get();
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(),
                 "TCA failed to select channel %d. Cannot read sensor.",
                 vl53l1x_channel_param_);
    message.range = std::numeric_limits<float>::infinity();
    publisher_->publish(message);
    return;
  }

  uint16_t reading = sensor.read_range();
  if (!sensor.timeoutOccurred()) {
    message.header.stamp = this->now();
    message.header.frame_id = "vl53l1x_link";
    message.radiation_type = sensor_msgs::msg::Range::INFRARED;
    message.field_of_view = 0.4712;
    message.min_range = 0.04;
    message.max_range = 1.00;
    message.range = reading / 1000.0f;
  } else {
    reading = 0;
    RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
    message.header.stamp = this->now();
    message.header.frame_id = "vl53l1x_link";
    message.radiation_type = sensor_msgs::msg::Range::INFRARED;
    message.field_of_view = 0.4712;
    message.min_range = 0.04;
    message.max_range = 1.00;
    message.range = std::numeric_limits<float>::infinity();
    RCLCPP_WARN(this->get_logger(),
                "Failed to read from sensor. Publishing invalid data.");
  }

  auto deselect_request =
      std::make_shared<tca9548a::srv::SelectChannel::Request>();
  deselect_request->channel = 0x00; // A special channel 0x00 to deselect all
  deselect_request->address = tca_address_param_;
  select_channel_client_->async_send_request(deselect_request);

  publisher_->publish(message);
}
} // namespace vl53l1x

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 C++ client library
    rclcpp::init(argc, argv);

    // Create a shared pointer to your node
    auto node = std::make_shared<vl53l1x::Vl53l1xNode>();

    // Start the node's event loop. This will keep the node alive
    // and process all callbacks (like the timer callback).
    rclcpp::spin(node);

    // Shutdown the ROS 2 C++ client library when the node is finished
    rclcpp::shutdown();

    return 0;
}