#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"  // Corrected header

using namespace std::chrono_literals;

class Test_Pub_MultiUint8 : public rclcpp::Node {
public:
  Test_Pub_MultiUint8() : Node("test_pub_MultiUint8"), count(0) {
    RCLCPP_INFO(this->get_logger(), "create publisher(UInt8_Multi_Array)");

    // Corrected namespace usage and header
    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/stm32/recv", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&Test_Pub_MultiUint8::on_timer, this));
  }

private:
void on_timer() {
    auto msg = std_msgs::msg::ByteMultiArray();

    msg.data = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    RCLCPP_INFO(get_logger(), "Publishing: [%d, %d, %d]",msg.data[0], msg.data[1], msg.data[2]);
    publisher_->publish(msg);
}


  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Test_Pub_MultiUint8>());

  rclcpp::shutdown();
  return 0;
}
