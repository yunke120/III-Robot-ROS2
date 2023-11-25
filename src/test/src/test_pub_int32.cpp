

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class Test_Sub : public rclcpp::Node{
public:
  Test_Sub():Node("test_sub"), count(0)
  {
      RCLCPP_INFO(this->get_logger(), "create publisher(Int32)");

      publisher_ = this->create_publisher<std_msgs::msg::Int32>("/microros/int32", 10);

      timer_ = this->create_wall_timer(1s, std::bind(&Test_Sub::on_timer, this));
  }
private:
  void on_timer()
  {
    auto msg = std_msgs::msg::Int32();
    msg.data = count++;
    RCLCPP_INFO(this->get_logger(), "publish msg: %d", msg.data);
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Test_Sub>());

  rclcpp::shutdown();
  return 0;
}
