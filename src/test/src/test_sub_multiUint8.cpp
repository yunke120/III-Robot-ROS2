

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp" 

using std::placeholders::_1;

class Test_Sub_MultiUint8 : public rclcpp::Node{
public:
  Test_Sub_MultiUint8():Node("test_sub_MultiUint8")
  {
    RCLCPP_INFO(this->get_logger(), "create subscriber(UInt8_Multi_Array)");
    subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>("/stm32/recv", 10,  std::bind(&Test_Sub_MultiUint8::on_cb ,this,std::placeholders::_1));
    
  }
private:

    void on_cb(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        // 获取 ByteMultiArray 中的数据数组
        const std::vector<uint8_t>& data = msg->data;

        // 打印数据数组中的每个元素
        RCLCPP_INFO(this->get_logger(), "-------------------------------");
        for (size_t i = 0; i < data.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "data[%zu]: %u", i, data[i]);
        }
    }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;

};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Test_Sub_MultiUint8>());

  rclcpp::shutdown();
  return 0;
}

