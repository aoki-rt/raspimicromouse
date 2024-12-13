#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include <string>
#include <chrono>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define ADDR_TORQUE_ENABLE 64
#define ADDR_LED 65
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 1000000
#define DEVICE_NAME "/dev/ttyUSB0"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

using std::placeholders::_1;
using namespace std::chrono_literals;


int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;


class SweeperNode : public rclcpp::Node{
  public:
    SweeperNode() : Node("sweeper_node"){
      publisher_angle = this->create_publisher<std_msgs::msg::Int16>("current_arm_angle",10);
      subscription_angle = this->create_subscription<std_msgs::msg::Int16>(
        "target_arm_angle",10,std::bind(&SweeperNode::angle_callback,this,_1));
      subscription_shot = this->create_subscription<std_msgs::msg::Int8>(
        "shot",10,std::bind(&SweeperNode::shot_callback,this,_1));
    }

  void angle_callback(const std_msgs::msg::Int16::SharedPtr angle_msg){
    RCLCPP_INFO(this->get_logger(),"target_angle %d",angle_msg->data);
  }
  
  void shot_callback(const std_msgs::msg::Int8::SharedPtr shot_msg){
    RCLCPP_INFO(this->get_logger(),"shot %d",shot_msg->data);
  }

  private:

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_angle;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_angle;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_shot;
};

int main(int argc, char **argv){

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("sweeper_node"), "Failed to open the port!");
    return -1;
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("sweeper_node"), "Failed to set the baudrate!");
    return -1;
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );
 

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }


  rclcpp::init(argc,argv);
  auto node = std::make_shared<SweeperNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
    
