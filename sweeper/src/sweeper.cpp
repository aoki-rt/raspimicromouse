// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include <string>
#include <chrono>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "raspimouse_msgs/msg/switches.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <iostream>
#include <boost/asio.hpp>


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
using std::string;
using std::cout;
using std::endl;

int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;



class SweeperNode : public rclcpp::Node{
  public:
    SweeperNode(const std::string& portname) : Node("sweeper_node"), _io(), _port(_io){
      publisher_angle = this->create_publisher<std_msgs::msg::Int16>("current_arm_angle",10);
      subscription_angle = this->create_subscription<std_msgs::msg::Int16>(
        "target_arm_angle",10,std::bind(&SweeperNode::angle_callback,this,_1));
      subscription_shot = this->create_subscription<std_msgs::msg::Int8>(
        "shot",10,std::bind(&SweeperNode::shot_callback,this,_1));
      subscription_strength = this->create_subscription<std_msgs::msg::Int8>(
        "shot_strength",10,std::bind(&SweeperNode::strength_callback,this,_1));
      subscription_sw = this->create_subscription<raspimouse_msgs::msg::Switches>(
        "switches",10,std::bind(&SweeperNode::switch_callback,this,_1));
      client_ = this->create_client<std_srvs::srv::SetBool>("motor_power");
      timer_ = this->create_wall_timer(10ms,std::bind(&SweeperNode::timer_callback,this));
      _port.open(portname);
    }

  void angle_callback(const std_msgs::msg::Int16::SharedPtr angle_msg){
    RCLCPP_INFO(this->get_logger(),"target_angle %d",angle_msg->data);
    angle_value=angle_msg->data;

    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler,
      BROADCAST_ID,
      ADDR_GOAL_POSITION,
      angle_msg->data,
      &dxl_error
    );
  }
  
  void shot_callback(const std_msgs::msg::Int8::SharedPtr shot_msg){
    std::string str; 
    if(shot_msg->data==1){
      RCLCPP_INFO(this->get_logger(),"shot %d",shot_msg->data);
      if(angle_value <2100){ 
        dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler,
          BROADCAST_ID,
          ADDR_GOAL_POSITION,
          2200,
          &dxl_error
        );
        rclcpp::sleep_for(1000ms);
      }
      str = std::to_string(shot_strength);
      boost::asio::write(_port,boost::asio::buffer(str)); 
    }
  }

  void strength_callback(const std_msgs::msg::Int8::SharedPtr strength_msg){
    shot_strength=strength_msg->data;
  }


  void switch_callback(const raspimouse_msgs::msg::Switches::SharedPtr sw_msg){
   auto request = std::make_shared<std_srvs::srv::SetBool::Request>();  

   if(sw_msg->switch1){
      request->data=true;
      client_->async_send_request(request,std::bind(&SweeperNode::callback_response_,this,_1));
    }
    if(sw_msg->switch2){
      request->data=false;
      client_->async_send_request(request,std::bind(&SweeperNode::callback_response_,this,_1));
    }
  }

  void callback_response_(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
  }

  void timer_callback(){//100Hz interupt
    char reply[100];

    //string msg ="Hello , Serail!";
    //write(serial,buffer(msg));

    boost::asio::read(_port,boost::asio::buffer(reply,100));
    cout << "Receiced: " << reply << endl;

  }

  private:
  boost::asio::io_service _io;
  boost::asio::serial_port _port;
  char shot_strength;
  short angle_value;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_angle;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_angle;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_shot;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_strength;
  rclcpp::Subscription<raspimouse_msgs::msg::Switches>::SharedPtr subscription_sw;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
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
  auto node = std::make_shared<SweeperNode>("/dev/ttyACM0");

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
    
