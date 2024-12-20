// Copyright 2024 RT Corporation
//
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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"

using std::placeholders::_1;

float linear_x, angular_z;

class JoypadNode : public rclcpp::Node{
  public:
    JoypadNode() : Node("joypad_node"){
      publisher_angle = this->create_publisher<std_msgs::msg::Int16>("target_arm_angle",10);
      publisher_twist = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      publisher_shot = this->create_publisher<std_msgs::msg::Int8>("shot",10);
      publisher_strength = this->create_publisher<std_msgs::msg::Int8>("shot_strength",10);
      subscription_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",10,std::bind(&JoypadNode::joy_callback,this,_1));
    }
    geometry_msgs::msg::Twist twist_msg;
    std_msgs::msg::Int16 angle_data;
    std_msgs::msg::Int8 shot_data;
    std_msgs::msg::Int8 strength_data;


  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr data){
      if(data->axes[7] !=0) {
        linear_x = 0.150 * data->axes[7]/2;
      }else if ((data->axes[1] > 0.000001) || (data->axes[1] < -0.000001)) {
        linear_x = data->axes[1] * 0.500;
      } else {
        linear_x = 0;
      }
      if (data->axes[6] != 0) {
        angular_z = 3 * data->axes[6]/2;
      } else if ((data->axes[3] > 0.00001) || (data->axes[3] < -0.00001)) {
        angular_z = data->axes[3] * 5;
      } else {
        angular_z = 0;
      }
      
      twist_msg.linear.x = linear_x;
      twist_msg.angular.z = angular_z;
      publisher_twist->publish(twist_msg);

      if(data->buttons[2]==1){//upper
        angle_data.data=1000;
        publisher_angle->publish(angle_data);
      }

      if(data->buttons[3]==1){//mid
        angle_data.data=1670;
        publisher_angle->publish(angle_data);
      }

      if(data->buttons[0]==1){//under
        angle_data.data=2200;
        publisher_angle->publish(angle_data);
      }

      strength_data.data=(int)(((data->axes[2]*-1)+1)*50);
      publisher_strength->publish(strength_data); 

      if(data->buttons[1]==1){//shot
        shot_data.data=1;
      }else{
        shot_data.data=0;
      }
      publisher_shot->publish(shot_data);

  }

  private:
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_angle;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_shot;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_strength;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JoypadNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

