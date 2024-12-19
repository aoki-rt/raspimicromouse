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
#include "raspimicromouse_msgs/msg/run.hpp"
#include "raspimicromouse_msgs/msg/len.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include <chrono>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;


class SearchNode : public rclcpp::Node{
  public:
    SearchNode() : Node("search_node"){
      publisher_done = this->create_publisher<std_msgs::msg::Int8>("exec_done",10);
      publisher_run = this->create_publisher<raspimicromouse_msgs::msg::Run>("run",10);
      publisher_arm_angle = this->create_publisher<std_msgs::msg::Int16>("target_arm_angle",10);
      publisher_shot = this->create_publisher<std_msgs::msg::Int8>("shot",10);
      bool_client = this->create_client<std_srvs::srv::SetBool>("motor_power");
      subscription_mode = this->create_subscription<std_msgs::msg::Int8>(
        "modeexec",10,std::bind(&SearchNode::mode_exec_callback,this,_1));
      subscription_result = this->create_subscription<raspimicromouse_msgs::msg::Len>(
        "run_result",10,std::bind(&SearchNode::result_callback,this,_1));
      subscription_arm_angle = this->create_subscription<std_msgs::msg::Int16>(
        "current_arm_angle",10,std::bind(&SearchNode::current_angle_callback,this,_1));
    }
  private:
    char run_result;
    float run_lengthed;

    void StraightCheck(int num){
      raspimicromouse_msgs::msg::Run mode;

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      auto result = bool_client->async_send_request(request);
      auto return_code = rclcpp::spin_until_future_complete(this->get_node_base_interface(),result);
      if(return_code == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO_STREAM(this->get_logger(),"result:" << result.get()->message);
      }      

      rclcpp::sleep_for(1000ms);
      mode.length=0.180*num;
      mode.mode=0;
      publisher_run->publish(mode);
      while((0.180*num > run_lengthed)||(!run_result));

      rclcpp::sleep_for(300ms);
      request->data = false;
      result = bool_client->async_send_request(request);
    };

    void result_callback(const raspimicromouse_msgs::msg::Len::SharedPtr len_msg){
      run_lengthed = len_msg->length;
      run_result = len_msg->finish;
    }
    
    void current_angle_callback(const std_msgs::msg::Int16::SharedPtr angle_msg){
      RCLCPP_INFO(this->get_logger(),"current_arm: %d",angle_msg->data);
    }

    void mode_exec_callback(const std_msgs::msg::Int8::SharedPtr mode_msg){
      raspimicromouse_msgs::msg::Run mode;
      std_msgs::msg::Int8 done;
      std_msgs::msg::Int16 target;
      RCLCPP_INFO(this->get_logger(),"subscribe_mode: %d",mode_msg->data);
      switch(mode_msg->data){
      case 0://reset
        done.data=1;
        mode.length=0.0;
        mode.mode=0;
        publisher_run->publish(mode);
        publisher_done->publish(done); 
        break;
      case 1://left hand
        break;
      case 2://adachi-method
        break;
      case 3://fast
        break;
      case 4://adachi-method sura
        break;
      case 5://fast sura
        break;
      case 6://stright check
        StraightCheck(4);
        break;
      case 7://right turn check
        mode.length=0.07379;
        mode.mode=1;
        publisher_run->publish(mode);
        break;
      case 8://left turn check
        break;
      case 9://right sura check
        break;
      case 10://left sura check
        break;
      case 11://ball hold
        break;
      case 12://ball shot
        break;
      case 13://arm upper
        target.data=3000;
        publisher_arm_angle->publish(target);
        break;
      case 14://arm mid
        target.data=2000;
        publisher_arm_angle->publish(target);
        break;
      case 15://arm lower
        target.data=1000;
        publisher_arm_angle->publish(target);
        break;
      }
    }
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_done;
  rclcpp::Publisher<raspimicromouse_msgs::msg::Run>::SharedPtr publisher_run;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_arm_angle;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_shot;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_mode;
  rclcpp::Subscription<raspimicromouse_msgs::msg::Len>::SharedPtr subscription_result;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_arm_angle;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bool_client;
};

int main(int argc, char **argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<SearchNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

