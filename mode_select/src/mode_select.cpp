
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
#include "raspimouse_msgs/msg/leds.hpp"
#include "raspimouse_msgs/msg/switches.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include <string>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;


class ModeSelectNode : public rclcpp::Node{
  public:
    ModeSelectNode() : Node("mode_select_node"){
      publisher_ = this->create_publisher<raspimouse_msgs::msg::Leds>("leds",10);
      publisher_exec = this->create_publisher<std_msgs::msg::Int8>("modeexec",10);
      publisher_buzzer = this->create_publisher<std_msgs::msg::Int16>("buzzer",10);
      subscription_ = this->create_subscription<raspimouse_msgs::msg::Switches>(
        "switches",10,std::bind(&ModeSelectNode::switch_callback,this,_1));
      subscription_done = this->create_subscription<std_msgs::msg::Int8>(
        "exec_done",10,std::bind(&ModeSelectNode::exec_done_callback,this,_1));

      mode=1;
      mode_running=0;
      ModeSelectNode::led_set();
    };
  private:
    signed char mode;
    char mode_running;
    void led_set(void){
      raspimouse_msgs::msg::Leds led_msg;
      led_msg.led0=mode&0x1;
      led_msg.led1=(mode>>1)&0x01;
      led_msg.led2=(mode>>2)&0x01;
      led_msg.led3=(mode>>3)&0x01;
      publisher_->publish(led_msg);
    }

    void switch_callback(const raspimouse_msgs::msg::Switches::SharedPtr sw_msg) {
//      RCLCPP_INFO(this->get_logger(),"switch: %d %d %d ",sw_msg->switch0,sw_msg->switch1,sw_msg->switch2);
      std_msgs::msg::Int8 data;
      std_msgs::msg::Int16 hz;
      if(mode_running){
        if((sw_msg->switch0)||(sw_msg->switch1)||(sw_msg->switch2)){
          hz.data=1000;
          publisher_buzzer->publish(hz);
          rclcpp::sleep_for(300ms);
          data.data=0; 
          publisher_exec->publish(data);
        }
      }else{
        if(sw_msg->switch0){
          hz.data=2000; 
          mode--;
          if(mode<1)mode=1;
          publisher_buzzer->publish(hz); 
          rclcpp::sleep_for(30ms);
          hz.data=0;
          publisher_buzzer->publish(hz);
        }
        if(sw_msg->switch2){
          hz.data=3000;
          mode++;
          if(mode>15)mode=15;
          publisher_buzzer->publish(hz); 
          rclcpp::sleep_for(30ms);
          hz.data=0;
          publisher_buzzer->publish(hz);
        }
        ModeSelectNode::led_set();
        if(sw_msg->switch1){
          mode_running=1;
          data.data=mode;
          publisher_exec->publish(data);
          hz.data=2000;
          publisher_buzzer->publish(hz);          
          rclcpp::sleep_for(80ms);
          hz.data=3000;
          publisher_buzzer->publish(hz);
          rclcpp::sleep_for(80ms);
          hz.data=0;
          publisher_buzzer->publish(hz);
        }
      }
      rclcpp::sleep_for(33ms);
      hz.data=0;
      publisher_buzzer->publish(hz);
    };
    void exec_done_callback(const std_msgs::msg::Int8::SharedPtr done_msg){
     RCLCPP_INFO(this->get_logger(),"exec dont:%d",done_msg->data);
     mode_running=0;
     mode=1;
    }
    rclcpp::Publisher<raspimouse_msgs::msg::Leds>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_exec;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_buzzer;
    rclcpp::Subscription<raspimouse_msgs::msg::Switches>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_done;
};

int main(int argc, char **argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ModeSelectNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
    
