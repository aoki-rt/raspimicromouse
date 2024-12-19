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
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "raspimouse_msgs/msg/light_sensors.hpp"
#include "raspimicromouse_msgs/msg/run.hpp" 
#include "raspimicromouse_msgs/msg/len.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/convert.h"
#include "tf2/utils.h"

#include <chrono>
#include <string>
#include <memory>
#include <math.h>


geometry_msgs::msg::Twist twist_msg;
raspimicromouse_msgs::msg::Len run_result_msg;
using std::placeholders::_1;
using namespace std::chrono_literals;

#define MAX_SPEED 0.35f
#define MIN_SPEED 0.04f
#define ACCEL 1.5f
#define TREAD 0.094f
#define KP 0.002f


#define REF_R 1180
#define REF_L 570
#define SEN_R_TH 300
#define SEN_L_TH 200

typedef enum { front,right,left,rear} t_direction;
typedef enum { accel_state,decel_state} t_run_state;

class SpdctlNode : public rclcpp::Node{
  public:
    SpdctlNode() : Node("spdctl_node"){
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      publisher_len = this->create_publisher<raspimicromouse_msgs::msg::Len>("run_result",10);
      subscription_ = this->create_subscription<raspimicromouse_msgs::msg::Run>(
        "run",10,std::bind(&SpdctlNode::run_callback,this,_1));
      subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",10,std::bind(&SpdctlNode::odom_callback,this,_1));
      subscription_sensor = this->create_subscription<raspimouse_msgs::msg::LightSensors>(
        "light_sensors",10,std::bind(&SpdctlNode::sensor_callback,this,_1));
      timer_ = this->create_wall_timer(10ms,std::bind(&SpdctlNode::timer_callback,this));
      g_motor_move=0;
    }
  private:
    
    float g_length,g_lengthed;
    char run_result;
    float g_theta;
    t_direction g_direction_mode;
    float g_accel;   
    float g_speed;
    float g_finish_speed;
    t_run_state g_run_state;
    short g_sen_r_value,g_sen_l_value;
    char g_motor_move;
    float old_theta,new_theta;
    float old_x,old_y,new_x,new_y;


    void timer_callback(){//100Hz interrupt
      float omega;
      float tmp_speed_r,tmp_speed_l;
      short tmp_r,tmp_l;
      
      if(g_motor_move==0){
        twist_msg.linear.x=0.0;
        twist_msg.angular.z=0.0;
        run_result_msg.finish=1;
      }else{
        run_result_msg.finish=0;
        g_speed+=g_accel/100.0;
        if(g_speed > MAX_SPEED){
          g_speed = MAX_SPEED;
        }

        switch(g_direction_mode){
          case front:
            switch(g_run_state){
              case accel_state:
                if( (g_length - g_lengthed) < ((g_speed*g_speed)-(g_finish_speed*g_finish_speed))/(2*g_accel) ){
                  g_run_state = decel_state;
                  g_accel = -1*ACCEL;
                }
                break;
              case decel_state: 
                if(g_length < g_lengthed){
                  g_motor_move=0;
                  g_accel=0.0;
                  g_speed=0.0;
                }
                break;
            }
            if(g_sen_r_value > SEN_R_TH){
              tmp_r = REF_R - g_sen_r_value;
            }else{
              tmp_r = 0;
            }
            if(g_sen_l_value > SEN_L_TH){
              tmp_l = g_sen_l_value - REF_L;
            }else{
              tmp_l =0;
            }
            omega=KP*(tmp_r+tmp_l);

            tmp_speed_r = g_speed + omega;
            tmp_speed_l = g_speed - omega;

            if(tmp_speed_r< MIN_SPEED){
              tmp_speed_r = MIN_SPEED;
            }
            if(tmp_speed_l < MIN_SPEED){
              tmp_speed_l = MIN_SPEED;
            }
            twist_msg.linear.x=(tmp_speed_r+tmp_speed_l)/2;
            twist_msg.angular.z=(tmp_speed_r-tmp_speed_l)/TREAD; 
            break;
          case right:
            switch(g_run_state){
              case accel_state:
                if( (g_length - g_lengthed) < ((g_speed*g_speed)-(MIN_SPEED*MIN_SPEED))/(2*g_accel) ){
                  g_run_state = decel_state;
                  g_accel=-1*ACCEL;
                }
                break;
              case decel_state:
                if(g_length < g_lengthed){
                  g_motor_move=0;
                  g_accel=0.0;
                  g_speed=0.0;
//                  RCLCPP_INFO(this->get_logger(),"interrupt: %f %f",g_lengthed,g_length);
                }
                break;
            }
            twist_msg.linear.x=0.0;
            twist_msg.angular.z=-1*g_speed/TREAD/2.0;
            break;
          case left:
            switch(g_run_state){
              case accel_state:
                if( (g_length - g_lengthed) < ((g_speed*g_speed)-(MIN_SPEED*MIN_SPEED))/(2*g_accel) ){
                  g_run_state = decel_state;
                  g_accel=-1*ACCEL;
                }
                break;
              case decel_state:
                if(g_length < g_lengthed){
                  g_motor_move=0;
                  g_accel=0.0;
                  g_speed=0.0;
                }
                break;
            }
            twist_msg.linear.x=0.0;
            twist_msg.angular.z=g_speed/TREAD/2.0;
            break;
          default:
            g_speed=0.0;
            omega=0.0;
        }
      }
      run_result_msg.length=g_lengthed;
      publisher_->publish(twist_msg);
      publisher_len->publish(run_result_msg);
    }

    void run_callback(const raspimicromouse_msgs::msg::Run::SharedPtr run_msg)  {
      RCLCPP_INFO(this->get_logger(),"subscribe: %f %d ",run_msg->length,run_msg->mode);
      g_length = run_msg->length;
      switch(run_msg->mode){
        case 0:
          g_direction_mode=front;
          break;
        case 1:
          g_direction_mode=right;
          break;
        case 2:
          g_direction_mode=left;
          break;
      }
      g_lengthed=0.0;
      g_theta=0.0;
      g_run_state=accel_state;
      g_motor_move=1;
      g_accel = ACCEL;
    }
 
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)  {
//    RCLCPP_INFO(this->get_logger(),"subscribe_odom: %f %f",odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y);
      double r,p,y;
      old_x = new_x;
      old_y = new_y;
      old_theta = new_theta;
      new_x = odom_msg->pose.pose.position.x;
      new_y = odom_msg->pose.pose.position.y;
      double orient_x = odom_msg->pose.pose.orientation.x;
      double orient_y = odom_msg->pose.pose.orientation.y;
      double orient_z = odom_msg->pose.pose.orientation.z;
      double orient_w = odom_msg->pose.pose.orientation.w;

      tf2::Quaternion quat(odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z,odom_msg->pose.pose.orientation.w);
      if(g_motor_move){
        switch(g_direction_mode){
          case front:
            g_lengthed = g_lengthed + sqrt( (new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y) );
            break;
          case right:
          case left:
            new_theta = atan2(2*orient_w*orient_z, 1-2*orient_z*orient_z);
            g_theta = g_theta + (new_theta - old_theta);
            g_lengthed = abs(TREAD*g_theta);
            tf2::getEulerYPR(quat,y,p,r); 
            break;
          default:
            break;
        }
      }
      RCLCPP_INFO(this->get_logger(),"odo_callback: %f %f %f %f %f %f %f %f %f %f",g_theta,g_lengthed,new_theta,odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,orient_z,orient_w,r,p,y);
    }

    void sensor_callback(const raspimouse_msgs::msg::LightSensors::SharedPtr sensor_msg) {
      g_sen_r_value=sensor_msg->right;
      g_sen_l_value=sensor_msg->left;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<raspimicromouse_msgs::msg::Len>::SharedPtr publisher_len;
    rclcpp::Subscription<raspimicromouse_msgs::msg::Run>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<raspimouse_msgs::msg::LightSensors>::SharedPtr subscription_sensor;

};

int main(int argc, char **argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<SpdctlNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
