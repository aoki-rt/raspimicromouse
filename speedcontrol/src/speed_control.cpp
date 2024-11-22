#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "raspimouse_msgs/msg/light_sensors.hpp"
#include "raspimouseclassic_msgs/msg/run.hpp" 
#include <chrono>
#include <string>
#include <memory>
#include <math.h>


geometry_msgs::msg::Twist twist_msg;
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


float g_length=0.0,g_lengthed=0.0;
char g_mode;
float g_accel=0.0;
float g_speed=0.0;
char g_run_state=0;
short g_sen_r_value,g_sen_l_value;

class SpdctlNode : public rclcpp::Node{
  public:
    SpdctlNode() : Node("spdctl_node"){
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      subscription_ = this->create_subscription<raspimouseclassic_msgs::msg::Run>(
        "run",10,std::bind(&SpdctlNode::run_callback,this,_1));
      subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",10,std::bind(&SpdctlNode::odom_callback,this,_1));
      subscription_sensor = this->create_subscription<raspimouse_msgs::msg::LightSensors>(
        "light_sensors",10,std::bind(&SpdctlNode::sensor_callback,this,_1));
      timer_ = this->create_wall_timer(10ms,std::bind(&SpdctlNode::timer_callback,this));
    }
  private:
    void timer_callback(){
      float omega;
      short tmp_r,tmp_l;
      if(g_length == g_lengthed){
        twist_msg.linear.x=0.0;
        twist_msg.angular.z=0.0;
      }else{
        g_speed+=g_accel/100.0;
        if(g_speed > MAX_SPEED){
          g_speed = MAX_SPEED;
        }
        if(g_speed < MIN_SPEED){
          g_speed = MIN_SPEED;
        }

        switch(g_mode){
          case 0:
            if(g_run_state==0){
              if( (g_length - g_lengthed) > ((g_speed*g_speed)-(MIN_SPEED*MIN_SPEED))/(2*g_accel) ){
              }else{
                g_run_state++;
                g_accel=-1*ACCEL;
              }
            }else{
              if(g_length < g_lengthed){
                g_length=g_lengthed=0.0;
                g_accel=0.0;
                g_speed=0.0;
              }
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
            break;
          default:
            g_speed=0.0;
            omega=0.0;
        }
        float tmp_speed_r = g_speed + omega;
        float tmp_speed_l = g_speed - omega;

        if(tmp_speed_r< MIN_SPEED){
           tmp_speed_r = MIN_SPEED;
        }
        if(tmp_speed_l < MIN_SPEED){
           tmp_speed_l = MIN_SPEED;
        }
        twist_msg.linear.x=(tmp_speed_r+tmp_speed_l)/2;
        twist_msg.angular.z=(tmp_speed_r-tmp_speed_l)/TREAD;
      }
      publisher_->publish(twist_msg);
    }
    void run_callback(const raspimouseclassic_msgs::msg::Run::SharedPtr run_msg) const {
      RCLCPP_INFO(this->get_logger(),"subscribe: %f %d ",run_msg->length,run_msg->mode);
      g_length = run_msg->length;
      g_mode = run_msg->mode;
      g_lengthed=0.0;
      g_run_state=0;
      g_accel = ACCEL;
      
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) const {
      static float old_x=0.0,new_x=0.0,old_y=0.0,new_y=0.0;

//    RCLCPP_INFO(this->get_logger(),"subscribe_odom: %f %f",odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y);
      old_x = new_x;
      old_y = new_y;
      new_x = odom_msg->pose.pose.position.x;
      new_y = odom_msg->pose.pose.position.y;
      g_lengthed = g_lengthed + sqrt( (new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y) );
    }
    void sensor_callback(const raspimouse_msgs::msg::LightSensors::SharedPtr sensor_msg) const {
      g_sen_r_value=sensor_msg->right;
      g_sen_l_value=sensor_msg->left;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<raspimouseclassic_msgs::msg::Run>::SharedPtr subscription_;
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
