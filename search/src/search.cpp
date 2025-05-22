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
#include "raspimouse_msgs/msg/light_sensors.hpp"
#include "raspimicromouse_msgs/msg/run.hpp"
#include "raspimicromouse_msgs/msg/len.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include <chrono>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

#define SEN_R_TH 100
#define SEN_L_TH 100
#define SEN_FR_TH 100
#define SEN_FL_TH 100

#define MAZESIZE_X (16)  //迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y (16)  //迷路の大きさ(MAZESIZE_Y * MAZESIZE_Y)迷路

#define _UNKNOWN 2  //壁があるかないか判らない状態の場合の値
#define NOWALL 0    //壁がない場合の値
#define WALL 1      //壁がある場合の値
#define VWALL 3     //仮想壁の値(未使用)

typedef enum {
  front,
  right,
  left,
  rear,
  local_dir_error
} t_local_direction;

typedef enum {
  north,
  east,
  south,
  west,
  glob_dir_error
} t_global_direction;

typedef struct
{
  unsigned char x;
  unsigned char y;
  t_global_direction dir;
} t_position;

typedef struct
{
  unsigned char west : 2;   //西の壁情報bit7-6
  unsigned char south : 2;  //南の壁情報 bit5-4
  unsigned char east : 2;   //東の壁情報 bit3-2
  unsigned char north : 2;  //北の壁情報 bit1-0
} t_wall;                   //壁情報を格納する構造体(ビットフィールド)


class MapManager {
  public:
    t_position mypos;
    unsigned char goal_mx, goal_my;

    MapManager() {
      for (int i = 0; i < MAZESIZE_X; i++) {
        for (int j = 0; j < MAZESIZE_Y; j++) {
          wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west =
          _UNKNOWN;  //迷路情報を未知で初期化する
        }
      }
      for (int i = 0; i < MAZESIZE_X; i++) {
        wall[i][0].south = WALL;               //四方の壁を追加する(南)
        wall[i][MAZESIZE_Y - 1].north = WALL;  //四方の壁を追加する(北)
      }
      for (int i = 0; i < MAZESIZE_Y; i++) {
        wall[0][i].west = WALL;               //四方の壁を追加する(西)
        wall[MAZESIZE_X - 1][i].east = WALL;  //四方の壁を追加する(東)
      }
      wall[0][0].east = wall[1][0].west = WALL;  //スタート地点の右の壁を追加する
      wall[0][0].north = wall[0][1].south = NOWALL;
      mypos.x = 0;
      mypos.y = 0;
      mypos.dir = north;
      goal_mx = 0x07;
      goal_my = 0x07; 
    }

    void positionInit(void){
      mypos.x = mypos.y = 0;
      mypos.dir = north;
    }

    unsigned int wallDataRawGet(unsigned char x, unsigned char y) {
      return (unsigned int)(wall[x][y].north | (wall[x][y].east << 2) | (wall[x][y].south << 4) | (wall[x][y].west << 6));
    }

    char wallDataGet(unsigned char x, unsigned char y, t_global_direction l_global_dir) {
      switch (l_global_dir) {
        case north:
          return wall[x][y].north;
          break;
        case west:
          return wall[x][y].west;
          break;
        case south:
          return wall[x][y].south;
          break;
        case east:
          return wall[x][y].east;
          break;
        default:
          return 99;
      }
      return 99;
    }

    void wallDataSet(unsigned char x, unsigned char y, t_global_direction l_global_dir, char data) {
      switch (l_global_dir) {
        case north:
          wall[x][y].north = data;
          break;
        case west:
          wall[x][y].west = data;
          break;
        case south:
          wall[x][y].south = data;
          break;
        case east:
          wall[x][y].east = data;
          break;
        default:
          break;
      }
    }

    void axisUpdate(void) {
      switch (mypos.dir) {
        case north:
          mypos.y++;
          break;
        case east:
          mypos.x++;
          break;
        case south:
          mypos.y--;
          break;
        case west:
          mypos.x--;
          break;
        default:
          break;
      }
    }

    void rotateDirSet(t_local_direction l_local_dir) {
      if (l_local_dir == right) {
        switch (mypos.dir) {
          case north:
            mypos.dir = east;
            break;
          case east:
            mypos.dir = south;
            break;
          case south:
            mypos.dir = west;
            break;
          case west:
            mypos.dir = north;
            break;
          default:
            break;
        }
      } else if (l_local_dir == left) {
        switch (mypos.dir) {
          case north:
            mypos.dir = west;
            break;
          case east:
            mypos.dir = north;
            break;
          case south:
            mypos.dir = east;
            break;
          case west:
            mypos.dir = south;
            break;
          default:
            break;
        }
      }
    }

    void wallSet(bool IS_SEN_FR, bool IS_SEN_R, bool IS_SEN_L)  //壁情報を記録
    {
      switch (mypos.dir) {
        case north:
          wall[mypos.x][mypos.y].north = IS_SEN_FR ? WALL : NOWALL;
          wall[mypos.x][mypos.y].east = IS_SEN_R ? WALL : NOWALL;
          wall[mypos.x][mypos.y].west = IS_SEN_L ? WALL : NOWALL;
          if (mypos.y < (MAZESIZE_Y - 1)) wall[mypos.x][mypos.y + 1].south = IS_SEN_FR ? WALL : NOWALL;
          if (mypos.x < (MAZESIZE_X - 1)) wall[mypos.x + 1][mypos.y].west = IS_SEN_R ? WALL : NOWALL;
          if (mypos.x > 0) wall[mypos.x - 1][mypos.y].east = IS_SEN_L ? WALL : NOWALL;
          break;
        case east:
          wall[mypos.x][mypos.y].east = IS_SEN_FR ? WALL : NOWALL;
          wall[mypos.x][mypos.y].south = IS_SEN_R ? WALL : NOWALL;
          wall[mypos.x][mypos.y].north = IS_SEN_L ? WALL : NOWALL;
          if (mypos.x < (MAZESIZE_X - 1)) wall[mypos.x + 1][mypos.y].west = IS_SEN_FR ? WALL : NOWALL;
          if (mypos.y > 0) wall[mypos.x][mypos.y - 1].north = IS_SEN_R ? WALL : NOWALL;
          if (mypos.y < (MAZESIZE_Y - 1)) wall[mypos.x][mypos.y + 1].south = IS_SEN_L ? WALL : NOWALL;
          break;
        case south:
          wall[mypos.x][mypos.y].south = IS_SEN_FR ? WALL : NOWALL;
          wall[mypos.x][mypos.y].west = IS_SEN_R ? WALL : NOWALL;
          wall[mypos.x][mypos.y].east = IS_SEN_L ? WALL : NOWALL;
          if (mypos.y > 0) wall[mypos.x][mypos.y - 1].north = IS_SEN_FR ? WALL : NOWALL;
          if (mypos.x > 0) wall[mypos.x - 1][mypos.y].east = IS_SEN_R ? WALL : NOWALL;
          if (mypos.x < (MAZESIZE_X - 1)) wall[mypos.x + 1][mypos.y].west = IS_SEN_L ? WALL : NOWALL;
          break;
        case west:
          wall[mypos.x][mypos.y].west = IS_SEN_FR ? WALL : NOWALL;
          wall[mypos.x][mypos.y].north = IS_SEN_R ? WALL : NOWALL;
          wall[mypos.x][mypos.y].south = IS_SEN_L ? WALL : NOWALL;
          if (mypos.x > 0) wall[mypos.x - 1][mypos.y].east = IS_SEN_FR ? WALL : NOWALL;
          if (mypos.y < (MAZESIZE_Y - 1)) wall[mypos.x][mypos.y + 1].south = IS_SEN_R ? WALL : NOWALL;
          if (mypos.y > 0) wall[mypos.x][mypos.y - 1].north = IS_SEN_L ? WALL : NOWALL;
          break;
        default:
          break;
      }
    }

    void stepMapSet(unsigned char posX, unsigned char posY, t_global_direction l_global_dir, int *little, t_global_direction *now_dir, int *priority) {
      int tmp_priority;
      tmp_priority = priorityGet(posX, posY, l_global_dir);
      if (steps_map[posX][posY] < *little) {
        *little = steps_map[posX][posY];
        *now_dir = l_global_dir;
        *priority = tmp_priority;
      } else if (steps_map[posX][posY] == *little) {
        if (*priority < tmp_priority) {
          *now_dir = l_global_dir;
          *priority = tmp_priority;
        }
      }
    }

    t_local_direction nextGdir(t_global_direction *p_global_dir) {
      switch (*p_global_dir) {
        case north:
          switch (mypos.dir) {
            case north:
              return front;
              break;
            case east:
              return left;
              break;
            case south:
              return rear;
              break;
            case west:
              return right;
              break;
            default:
              return local_dir_error;
              break;
          }
          break;
        case east:
          switch (mypos.dir) {
            case east:
              return front;
              break;
            case south:
              return left;
              break;
            case west:
              return rear;
              break;
            case north:
              return right;
              break;
            default:
              return local_dir_error;
              break;
          }
          break;
        case south:
          switch (mypos.dir) {
            case south:
              return front;
              break;
            case west:
              return left;
              break;
            case north:
              return rear;
              break;
            case east:
              return right;
              break;
            default:
              return local_dir_error;
              break;
          }
          break;
        case west:
          switch (mypos.dir) {
            case west:
              return front;
              break;
            case north:
              return left;
              break;
            case east:
              return rear;
              break;
            case south:
              return right;
              break;
            default:
              return local_dir_error;
              break;
          }
          break;
        default:
          return local_dir_error;
          break;
      }
    }


    t_local_direction nextDirGet(unsigned char x, unsigned char y, t_global_direction *p_global_dir) {
      int little, priority;

      searchMapMake(x, y);
      little = 65535;
      priority = 0;

      if ((wall[mypos.x][mypos.y].north != WALL) && (mypos.y < (MAZESIZE_Y - 1))) {
        stepMapSet(mypos.x, mypos.y + 1, north, &little, p_global_dir, &priority);
      }
      if ((wall[mypos.x][mypos.y].east != WALL) && (mypos.x < (MAZESIZE_X - 1))) {
        stepMapSet(mypos.x + 1, mypos.y, east, &little, p_global_dir, &priority);
      }
      if ((wall[mypos.x][mypos.y].south != WALL) && (mypos.y > 0)) {
        stepMapSet(mypos.x, mypos.y - 1, south, &little, p_global_dir, &priority);
      }
      if ((wall[mypos.x][mypos.y].west != WALL) && (mypos.x > 0)) {
        stepMapSet(mypos.x - 1, mypos.y, west, &little, p_global_dir, &priority);
      }

      if (steps_map[mypos.x][mypos.y] == 65535) {
        return local_dir_error;   
      } else {
        return nextGdir(p_global_dir);
      }

      return front;
    }

    t_local_direction nextDir2Get(unsigned char x, unsigned char y, t_global_direction *p_global_dir) {
      int little, priority;

      map2Make(x, y);
      little = 65535;

      priority = 0;

      if ((wall[mypos.x][mypos.y].north == NOWALL) && ((mypos.y + 1) < MAZESIZE_Y)) {
        stepMapSet(mypos.x, mypos.y + 1, north, &little, p_global_dir, &priority);
      }

      if ((wall[mypos.x][mypos.y].east == NOWALL) && ((mypos.x + 1) < MAZESIZE_X)) {
        stepMapSet(mypos.x + 1, mypos.y, east, &little, p_global_dir, &priority);
      }

      if ((wall[mypos.x][mypos.y].south == NOWALL) && (mypos.y > 0)) {
        stepMapSet(mypos.x, mypos.y - 1, south, &little, p_global_dir, &priority);
      }

      if ((wall[mypos.x][mypos.y].west == NOWALL) && (mypos.x > 0)) {
        stepMapSet(mypos.x - 1, mypos.y, west, &little, p_global_dir, &priority);
      }

      if (steps_map[mypos.x][mypos.y] == 65535) {  //Goalにいけない
        return local_dir_error;
      } else {
        return nextGdir(p_global_dir);
      }

      return front;
    }

    void searchMapMake(unsigned char x, unsigned char y) {
      bool change_flag;

      for (int i = 0; i < MAZESIZE_X; i++) {
        for (int j = 0; j < MAZESIZE_Y; j++) {
          steps_map[i][j] = 65535;
        }
      }
      steps_map[x][y] = 0;

      do {
        change_flag = false;
        for (int i = 0; i < MAZESIZE_X; i++) {
          for (int j = 0; j < MAZESIZE_Y; j++) {
            if (steps_map[i][j] == 65535)
              continue;
            if ((j < (MAZESIZE_Y - 1)) && (wall[i][j].north != WALL) && (steps_map[i][j + 1] == 65535)) {
              steps_map[i][j + 1] = steps_map[i][j] + 1;
              change_flag = true;
            }

            if ((i < (MAZESIZE_X - 1)) && (wall[i][j].east != WALL) && (steps_map[i + 1][j] == 65535)) {
              steps_map[i + 1][j] = steps_map[i][j] + 1;
              change_flag = true;
            }

            if ((j > 0) && (wall[i][j].south != WALL) && (steps_map[i][j - 1] == 65535)) {
              steps_map[i][j - 1] = steps_map[i][j] + 1;
              change_flag = true;
            }

            if ((i > 0) && (wall[i][j].west != WALL) && (steps_map[i - 1][j] == 65535)) {
              steps_map[i - 1][j] = steps_map[i][j] + 1;
              change_flag = true;
            }
          }
        }
      } while (change_flag == true);
    }

    void map2Make(unsigned char x, unsigned char y) {
      bool change_flag;

      for (int i = 0; i < MAZESIZE_X; i++) {
        for (int j = 0; j < MAZESIZE_Y; j++) {
          steps_map[i][j] = 65535;
        }
      }
      steps_map[x][y] = 0;

      do {
        change_flag = false;
        for (int i = 0; i < MAZESIZE_X; i++) {
          for (int j = 0; j < MAZESIZE_Y; j++) {
            if (steps_map[i][j] == 65535)
              continue;
            if ((j < (MAZESIZE_Y - 1)) && (wall[i][j].north == NOWALL) && (steps_map[i][j + 1] == 65535)) {
              steps_map[i][j + 1] = steps_map[i][j] + 1;
              change_flag = true;
            }

            if ((i < (MAZESIZE_X - 1)) && (wall[i][j].east == NOWALL) && (steps_map[i + 1][j] == 65535)) {
              steps_map[i + 1][j] = steps_map[i][j] + 1;
              change_flag = true;
            }

            if ((j > 0) && (wall[i][j].south == NOWALL) && (steps_map[i][j - 1] == 65535)) {
              steps_map[i][j - 1] = steps_map[i][j] + 1;
              change_flag = true;
            }

            if ((i > 0) && (wall[i][j].west == NOWALL) && (steps_map[i - 1][j] == 65535)) {
              steps_map[i - 1][j] = steps_map[i][j] + 1;
              change_flag = true;
            }
          }
        }
      } while (change_flag == true);
    }

    int priorityGet(unsigned char x, unsigned char y, t_global_direction dir) {
      int priority;
      priority = 0;

      if (mypos.dir == dir) {
        priority = 2;
      } else if (
           ((mypos.dir == north) && (dir == south))
        || ((mypos.dir == east) && (dir == west))
        || ((mypos.dir == south) && (dir == north))
        || ((mypos.dir == west) && (dir == east))) {
        priority = 0;
      } else {
        priority = 1;
      }

      if (
         (wall[x][y].north == _UNKNOWN)
      || (wall[x][y].east == _UNKNOWN)
      || (wall[x][y].south == _UNKNOWN)
      || (wall[x][y].west == _UNKNOWN)) {
        priority += 4;
      }

      return priority;
    }

  private:
    unsigned short steps_map[MAZESIZE_X][MAZESIZE_Y];  //歩数マップ
    t_wall wall[MAZESIZE_X][MAZESIZE_Y];               //壁の情報を格納する構造体配列


};

class SearchNode : public rclcpp::Node, public MapManager{
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
      subscription_sensor = this->create_subscription<raspimouse_msgs::msg::LightSensors>(
        "light_sensors",10,std::bind(&SearchNode::sensor_callback,this,_1));
    }
  private:
    char run_result;
    float run_lengthed;
    short g_sen_fr_value,g_sen_fl_value,g_sen_r_value,g_sen_l_value;

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
   void Straight(int num){
   }

   void Accelerate(void){
   };

   void OneStep(void){
   }

   void Decelerate(void){
   }; 
 
   void rotate(t_local_direction dir,int times){
   };   


    void lefthand(void){
      Accelerate();
      while(1){
        if(g_sen_l_value < SEN_L_TH){
  	  Decelerate();
          rotate(left,1);
          Accelerate();
        }else if((g_sen_fl_value < SEN_FL_TH) && (g_sen_fr_value < SEN_FR_TH)){
          Straight(1);
        }else if(g_sen_r_value < SEN_R_TH){
          Decelerate();
          rotate(right,1);
          Accelerate();
        }else{
          Decelerate();
          rotate(right,2);
          Accelerate();
        }
      }
    }

    void adachi(unsigned char x,unsigned char y ){



    }

    void result_callback(const raspimicromouse_msgs::msg::Len::SharedPtr len_msg){
      run_lengthed = len_msg->length;
      run_result = len_msg->finish;
    }
    
    void current_angle_callback(const std_msgs::msg::Int16::SharedPtr angle_msg){
      RCLCPP_INFO(this->get_logger(),"current_arm: %d",angle_msg->data);
    }

    void sensor_callback(const raspimouse_msgs::msg::LightSensors::SharedPtr sensor_msg) {
      g_sen_fr_value=sensor_msg->forward_r;
      g_sen_fl_value=sensor_msg->forward_l; 
      g_sen_r_value=sensor_msg->right;
      g_sen_l_value=sensor_msg->left;
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
        lefthand();
        break;
      case 2://adachi-method
        positionInit();
        adachi(7,7);
        rotate(right,2);
        rotateDirSet(right);
        rotateDirSet(right);
      
        adachi(0,0);
        rotate(right,2);
        rotateDirSet(right);
        rotateDirSet(right);
//mapwrite
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
  rclcpp::Subscription<raspimouse_msgs::msg::LightSensors>::SharedPtr subscription_sensor;

};

int main(int argc, char **argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<SearchNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

