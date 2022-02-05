#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <signal.h>
#include <errno.h>

#include <stdio.h>
#include <stdlib.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <chrono>
#include <thread>

#include "livox_sdk.h"


using std::this_thread::sleep_for;
using namespace std;

constexpr int TIME_TO_SLEEP = 3000;


typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;


using namespace std::chrono_literals;
DeviceItem devices[kMaxLidarCount];
uint32_t data_recveive_count[kMaxLidarCount];

/** Connect all the broadcast device. */
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];

volatile sig_atomic_t bLoopEnd = 0;
void abrt_handler(int sig) {
  bLoopEnd = 1;
}


class ROS2LivoxHandler : public rclcpp::Node
{
  public:
    ROS2LivoxHandler()
    : Node("ros2_win_livox_handler")
    {
      // publisher = node->create_publisher<std_msgs::msg::String>("livox_status",rclcpp::QoS(10));
      publisher = this->create_publisher<std_msgs::msg::String>("livox_status", 10);
    }

    ~ROS2LivoxHandler(){
      std::cout << "end loop" << std::endl;

    }

    void run()
    {
      rclcpp::WallRate loop_rate(100ms);
      auto pub_counter=0;
      auto message=std::make_shared<std_msgs::msg::String>();

      while(rclcpp::ok() && !bLoopEnd){
        message->data = "hello " + std::to_string(pub_counter++);
        RCLCPP_INFO(this->get_logger(),"Pub:%s",message->data.c_str());
        publisher->publish(*message);
        loop_rate.sleep();                                                                                                                                                                                                                                                                            
      }
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    
};


int main(int argc, char * argv[])
{

  // Set Signal Handler to end cleanly with ctrl+c
  if ( signal(SIGINT, abrt_handler) == SIG_ERR ) {
    exit(1);
  }
  rclcpp::init(argc, argv);

  ROS2LivoxHandler handler;
  handler.run();

  rclcpp::shutdown();
  std::cout << "ros shutdown" << std::endl;
  std::cout << "End Successfully" << std::endl;

  return 0;
}

