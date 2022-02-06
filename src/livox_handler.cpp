#include <livox_handler.hpp>

volatile sig_atomic_t bLoopEnd = 0;
void abrt_handler(int sig) {
  bLoopEnd = 1;
}

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc2;

// typedef struct sensor_msgs__msg__PointCloud2
// {
//   std_msgs__msg__Header header;
//   uint32_t height;
//   uint32_t width;
//   sensor_msgs__msg__PointField__Sequence fields;
//   bool is_bigendian;
//   uint32_t point_step;
//   uint32_t row_step;
//   rosidl_runtime_c__uint8__Sequence data;
//   bool is_dense;
// } sensor_msgs__msg__PointCloud2;


class ROS2LivoxHandler : public rclcpp::Node
{
  public:
    ROS2LivoxHandler()
    : Node("ros2_win_livox_handler")
    {
      // publisher = this->create_publisher<std_msgs::msg::String>("livox_status", 10);
      // publisher_pc2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2",10);
      publisher_pc2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2",rclcpp::QoS(10));

    }

    ~ROS2LivoxHandler(){
    }

    bool init_livox_sdk(){
      if (!Init()) {
        return false;
      }      

      printf("Livox SDK has been initialized.\n");
      GetLivoxSdkVersion(&_sdkversion);
      printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

      InitDevicesAndDataReceiveCount();

      /** Set the callback function receiving broadcast message from Livox LiDAR. */
      SetBroadcastCallback(OnDeviceBroadcast);

      /** Set the callback function called when device state change,
       * which means connection/disconnection and changing of LiDAR state. */
      SetDeviceStateUpdateCallback(OnDeviceInfoChange);

      return true;
    }    

    bool run()
    {
      rclcpp::WallRate loop_rate(100ms);
      auto pub_counter=0;
      auto message=std::make_shared<std_msgs::msg::String>();

      /** Start the device discovering routine. */
      if (!Start()) {
        Uninit();
        return false;
      }
      printf("Start discovering device.\n");


      while(rclcpp::ok() && !bLoopEnd){

        // std::mutex mtx;
        // {
        //   std::lock_guard<std::mutex> lock(mtx);
        //   publisher_pc2->publish(*pc2_msg);
        // }

        // message->data = "hello " + std::to_string(pub_counter++);
        // RCLCPP_INFO(this->get_logger(),"Pub:%s",message->data.c_str());
        // publisher->publish(*message);
        loop_rate.sleep();                                                                                                                                                                                                                                                                            
      }

      StopSampling();
      std::cout << "livox stop sampling" << std::endl;      

      Uninit();
      std::cout << "livox uninit" << std::endl;   

      return true;   
    }


  private:
    LivoxSdkVersion _sdkversion;
    // shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg;

    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc2;
};


int main(int argc, char * argv[])
{

  // Set Signal Handler to end cleanly with ctrl+c
  if ( signal(SIGINT, abrt_handler) == SIG_ERR ) {
    exit(1);
  }
  rclcpp::init(argc, argv);

  ROS2LivoxHandler handler;

  if (!handler.init_livox_sdk()){
    std::cerr << "handler init failed" << std::endl;
    return -1;
  }

  if(!handler.run()){
    std::cout << "Not started, ending immediately" << std::endl;
    return -1;
  }

  rclcpp::shutdown();
  std::cout << "ros shutdown" << std::endl;
  std::cout << "End Successfully" << std::endl;

  return 0;
}

