#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class PointCloudSubscriber : public rclcpp::Node
{
  public:
    PointCloudSubscriber()
    : Node("pointcloud_subscriber")
    {
      // subscription_ = this->create_subscription<std_msgs::msg::String>(
      // "topic", 10, std::bind(&PointCloudSubscriber::topic_callback, this, _1));
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", 10, std::bind(&PointCloudSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I heard something ");
      //do some processing here
      sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z"); 
      sensor_msgs::PointCloud2Iterator<int8_t> iter_r(*msg, "r");
      sensor_msgs::PointCloud2Iterator<int8_t> iter_g(*msg, "g");
      sensor_msgs::PointCloud2Iterator<int8_t> iter_b(*msg, "b"); 
    
      float x,y,z;
      uint8_t r,g,b;
      // for(unsigned int i = 0; i < p_point_buffer.size(); ++i,++iter_x, ++iter_y, ++iter_z,++iter_r, ++iter_g, ++iter_b)
      for(unsigned int i = 0; i < 5; ++i,++iter_x, ++iter_y, ++iter_z,++iter_r, ++iter_g, ++iter_b)
      {
          x = *iter_x;
          y = *iter_y;
          z = *iter_z;
          r = *iter_r;
          g = *iter_g;
          b = *iter_b;
          std::cout << "(x,y,z,r,g,b) = ("
            << x << ","
            << y << ","
            << z << ","
            << (int)r << ","
            << (int)g << ","
            << (int)b << ")" << std::endl;
      }
      
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}