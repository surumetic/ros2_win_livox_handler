#include <livox_handler.hpp>


// extern shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc2;


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

DeviceItem devices[kMaxLidarCount];
uint32_t data_recveive_count[kMaxLidarCount];

/** Connect all the broadcast device. */
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];    

void InitDevicesAndDataReceiveCount(){
    memset(devices, 0, sizeof(devices));
    memset(data_recveive_count, 0, sizeof(data_recveive_count));
}

void StopSampling(){
      /** Uninitialize Livox-SDK. */
      int i = 0;
      for (i = 0; i < kMaxLidarCount; ++i) {
        if (devices[i].device_state == kDeviceStateSampling) {
          /** Stop the sampling of Livox LiDAR. */
          std::cout << "Initialized device [" << i << "]" << std::endl; 
          LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);
        }
      }      
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
    // LivoxExtendRawPoint *p_point_buffer;

    static std::vector<LivoxExtendRawPoint> p_point_buffer;

    data_recveive_count[handle] ++ ;

    if ( data ->data_type == kExtendCartesian) {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
        // p_point_buffer.push_back(*p_point_data);    


        for(unsigned int i = 0; i < data_num; i++){
            p_point_buffer.push_back(p_point_data[i]);
                    
        }        
    }


    // std::mutex mtx;
    if (data_recveive_count[handle] % 500 == 0) {
        //https://github.com/ros2/turtlebot2_demo/blob/master/depthimage_to_pointcloud2/src/depthimage_to_pointcloud2_node.cpp

        shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // pc2_msg->header.stamp =ros_clock.now();// now;
        pc2_msg->header.frame_id="map";
        pc2_msg->height=1;
        pc2_msg->is_dense=false;
        pc2_msg->is_bigendian=false;
        pc2_msg->width=p_point_buffer.size();
        pc2_msg->fields.clear();
        pc2_msg->fields.reserve(1);
        pc2_msg->data.resize(p_point_buffer.size());
        sensor_msgs::PointCloud2Modifier pcd_modifier(*pc2_msg);
        // pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz","rgb");
        // pcd_modifier.setPointCloud2FieldsByString(2, "xyz","intensity");


        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");            
        sensor_msgs::PointCloud2Iterator<int8_t> iter_r(*pc2_msg, "r");
        sensor_msgs::PointCloud2Iterator<int8_t> iter_g(*pc2_msg, "g");
        sensor_msgs::PointCloud2Iterator<int8_t> iter_b(*pc2_msg, "b");            
        // sensor_msgs::PointCloud2Iterator<float> iter_i(*pc2_msg, "intensity");            

        // std::cout << "size of buffer : " << p_point_buffer.size() << std::endl;
        // for(unsigned int i = 0; i < p_point_buffer.size(); ++i,++iter_x, ++iter_y, ++iter_z){
        for(unsigned int i = 0; i < p_point_buffer.size(); ++i,++iter_x, ++iter_y, ++iter_z,++iter_r, ++iter_g, ++iter_b){
        // for(unsigned int i = 0; i < p_point_buffer.size(); ++i,++iter_x, ++iter_y, ++iter_z,++iter_i){
            *iter_x = (float)(p_point_buffer[i].x / 1000.f);
            *iter_y = (float)(p_point_buffer[i].y / 1000.f);
            *iter_z = (float)(p_point_buffer[i].z / 1000.f);

            // *iter_i = (float)(p_point_buffer[i].reflectivity * 1.f);

            *iter_r =  p_point_buffer[i].reflectivity;
            *iter_g =  p_point_buffer[i].reflectivity;
            *iter_b =  p_point_buffer[i].reflectivity;
            

            // *iter_r = (float)(p_point_buffer[i].reflectivity / 255.f);
            // *iter_g = (float)(p_point_buffer[i].reflectivity / 1.f);
            // *iter_b = (float)(p_point_buffer[i].reflectivity / 255.f);
        }
        publisher_pc2->publish(*pc2_msg);

        p_point_buffer.clear();


    }        

    if (data_recveive_count[handle] % 100 == 0) {
      /** Parsing the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      if(data ->data_type == kCartesian) {
        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
      }else if ( data ->data_type == kSpherical) {
        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
      }else if ( data ->data_type == kExtendCartesian) {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
        // cout << "reflectivity" << endl;
        // cout << (p_point_data[0].reflectivity / 255.f) << endl;//0-big

        // p_point_buffer.push_back(*p_point_data);

        // cout << "----" << endl;
        // cout << sizeof(LivoxExtendRawPoint) << endl; // 14
        // cout << sizeof(*p_point_data) << endl; // 14
        // cout << sizeof(p_point_data) << endl; // 8
        // // cout << sizeof(data->data) << endl; //1
        // // cout << sizeof(*data->data) << endl; //1
        // cout << sizeof(p_point_data[0]) << endl; //14
        // cout << p_point_data[0].x << endl;//0-big
        // cout << "data_num" << data_num << endl;//96

        // std::mutex mtx;
        // {
        //     // rosidl_runtime_c__uint8__Sequence _data;
        //     // _data.data;
        //     // std::cout << _data.size << std::endl;

        //     // ROSIDL_RUNTIME_C__PRIMITIVE_SEQUENCE(uint8, uint8_t)

        //     //   typedef struct rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence \
        //     //   { \
        //     //     TYPE_NAME * data; /*!< The pointer to an array of STRUCT_NAME */ \
        //     //     size_t size; /*!< The number of valid items in data */ \
        //     //     size_t capacity; /*!< The number of allocated items in data */ \
        //     //   } rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence;


        //     std::lock_guard<std::mutex> lock(mtx);

        //     // rclcpp::Clock system_clock(rcl_clock_type_t RCL_SYSTEM_TIME);
        //     // rclcpp::Clock ros_clock(rcl_clock_type_t RCL_ROS_TIME);
        //     // rclcpp::Clock steady_clock(rcl_clock_type_t RCL_STEADY_TIME);

        //     // rclcpp::Time now = system_clock.now();

        //     //https://github.com/ros2/turtlebot2_demo/blob/master/depthimage_to_pointcloud2/src/depthimage_to_pointcloud2_node.cpp

        //     shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        //     // pc2_msg->header.stamp =ros_clock.now();// now;
        //     pc2_msg->header.frame_id="map";
        //     pc2_msg->height=1;
        //     pc2_msg->is_dense=false;
        //     pc2_msg->is_bigendian=false;
        //     pc2_msg->width=data_num;
        //     pc2_msg->fields.clear();
        //     pc2_msg->fields.reserve(1);
        //     sensor_msgs::PointCloud2Modifier pcd_modifier(*pc2_msg);
        //     pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        //     sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
        //     sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
        //     sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");            
        //     // pc2_msg->data.size();// = _data;

        //     // std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
        //     //   _data_type data;


        //     // delete(pc2_msg->data.data)

        //     for(unsigned int i = 0; i < data_num; i++){
        //         *iter_x = (float)(p_point_data[i].x / 1000.f);
        //         *iter_y = (float)(p_point_data[i].y / 1000.f);
        //         *iter_z = (float)(p_point_data[i].z / 1000.f);

        //         // pc2_msg->data.
            
        //     }
        //     publisher_pc2->publish(*pc2_msg);

        // }


        //cout << p_point_data[0] << endl;

        // auto message = std::make_shared<std_msgs::msg::String>();
        // message->data = "GetLidarData";
        // publisher->publish(*message);

        // std::cout << "here?" << std::endl;


        // unsigned int dsize = sizeof(p_point_data);
        // std::cout << sizeof(data->data) << std::endl;

        // /** Extend cartesian coordinate format. */
        // typedef struct {
        //   int32_t x;            /**< X axis, Unit:mm */
        //   int32_t y;            /**< Y axis, Unit:mm */
        //   int32_t z;            /**< Z axis, Unit:mm */
        //   uint8_t reflectivity; /**< Reflectivity */
        //   uint8_t tag;          /**< Tag */
        // } LivoxExtendRawPoint;


      }else if ( data ->data_type == kExtendSpherical) {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kDualExtendCartesian) {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }else if ( data ->data_type == kDualExtendSpherical) {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kImu) {
        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendCartesian) {
        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendSpherical) {
        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
      }
    //   printf("data_type %d packet num %d\n", data->data_type, data_recveive_count[handle]);
    }
  }
}


/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function. */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL || info->dev_type == kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (lidar_count > 0) {
    bool found = false;
    int i = 0;
    for (i = 0; i < lidar_count; ++i) {
      if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess) {
    /** Set the point cloud data for a specific Livox LiDAR. */
    SetDataCallback(handle, GetLidarData, NULL);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}


/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventConnect) {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal) {
      LidarStartSampling(handle, OnSampleCallback, NULL);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}


/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

void LidarConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect) {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

void LidarDisConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}