
#ifndef __LIVOX_HANDLER_HPP__

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

// #include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>
#include <thread>

#include "livox_sdk.h"




constexpr int TIME_TO_SLEEP = 3000;



using namespace std::chrono_literals;
using std::this_thread::sleep_for;
using namespace std;

void InitDevicesAndDataReceiveCount();
void StopSampling();
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);
void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type);
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data);
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data);
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data);
void LidarConnect(const DeviceInfo *info);
void LidarDisConnect(const DeviceInfo *info);
void LidarStateChange(const DeviceInfo *info);
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message);

#endif
#define __LIVOX_HANDLER_HPP__

