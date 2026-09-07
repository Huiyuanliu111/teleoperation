// Copyright (c) Yansong
#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <typeinfo>
#include <filesystem>
#include <cstdio>
#include <memory>
#include <future>
#include <stdexcept>
#include <vector>



#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>


#include "Recorder.cpp"
#include "examples_common.h"
#include "Kinematics.h"
#include "Dynamics.h"

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/IPAddress.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "RealSenseCam1.cpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <zstd.h>

// #include <gram_savitzky_golay/gram_savitzky_golay.h>
#include "savgol.cpp"

// #include "TDPA.h"
#include "TDPA2010AsWhole.cpp"

// Json related, parameter configuration
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


// #include <Poco/Exception.h>



namespace
{
  template <class T, size_t N>
  std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
  {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
  }
} // anonymous namespace

// Leader

struct send_data
{
  std::mutex mutex;
  double pandatime = 0;
  std::array<double, 7> q_local_delta{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> dq_local = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> tau_local = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 6> f_local = {{0, 0, 0, 0, 0, 0}};
  double energy = 0;
  double stop_code = 0;
  std::atomic<double> gripper_width{0.08};
  double teleop_active = 0.0;
};

struct recv_data
{
  std::mutex mutex;
  std::atomic<bool> has_received{false};
  std::atomic<int64_t> last_receive_time_ns{0};
  double remotetime = 0;
  std::array<double, 7> q_remote_delta = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> dq_remote = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> tau_remote = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 6> f_remote = {{0, 0, 0, 0, 0, 0}};
  double energy = 0;
  double stop_code = 0;
  double gripper_width = 0.08;
  double teleop_active = 0.0;
};


void rgbd_camera_thread_func(
    int camera_index, std::atomic<bool> &running, const std::string &out_dir,
    const std::string &camera_serial, std::atomic<int> &ready_count,
    std::atomic<bool> &capture_failed, std::atomic<uint64_t> &committed_frames);

void udpwithremote_send(send_data &Data2Send, std::atomic<bool> &running);

void udpwithremote_recv(recv_data &Data2Recv, std::atomic<bool> &running);

bool movetoGrasp(franka::Gripper &gripper, double target_width,
                 bool &grasp_flag, bool &ever_grasped, double grasp_force);

void gripperControl(send_data &Data2Send, recv_data &Data2Recv, std::atomic<bool> &running, franka::Gripper &gripper,
                   const std::string &leadorfollow, bool initially_grasped,
                   double grasp_force);

//void check_wiggle_info(wiggle_para & w_para, json parameter);

std::condition_variable cv_send;
bool send_allowed{false};

const char *IP_remote;
