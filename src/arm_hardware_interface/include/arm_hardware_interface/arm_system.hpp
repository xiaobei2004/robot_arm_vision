// Copyright 2020 ros2_control Development Team
//
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

#ifndef ARM_HARDWARE_INTERFACE__ARM_SYSTEM_HPP_
#define ARM_HARDWARE_INTERFACE__ARM_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <optional>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_hardware_interface
{

class ArmSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // TCP Socket 连接管理
  bool connect_to_hardware();
  void disconnect_from_hardware();
  
  // 命令发送
  bool send_command(const std::vector<double> & positions);
  
  // 原始消息发送（不加锁）
  bool send_raw_message(const std::string & message);
  
  // 接收消息（带超时）
  std::string receive_message(int timeout_ms);
  
  // 状态读取线程
  void state_reader_thread();

  // 关节数据
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;

  // TCP Socket 相关
  int socket_fd_;
  std::mutex socket_mutex_;
  std::string hardware_ip_;
  int hardware_port_;
  std::atomic<bool> connected_;
  std::atomic<bool> running_;

  // 状态读取线程
  std::unique_ptr<std::thread> state_reader_thread_ptr_;
  std::optional<std::vector<double>> last_valid_state_;
};

}  // namespace arm_hardware_interface

#endif  // ARM_HARDWARE_INTERFACE__ARM_SYSTEM_HPP_
