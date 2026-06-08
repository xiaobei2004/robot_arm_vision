// Copyright 2020 ros2_control Development Team
// Licensed under the Apache License, Version 2.0

#include "arm_hardware_interface/arm_system.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware_interface
{

// JSON辅助函数
std::string create_json_message(const std::string & type, const std::vector<double> & positions = {})
{
    std::stringstream ss;
    ss << "{\"type\":\"" << type << "\"";
    if (!positions.empty()) {
        ss << ",\"positions\":[";
        for (size_t i = 0; i < positions.size(); ++i) {
            if (i > 0) ss << ",";
            ss << positions[i];
        }
        ss << "]";
    }
    ss << "}\n";
    return ss.str();
}

std::vector<double> parse_json_positions(const std::string & json_str)
{
    std::vector<double> positions;
    
    // 查找 "positions" 并处理可能的空格
    size_t pos_key = json_str.find("\"positions\"");
    if (pos_key == std::string::npos) return positions;
    
    // 找到 ":" 后的 "["
    size_t bracket_start = json_str.find("[", pos_key);
    if (bracket_start == std::string::npos) return positions;
    
    // 找到对应的 "]"
    size_t bracket_end = json_str.find("]", bracket_start);
    if (bracket_end == std::string::npos) return positions;
    
    std::string pos_str = json_str.substr(bracket_start + 1, bracket_end - bracket_start - 1);
    std::stringstream ss(pos_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) positions.push_back(std::stod(token));
        } catch (...) { return {}; }
    }
    return positions;
}

std::string parse_json_type(const std::string & json_str)
{
    // 查找 "type" key
    size_t type_key = json_str.find("\"type\"");
    if (type_key == std::string::npos) return "";
    
    // 从 "type" 后找 ":"
    size_t colon_pos = json_str.find(":", type_key);
    if (colon_pos == std::string::npos) return "";
    
    // 跳过空格找到开头的引号
    size_t quote_start = json_str.find("\"", colon_pos);
    if (quote_start == std::string::npos) return "";
    quote_start++;  // 跳过开头的引号
    
    // 找到结尾的引号
    size_t quote_end = json_str.find("\"", quote_start);
    if (quote_end == std::string::npos) return "";
    
    return json_str.substr(quote_start, quote_end - quote_start);
}

// 生命周期回调
hardware_interface::CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    joint_names_.clear();
    for (const auto & joint : info.joints)
        joint_names_.push_back(joint.name);
    num_joints_ = joint_names_.size();
    
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Initialized with %zu joints", num_joints_);

    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);
    joint_position_command_.resize(num_joints_, 0.0);

    hardware_ip_ = "192.168.1.100";
    hardware_port_ = 12345;
    
    if (info.hardware_parameters.find("ip") != info.hardware_parameters.end())
        hardware_ip_ = info.hardware_parameters.at("ip");
    if (info.hardware_parameters.find("port") != info.hardware_parameters.end()) {
        try { hardware_port_ = std::stoi(info.hardware_parameters.at("port")); }
        catch (...) {}
    }
    
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Hardware: %s:%d", hardware_ip_.c_str(), hardware_port_);

    socket_fd_ = -1;
    connected_ = false;
    running_ = false;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystem::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Configuring...");
    
    if (!connect_to_hardware()) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Failed to connect");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    running_ = true;
    state_reader_thread_ptr_ = std::make_unique<std::thread>(&ArmSystem::state_reader_thread, this);
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "状态读取线程已创建");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystem::on_activate(const rclcpp_lifecycle::State &)
{
    for (size_t i = 0; i < num_joints_; ++i) {
        joint_position_[i] = 0.0;
        joint_velocity_[i] = 0.0;
        joint_effort_[i] = 0.0;
        joint_position_command_[i] = 0.0;
    }
    last_valid_state_.reset();
    
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    running_ = false;
    if (state_reader_thread_ptr_ && state_reader_thread_ptr_->joinable())
        state_reader_thread_ptr_->join();
    
    disconnect_from_hardware();
    
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// 接口导出
std::vector<hardware_interface::StateInterface> ArmSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < num_joints_; ++i) {
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_[i]);
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]);
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_effort_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
    return interfaces;
}

// 读写周期
hardware_interface::return_type ArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (last_valid_state_ && last_valid_state_->size() == num_joints_) {
        for (size_t i = 0; i < num_joints_; ++i) {
            joint_position_[i] = (*last_valid_state_)[i];
            joint_velocity_[i] = 0.0;
            joint_effort_[i] = 0.0;
        }
    } else {
        static int count = 0;
        if (count++ % 100 == 0)
            RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Waiting for valid state... last_valid_state_=%s, size=%zu", 
                last_valid_state_ ? "has_value" : "null",
                last_valid_state_ ? last_valid_state_->size() : 0);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    static int write_count = 0;
    if (write_count++ % 100 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "write: 发送命令 [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            joint_position_command_[0], joint_position_command_[1], joint_position_command_[2],
            joint_position_command_[3], joint_position_command_[4], joint_position_command_[5]);
    }
    send_command(joint_position_command_);
    return hardware_interface::return_type::OK;
}

// TCP连接管理
bool ArmSystem::connect_to_hardware()
{
    std::lock_guard<std::mutex> lock(socket_mutex_);
    
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Socket create failed: %s", strerror(errno));
        return false;
    }

    int opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(hardware_port_);
    
    if (inet_pton(AF_INET, hardware_ip_.c_str(), &server_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Invalid IP: %s", hardware_ip_.c_str());
        ::close(socket_fd_); socket_fd_ = -1;
        return false;
    }

    if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Connect failed: %s", strerror(errno));
        ::close(socket_fd_); socket_fd_ = -1;
        return false;
    }

    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Connected to %s:%d", hardware_ip_.c_str(), hardware_port_);
    return true;
}

void ArmSystem::disconnect_from_hardware()
{
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "Disconnected");
}

// 消息发送
bool ArmSystem::send_command(const std::vector<double> & positions)
{
    std::string message = create_json_message("set_position", positions);
    
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!connected_ || socket_fd_ < 0) {
        return false;
    }
    
    ssize_t sent = ::send(socket_fd_, message.c_str(), message.length(), MSG_NOSIGNAL);
    if (sent <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "发送失败: %s", strerror(errno));
        connected_ = false;
        return false;
    }
    
    return true;
}

bool ArmSystem::send_raw_message(const std::string & message)
{
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!connected_ || socket_fd_ < 0) {
        return false;
    }
    
    ssize_t sent = ::send(socket_fd_, message.c_str(), message.length(), MSG_NOSIGNAL);
    if (sent <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "发送原始消息失败: %s", strerror(errno));
        connected_ = false;
        return false;
    }
    return true;
}

// 接收消息 - 按\n分割，只返回第一个完整消息
std::string ArmSystem::receive_message(int timeout_ms)
{
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    
    char buffer[4096] = {0};
    ssize_t received = ::recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
    
    if (received <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return "";  // 超时
        }
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "接收失败: %s", strerror(errno));
        connected_ = false;
        return "";
    }
    
    std::string data(buffer, received);
    
    // 按\n分割，只取第一个完整消息
    size_t newline_pos = data.find('\n');
    if (newline_pos != std::string::npos) {
        std::string first_message = data.substr(0, newline_pos);
        // 去除可能的\r
        if (!first_message.empty() && first_message.back() == '\r') {
            first_message.pop_back();
        }
        return first_message;
    }
    
    // 如果没有\n，返回整个数据（可能是不完整的消息）
    return data;
}

// 状态读取线程 - 从树莓派获取实际状态
void ArmSystem::state_reader_thread()
{
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "===== 状态读取线程启动 =====");
    
    int loop_count = 0;
    while (running_) {
        loop_count++;
        
        // 发送get_state请求
        std::string get_state_msg = create_json_message("get_state");
        
        if (loop_count <= 5 || loop_count % 50 == 0) {
            RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "状态循环 #%d: 发送get_state", loop_count);
        }
        
        if (send_raw_message(get_state_msg)) {
            // 等待响应，可能需要多次读取跳过ack消息
            bool got_state = false;
            int max_attempts = 10;  // 最多读取10次
            
            while (!got_state && max_attempts-- > 0 && running_) {
                std::string response = receive_message(100);  // 100ms超时
                
                if (response.empty()) {
                    if (loop_count <= 5) {
                        RCLCPP_WARN(rclcpp::get_logger("ArmSystem"), "接收响应超时");
                    }
                    break;
                }
                
                if (loop_count <= 5) {
                    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "收到响应: '%s'", response.c_str());
                }
                
                // 解析响应
                std::string type = parse_json_type(response);
                if (loop_count <= 5) {
                    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "解析到类型: '%s'", type.c_str());
                }
                
                if (type == "state") {
                    std::vector<double> positions = parse_json_positions(response);
                    if (positions.size() == num_joints_) {
                        last_valid_state_ = positions;
                        got_state = true;
                        if (loop_count <= 5 || loop_count % 50 == 0) {
                            RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "状态更新成功: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                                positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);
                        }
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("ArmSystem"), "状态解析: positions大小=%zu, 期望=%zu", positions.size(), num_joints_);
                    }
                } else if (type == "ack") {
                    // 跳过ack消息，继续读取
                    if (loop_count <= 5) {
                        RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "跳过ack消息");
                    }
                } else {
                    if (loop_count <= 5) {
                        RCLCPP_WARN(rclcpp::get_logger("ArmSystem"), "收到未知类型: '%s'", type.c_str());
                    }
                }
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ArmSystem"), "发送get_state请求失败，尝试重连...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!connected_) {
                connect_to_hardware();
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz查询
    }
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "===== 状态读取线程停止 =====");
}

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware_interface::ArmSystem, hardware_interface::SystemInterface)
