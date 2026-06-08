# 六轴机械臂视觉闭环抓取系统
[![演示视频](https://img.shields.io/badge/🎥-B站演示视频-ff69b4)](https://b23.tv/Bv54jQs)
> **当前版本 v1.2**：已完成舵机驱动、运动学建模仿真、关节级标定及**视觉闭环抓取**，系统已实现俄罗斯方块自主吸取与摆放。

## 🎯 项目亮点
- **视觉闭环抓取**：基于 YOLOv8 + 奥比中光深度相机，实现俄罗斯方块三维检测与定位，机械臂自主完成吸取与摆放。
- **全栈独立开发**：从结构设计、运动规划到硬件驱动全部独立完成。
- **低成本硬件方案**：幻尔总线舵机 + PCA9685 + 树莓派4B，物料成本 < 1500 元。
- **软件补偿精度**：结合关节级线性标定、雅可比误差转换与末端视觉反馈，将低成本舵机的末端定位误差从 >10mm 优化至 <3mm，抓取成功率 > 90%。
- **模块化架构**：感知、规划、控制三层松耦合，方便独立调试与迭代。

## 🎥 演示视频
**[点击观看：六轴机械臂视觉闭环抓取俄罗斯方块](https://b23.tv/Bv54jQs)**  
*视频中绿色检测框为 YOLOv8 实时识别结果，机械臂根据深度相机获取的三维坐标自主规划路径并完成吸取与摆放。*

## 🧱 系统架构
```
[奥比中光深度相机] → [arm_vision (YOLOv8)] → /object_poses → [MoveIt2 规划]
↓
/joint_trajectory
↓
[Rviz2 显示] ← /joint_states ← [ros2_control 硬件接口] ← TCP ← [树莓派 + PCA9685 + 舵机]
```
> **硬件接口层**：自研 `ros2_control` **SystemInterface** 插件，实现标准 `read()` / `write()` 生命周期方法，通过 TCP 与树莓派通信。**视觉推理（YOLOv8）运行于上位机（笔记本）**，树莓派仅负责接收角度指令并驱动 PCA9685。

| 层级 | 功能 | 实现 |
|:---|:---|:---|
| **感知层** | 目标检测与三维定位 | 奥比中光 Astra Pro Plus + YOLOv8 + 手眼标定（眼在手外） |
| **规划层** | 运动学解算与轨迹规划 | ROS2 Humble + MoveIt2 |
| **控制层** | 轨迹插补与闭环控制 | 自研 SystemInterface 硬件接口，TCP 通信 |
| **驱动层** | PWM 信号生成 | 树莓派 + PCA9685 + Python 舵机库 |

## 📊 性能指标
| 指标 | 当前值 | 备注 |
|:---|:---|:---|
| 目标检测帧率 | 30 FPS | YOLOv8n，运行于笔记本端 |
| 深度测距范围 | 0.3m - 5m | 奥比中光 Astra Pro Plus |
| 抓取成功率 | > 90% | 俄罗斯方块吸取测试 (20次) |
| 关节重复定位精度 | ±0.5° | 9 点线性标定后 |
| 末端开环精度 | < 10 mm | 仅关节补偿 |
| 末端闭环精度 | **< 3 mm** | 线性标定 + 雅可比误差转换 + 视觉反馈 |
| 通信延迟 | < 10ms | TCP 局域网 |

*注：视觉推理在笔记本端运行，树莓派仅负责 PWM 生成与舵机控制。*

## 🛠️ 技术栈
- **运动规划**: ROS2 Humble, MoveIt2, `ros2_control`
- **视觉**: YOLOv8 (ultralytics), OpenCV, ArUco
- **深度相机**: 奥比中光 Astra Pro Plus (RGB-D)
- **硬件驱动**: 自研 C++ SystemInterface 硬件接口（`ros2_control`），树莓派 PWM 驱动（Python）
- **标定**: 手眼标定, 关节9点线性标定, 雅可比伪逆误差补偿
- **通信**: TCP Sockets, ROS2 Topics/Services
- **模型**: SolidWorks 设计, Rviz2 可视化 (URDF 因专利申请暂不开源)

## 📂 仓库结构
```
arm_ws/src/
├── arm_description/          # 模型与可视化 (URDF 暂不公开)
├── arm_moveit_config/        # MoveIt2 配置 (默认模板，待替换)
├── arm_hardware_interface/   # ros2_control C++ 硬件接口（自研 SystemInterface）
├── arm_controller_config/    # 控制器配置与启动
├── arm_bringup/              # 系统总启动
├── arm_planner/              # 规划节点 (Python)
├── arm_vision/               # 视觉检测包 (YOLOv8)
│   ├── yolov8_node.py        # 主检测节点
│   ├── yolov8_detector.py    # 检测器类
│   ├── image_viewer.py       # 可视化节点
│   ├── config/camera_params.yaml
│   └── launch/vision.launch.py
└── arm_interfaces/           # 自定义消息/服务/动作
```

## 🔧 Roadmap (未来探索方向)
- [ ] **舵机升级为闭环电机**：将关节驱动从 PWM 开环舵机替换为带编码器的闭环伺服电机，从根本上提升关节控制精度与力矩反馈能力，实现末端亚毫米级重复定位精度。
- [ ] **VLA 大模型接入**：探索视觉-语言-动作（VLA）模型与机械臂系统的结合，实现自然语言指令到复杂任务规划的端到端控制，使机械臂具备多模态指令理解与自主决策能力。

## 🚀 快速开始（零硬件 · 看模型运动）
> **不需要机械臂、相机、树莓派**，5 分钟内看到你的六轴模型在 RViz 中运动。
说明：urdf文件因为专利申请中暂不开放，专利申请成功将同步开放。
### 前提条件
- Ubuntu 22.04
- ROS2 Humble（[安装指南](https://docs.ros.org/en/humble/Installation.html)）
- `joint_state_publisher_gui` 和 `rviz2`:
  ```bash
  sudo apt install ros-humble-joint-state-publisher-gui ros-humble-rviz2
  ```

### 1. 克隆仓库并编译
```bash
cd ~
git clone https://github.com/xiaobei2004/robot_arm_vision.git arm_ws/src
cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arm_bringup
source install/setup.bash
```

### 2. 启动 RViz 可视化
```bash
ros2 launch arm_bringup arm_upper_sim.launch.py
```
> ✅ **预期结果**：RViz 窗口弹出，显示六轴机械臂模型，左侧 `JointStatePublisher` 面板可见。

### 3. 发送一条模拟轨迹
打开**新终端**：
```bash
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [
    {positions: [0.5, -0.3, 0.2, -0.4, 0.1, 0.6], time_from_start: {sec: 2, nanosec: 0}}
  ]
}"
```
> ✅ **预期结果**：RViz 中的机械臂在 2 秒内平滑运动到目标角度。

**恭喜！** 你已跑通本项目的“Hello World”。接下来可以尝试：
- 有 **Orbbec 相机**？→ 运行视觉抓取流程
- 有 **真实机械臂**？→ 参考硬件上电流程并启动 `arm_hardware_interface`

## ⚠️ 注意事项
- **URDF 文件** 因专利申请暂不公开，待专利提交后补充；快速开始所用仿真模型为简化版，待仓库更新后可直接运行。
- **视觉模型权重** (`best.pt`) 未包含在仓库中，请自行训练或联系作者获取。
- 当前 **MoveIt 配置** 为 Setup Assistant 自动生成的默认模板，待标定数据更新后将替换为实际参数。
- 相机标定文件 (`camera_params.yaml`) 需根据实际使用的奥比中光相机重新标定，仓库内为示例文件。
- **手眼标定** 相关资料将在后续上传GitHub时单独整理，当前仓库中暂不提供详细步骤。

## 📝 许可证
本项目采用 [MIT License](LICENSE)。

## 📧 联系方式
如有疑问或合作意向，欢迎通过 GitHub Issues 或邮件联系：`3334540279@qq.com`
