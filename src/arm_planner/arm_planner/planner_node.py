#!/usr/bin/env python3
"""
路径规划节点（简单插值版）
功能：
  1. 订阅 /target_joints 话题，接收目标关节角度
  2. 生成一条包含单个路径点的 JointTrajectory
  3. 发布到 /joint_trajectory_controller/joint_trajectory
     控制器收到后会自动插值，驱动机械臂在指定时间内到达目标
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 使用我们自定义的消息类型
from arm_interfaces.msg import JointCommand


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner')

        # --- 订阅者：接收目标关节角度 ---
        self.subscription = self.create_subscription(
            JointCommand,                # 消息类型
            '/target_joints',            # 话题名称
            self.target_callback,        # 回调函数
            10                           # 队列长度
        )

        # --- 发布者：向控制器发送轨迹 ---
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 关节名称列表（必须和 URDF / 控制器配置中一致）
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        self.get_logger().info('规划器已启动，等待 /target_joints 指令...')
        self.get_logger().info(f'关节顺序：{self.joint_names}')

    def target_callback(self, msg: JointCommand):
        """
        收到目标角度后的处理逻辑
        JointCommand 消息包含 6 个字段：joint1 ~ joint6
        """
        # ---------- 1. 从消息中提取目标角度 ----------
        target_positions = [
            msg.joint1, msg.joint2, msg.joint3,
            msg.joint4, msg.joint5, msg.joint6
        ]

        self.get_logger().info(f'收到目标角度：{target_positions}')

        # ---------- 2. 构建轨迹消息 ----------
        trajectory = JointTrajectory()

        # 关节名称必须与控制器配置中的完全一致
        trajectory.joint_names = self.joint_names

        # ---------- 3. 构建路径点 ----------
        point = JointTrajectoryPoint()
        point.positions = target_positions

        # 2 秒后到达目标（控制器内部会用样条插值平滑运动）
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        # 可选：设置速度和加速度（如果不设置，控制器用默认值）
        # point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        trajectory.points.append(point)

        # ---------- 4. 发布轨迹 ----------
        self.trajectory_publisher.publish(trajectory)
        self.get_logger().info('轨迹已发布，机械臂将在 2 秒内到达目标')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    try:
        rclpy.spin(node)          # 保持节点运行，等待订阅消息
    except KeyboardInterrupt:
        node.get_logger().info('规划器被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
