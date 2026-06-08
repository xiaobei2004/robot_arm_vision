#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
import os
import time
from collections import defaultdict

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        
        # 加载YOLOv8模型
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.85)  # 置信度阈值参数
        self.declare_parameter('consecutive_valid_count', 6)  # 连续有效次数参数
        self.declare_parameter('coordinate_threshold', 1.2)  # 坐标差异阈值参数(mm)
        self.declare_parameter('storage_threshold', 0.05)  # 存储精度阈值，用于合并相似坐标
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.consecutive_valid_count = self.get_parameter('consecutive_valid_count').value
        self.coordinate_threshold = self.get_parameter('coordinate_threshold').value
        self.storage_threshold = self.get_parameter('storage_threshold').value
        
        self.model = YOLO(model_path)
        
        # 创建ROS 2相关对象
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅图像和相机信息
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            7)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            7)
        
        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.rgb_info_callback,
            10)
        
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback,
            10)
        
        self.publisher = self.create_publisher(Image, 'output_image', 10)
        self.poses_publisher = self.create_publisher(PoseStamped, 'object_poses', 10)
        
        # 存储相机内参和图像
        self.rgb_intrinsics = None
        self.depth_intrinsics = None
        self.color_image = None
        self.depth_image = None
        
        # 边界框优化参数
        self.threshold_factor = 0.8  # 用于确定前景的阈值因子
        self.min_area = 100  # 最小有效区域面积
        
        # 用于存储物体坐标历史和验证状态
        self.object_coordinates = {}  # {object_id: {'history': [], 'consecutive_valid': 0, 'last_valid': None}}
        
        # 用于跟踪物体ID是否已写入文件
        self.id_written_status = defaultdict(lambda: False)
        
        # 用于存储上次写入的坐标
        self.last_written_coordinates = defaultdict(lambda: None)
        
        # 用于存储各类别的文件句柄
        self.category_files = {}
        
        # 创建存储坐标的文档目录
        self.data_dir = os.path.join(os.path.expanduser("~"), "yolov8_coordinates_by_category")
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.get_logger().info(f'YOLOv8节点已启动，订阅图像和相机信息话题')
        self.get_logger().info(f'置信度阈值设置为: {self.confidence_threshold}')
        self.get_logger().info(f'连续有效次数要求: {self.consecutive_valid_count}')
        self.get_logger().info(f'坐标差异阈值: {self.coordinate_threshold}m')
        self.get_logger().info(f'坐标数据将按类别存储在: {self.data_dir}')

    def rgb_info_callback(self, msg):
        # 解析RGB相机内参
        self.rgb_intrinsics = np.array(msg.k).reshape(3, 3)

    def depth_info_callback(self, msg):
        # 解析深度相机内参
        self.depth_intrinsics = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        # 存储深度图像（使用32位浮点数格式，单位为米）
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            # 尝试其他可能的深度图像格式
            self.get_logger().warn(f'无法以32FC1格式解析深度图像，尝试其他格式: {e}')
            try:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                # 如果是16位无符号整数，转换为米（本相机单位为毫米，需除以1000）
                self.depth_image = self.depth_image.astype(np.float32) / 1000.0
            except Exception as e2:
                self.get_logger().error(f'深度图像解析失败: {e2}')
                self.depth_image = None

    def refine_bounding_box(self, image, bbox):
        """
        优化边界框，使其更紧密地贴合物体边缘
        
        参数:
        image: 原始RGB图像
        bbox: YOLO检测的边界框 [x1, y1, x2, y2]
        
        返回:
        refined_bbox: 优化后的边界框 [x1, y1, x2, y2]
        """
        # 提取边界框区域
        x1, y1, x2, y2 = map(int, bbox)
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(image.shape[1] - 1, x2)
        y2 = min(image.shape[0] - 1, y2)
        
        if x1 >= x2 or y1 >= y2:
            return bbox
            
        roi = image[y1:y2, x1:x2]
        
        # 转换为HSV色彩空间，更适合颜色分割
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 计算颜色直方图
        hist = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
        
        # 找到主要颜色
        mask = hist > (hist.max() * 0.1)  # 找到主要颜色区域
        hist[~mask] = 0
        
        # 创建颜色掩码
        flat_hsv = hsv.reshape(-1, 3)
        color_dist = np.zeros(flat_hsv.shape[0])
        
        # 计算每个像素与主要颜色的距离
        for i in range(flat_hsv.shape[0]):
            h, s, v = flat_hsv[i]
            color_dist[i] = np.sum(hist[h, s])
        
        # 归一化距离
        if color_dist.max() > 0:
            color_dist = color_dist / color_dist.max()
        
        # 二值化
        threshold = np.percentile(color_dist, 30)  # 使用30%作为阈值
        mask = (color_dist > threshold).reshape(roi.shape[:2])
        
        # 形态学操作清理噪声
        kernel = np.ones((3, 3), np.uint8)
        mask = mask.astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 找到轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return bbox
            
        # 找到最大轮廓
        largest_contour = max(contours, key=lambda c: cv2.contourArea(c))
        
        # 如果轮廓面积太小，返回原始边界框
        if cv2.contourArea(largest_contour) < self.min_area:
            return bbox
            
        # 计算最小外接矩形
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # 返回优化后的边界框
        refined_bbox = [x1 + x, y1 + y, x1 + x + w, y1 + y + h]
        
        return refined_bbox

    def is_coordinate_valid(self, current_coord, object_id, cls):
        """
        检查坐标是否连续有效
        
        参数:
        current_coord: 当前三维坐标 (x, y, z)
        object_id: 物体ID
        cls: 物体类别
        
        返回:
        is_valid: 坐标是否有效
        """
        x, y, z = current_coord
        
        # 初始化该物体的坐标历史（如果不存在）
        if object_id not in self.object_coordinates:
            self.object_coordinates[object_id] = {
                'history': [current_coord],
                'consecutive_valid': 0,
                'last_valid': None,
                'class': cls,
                'frame_count': 0  # 持续帧数统计
            }
            return False
        
        history = self.object_coordinates[object_id]['history']
        consecutive_valid = self.object_coordinates[object_id]['consecutive_valid']
        
        # 只保留最近的n次坐标记录
        if len(history) >= self.consecutive_valid_count:
            history.pop(0)
        history.append(current_coord)
        
        # 检查当前坐标与历史坐标的差异
        if len(history) < 2:
            return False
            
        # 计算与上一次坐标的差异
        last_coord = history[-2]
        dx = abs(x - last_coord[0])
        dy = abs(y - last_coord[1])
        dz = abs(z - last_coord[2])
        
        # 检查所有维度的差异是否都小于阈值
        if dx < self.coordinate_threshold and dy < self.coordinate_threshold and dz < self.coordinate_threshold:
            consecutive_valid += 1
            self.object_coordinates[object_id]['consecutive_valid'] = consecutive_valid
            
            # 如果连续有效次数达到要求，标记为有效
            if consecutive_valid >= self.consecutive_valid_count:
                self.object_coordinates[object_id]['last_valid'] = current_coord
                self.object_coordinates[object_id]['frame_count'] += 1  # 增加持续帧数
                return True
        else:
            # 差异超过阈值，重置连续有效计数
            self.object_coordinates[object_id]['consecutive_valid'] = 0
            self.object_coordinates[object_id]['frame_count'] = 0  # 重置持续帧数
            
        return False

    def should_store_coordinate(self, object_id):
        """
        判断是否应该存储当前坐标（每个ID仅存储一次）
        
        参数:
        object_id: 物体ID
        
        返回:
        bool: 是否应该存储
        """
        # 检查该ID是否已写入文件
        return not self.id_written_status[object_id]

    def get_category_file(self, category):
        """获取或创建指定类别的存储文件，仅在文件不存在时写入头部"""
        if category not in self.category_files:
            file_path = os.path.join(self.data_dir, f"{category}_coordinates.txt")
            
            # 打开文件（如果不存在则创建，存在则追加）
            file_mode = 'a'
            need_write_header = not os.path.exists(file_path)
            
            # 打开文件句柄
            self.category_files[category] = open(file_path, file_mode)
            
            # 仅在新文件中写入头部
            if need_write_header:
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                self.category_files[category].write(f"# {category}三维坐标记录 - {timestamp}\n")
                self.category_files[category].write("# 格式: 时间戳, 物体ID, 类别, x(mm), y(mm), z(mm), 持续帧数\n")
                self.category_files[category].flush()  # 刷新缓冲区确保写入
                
            self.get_logger().info(f"使用{category}类别存储文件: {file_path}")
        
        return self.category_files[category]

    def generate_object_id(self, cls, center_x, center_y):
        """
        生成唯一的物体ID，包含类别和位置信息
        由于每种颜色有5个方块，这里使用四舍五入的坐标来区分不同方块
        """
        # 对坐标进行四舍五入，以便将相近位置的方块视为同一个
        rounded_x = round(center_x / 20) * 20
        rounded_y = round(center_y / 20) * 20
        return f"{cls}_{int(rounded_x)}_{int(rounded_y)}"

    def image_callback(self, msg):
        try:
            # 确保已获取相机内参
            if self.rgb_intrinsics is None or self.depth_intrinsics is None:
                self.get_logger().warn('尚未获取相机内参，无法计算三维坐标')
                return
            
            # 确保已获取深度图像
            if self.depth_image is None:
                self.get_logger().warn('尚未获取深度图像，无法计算三维坐标')
                return
            
            # 将ROS图像消息转换为OpenCV格式
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 使用YOLOv8进行目标检测
            results = self.model(self.color_image)
            
            # 创建用于绘制的图像副本
            annotated_image = self.color_image.copy()
            
            # 处理每个检测到的目标
            for result in results:
                boxes = result.boxes  # 检测框
                for i, box in enumerate(boxes):
                    bbox = box.xyxy[0].tolist()  # 边界框坐标[x1, y1, x2, y2]
                    conf = box.conf[0].item()    # 置信度
                    cls = result.names[box.cls[0].item()]  # 类别名称
                    
                    # 置信度过滤，只处理置信度大于阈值的目标
                    if conf < self.confidence_threshold:
                        continue
                    
                    # 优化边界框
                    refined_bbox = self.refine_bounding_box(self.color_image, bbox)
                    
                    # 计算优化后边界框的中心点
                    center_x = (refined_bbox[0] + refined_bbox[2]) / 2
                    center_y = (refined_bbox[1] + refined_bbox[3]) / 2
                    
                    # 确保中心点坐标在深度图像范围内
                    if 0 <= int(center_y) < self.depth_image.shape[0] and 0 <= int(center_x) < self.depth_image.shape[1]:
                        # 从深度图像获取中心点深度值
                        try:
                            depth_value = self.depth_image[int(center_y), int(center_x)]
                            
                            # 跳过无效深度值
                            if np.isnan(depth_value) or depth_value <= 0 or np.isinf(depth_value):
                                self.get_logger().warn('深度值无效，跳过计算')
                                # 绘制黄色框（未处理状态）
                                cv2.rectangle(annotated_image, 
                                            (int(refined_bbox[0]), int(refined_bbox[1])), 
                                            (int(refined_bbox[2]), int(refined_bbox[3])), 
                                            (0, 255, 255), 2)  # 黄色边框
                                cv2.putText(annotated_image, 
                                          f'{cls} {conf:.2f} (无效深度)', 
                                          (int(refined_bbox[0]), int(refined_bbox[1]) - 10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                                continue
                            
                            # 计算三维坐标 (使用深度相机内参，单位为米)
                            z = float(depth_value)  # 深度值（米）
                            x = float((center_x - self.depth_intrinsics[0, 2]) * z / self.depth_intrinsics[0, 0])
                            y = float((center_y - self.depth_intrinsics[1, 2]) * z / self.depth_intrinsics[1, 1])
                            
                            # 生成唯一的物体ID，包含类别和四舍五入的坐标
                            object_id = self.generate_object_id(cls, center_x, center_y)
                            
                            # 检查坐标是否连续有效
                            is_valid = self.is_coordinate_valid((x, y, z), object_id, cls)
                            
                            if is_valid:
                                # 检查是否应该存储这个坐标（每个ID仅存储一次）
                                if self.should_store_coordinate(object_id):
                                    # 创建位姿消息
                                    pose = PoseStamped()
                                    pose.header.stamp = self.get_clock().now().to_msg()
                                    pose.header.frame_id = 'camera_depth_optical_frame'
                                    pose.pose.position.x = x
                                    pose.pose.position.y = y
                                    pose.pose.position.z = z
                                    
                                    # 发布三维坐标
                                    self.poses_publisher.publish(pose)
                                    
                                    # 标记该ID为已写入
                                    self.id_written_status[object_id] = True
                                    
                                    # 更新上次存储的坐标（保留记录）
                                    self.last_written_coordinates[object_id] = (x, y, z)
                                    
                                    # 打印有效坐标到控制台
                                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                                    frame_count = self.object_coordinates[object_id]['frame_count']
                                    self.get_logger().info(f'[存储并发布] 时间: {timestamp}, 物体: {cls}, 坐标: ({x:.3f}, {y:.3f}, {z:.3f})mm, 持续帧数: {frame_count}')
                                    
                                    # 按类别存储有效坐标到对应的文档
                                    category_file = self.get_category_file(cls)
                                    category_file.write(f"{timestamp}, {object_id}, {cls}, {x:.3f}, {y:.3f}, {z:.3f}, {frame_count}\n")
                                    category_file.flush()  # 强制刷新缓冲区，确保数据写入磁盘
                                    
                                    # 绘制绿色框（已发布/写入）
                                    box_color = (0, 255, 0)  # 绿色
                                    status_text = "(已存储)"
                                else:
                                    # 未存储但坐标有效，绘制黄色框
                                    box_color = (0, 255, 255)  # 黄色
                                    status_text = "(已检测)"
                                
                                # 绘制边界框和状态文本
                                cv2.rectangle(annotated_image, 
                                            (int(refined_bbox[0]), int(refined_bbox[1])), 
                                            (int(refined_bbox[2]), int(refined_bbox[3])), 
                                            box_color, 2)
                                cv2.putText(annotated_image, 
                                          f'{cls} {conf:.2f} {status_text}', 
                                          (int(refined_bbox[0]), int(refined_bbox[1]) - 10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                            
                            else:
                                # 坐标无效，绘制黄色框
                                box_color = (0, 255, 255)  # 黄色
                                status_text = "(坐标无效)"
                                cv2.rectangle(annotated_image, 
                                            (int(refined_bbox[0]), int(refined_bbox[1])), 
                                            (int(refined_bbox[2]), int(refined_bbox[3])), 
                                            box_color, 2)
                                cv2.putText(annotated_image, 
                                          f'{cls} {conf:.2f} {status_text}', 
                                          (int(refined_bbox[0]), int(refined_bbox[1]) - 10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                        
                        except IndexError:
                            self.get_logger().warn('深度图像索引超出范围')
                            # 绘制黄色框（索引错误）
                            cv2.rectangle(annotated_image, 
                                        (int(refined_bbox[0]), int(refined_bbox[1])), 
                                        (int(refined_bbox[2]), int(refined_bbox[3])), 
                                        (0, 255, 255), 2)  # 黄色边框
                            cv2.putText(annotated_image, 
                                      f'{cls} {conf:.2f} (索引错误)', 
                                      (int(refined_bbox[0]), int(refined_bbox[1]) - 10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            continue
                    else:
                        self.get_logger().warn('边界框中心点超出深度图像范围')
                        # 绘制黄色框（超出范围）
                        cv2.rectangle(annotated_image, 
                                    (int(refined_bbox[0]), int(refined_bbox[1])), 
                                    (int(refined_bbox[2]), int(refined_bbox[3])), 
                                    (0, 255, 255), 2)  # 黄色边框
                        cv2.putText(annotated_image, 
                                  f'{cls} {conf:.2f} (超出范围)', 
                                  (int(refined_bbox[0]), int(refined_bbox[1]) - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # 发布带标注的图像
            output_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            self.publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
    
    def destroy_node(self):
        """节点销毁时的回调函数，用于关闭所有文件"""
        self.get_logger().info('节点正在关闭，关闭所有数据文件...')
        for file in self.category_files.values():
            file.close()
        self.get_logger().info('数据文件已关闭')
        super().destroy_node()

    def __del__(self):
        """析构函数，确保文件关闭（作为额外保障）"""
        try:
            self.get_logger().info('析构函数被调用，关闭所有数据文件...')
            for file in self.category_files.values():
                file.close()
        except Exception as e:
            # 防止在节点未完全初始化时调用此方法
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
