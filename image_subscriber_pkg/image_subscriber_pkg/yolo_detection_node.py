#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import random
from ultralytics import YOLO

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.color_map = {}        
        # 订阅原始图像话题
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10)
        
        # 创建发布者，发布处理后的图像
        self.publisher = self.create_publisher(
            Image,
            '/dbg_image',
            10)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 加载YOLOv8模型
        self.get_logger().info('正在加载YOLOv8模型...')
        try:
            self.model = YOLO('yolo12m_best.pt')
            self.get_logger().info('YOLOv8模型加载成功')
        except Exception as e:
            self.get_logger().error(f'模型加载失败: {str(e)}')
            raise
    
    def image_callback(self, msg):
        """处理接收到的图像消息"""
        self.get_logger().debug('接收到图像')
        target_function = "tracking" #tracking

        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if target_function == 'objectDetection':
                # 使用YOLOv8模型进行推断
                results = self.model(cv_image)
                # 在图像上绘制检测结果
                annotated_image = results[0].plot()
                # 将处理后的图像转换回ROS消息格式
                output_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')

            elif target_function == 'tracking':
                results = self.model.track(cv_image, persist=True)
                
                # 確保有追蹤 ID
                if results[0].boxes.id is not None:
                    for box, track_id, cls in zip(results[0].boxes.xyxy, results[0].boxes.id, results[0].boxes.cls):
                        x1, y1, x2, y2 = map(int, box)
                        track_id = int(track_id)  # 確保 ID 是整數
                        class_name = self.model.names[int(cls)]  # 取得類別名稱
                        
                        # 如果該類別尚未指定顏色，則隨機生成一個
                        if class_name not in self.color_map:
                            self.color_map[class_name] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                        
                        color = self.color_map[class_name]  # 取得對應顏色
                        
                        label = f'ID {track_id} {class_name}'  # 物件 ID + 類別

                        # 畫出追蹤框
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # 将处理后的图像转换回ROS消息格式
                output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            output_msg.header = msg.header  # 保持原始消息的时间戳和帧ID
            
            # 发布处理后的图像
            self.publisher.publish(output_msg)
            self.get_logger().debug('已发布检测结果图像')
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'发生异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()