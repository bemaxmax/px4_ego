# 这是一个目标检测节点，使用 YOLO 模型在 RGB 图像上检测目标
# 并结合深度图和里程计数据计算目标在世界坐标系下的位置。
# 检测到新的目标会被保存为图片，并记录其世界坐标的xy值坐标（忽略z轴）以避免重复保存。
# 每次执行前会清空saved_targets目录，务必检查是否清空！！
import cv2
import json
import message_filters
import numpy as np
import rclpy
import requests
import torch
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from pathlib import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf_transformations import euler_matrix, quaternion_matrix
from ultralytics import YOLO


class TargetDetectorNode(Node):
    # 初始化模型、缓存状态以及订阅接口。
    def __init__(self):
        super().__init__('target_detector')

        # 当前仿真环境下固定使用这一组相机内外参。
        self.rgb_intrinsics = np.array([1397.2234694239507, 1397.2234694239507, 959.5, 539.5])
        self.depth_intrinsics = np.array([432.496042035043, 432.49604203504293, 319.5, 239.5])
        self.camera_to_base_translation = np.array([0.16233, 0.0, 0.22878])
        self.camera_to_base_rotation = euler_matrix(0.0, 0.0, 0.0)[:3, :3]

        self.model = YOLO('/home/ubuntu22/px4_ego/src/detect/models/best.pt')
        #判断是否为GPU环境，若是则使用GPU加速，否则使用CPU。
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.bridge = CvBridge()

        self.world_position = self.world_rotation = None
        self.has_odom = False
        self.saved_targets = {}
        self.save_dir = Path('/home/ubuntu22/px4_ego/saved_targets')
        self.n8n_webhook_url = 'http://localhost:5678/webhook/new_targets'
        self.save_dir.mkdir(exist_ok=True)
        
        # 启动前清空保存目录，避免之前的图片和记录干扰当前测试。
        for path in self.save_dir.iterdir():
            path.unlink()
            self.get_logger().info('已清空文件夹，等待节点启动...')

        self.create_subscription(Odometry, '/ego/odom_world', self.odom_callback, qos_profile_sensor_data)
        self.detect_image_pub = self.create_publisher(CompressedImage, '/detect/image/compressed', 10)
        self.detect_result_pub = self.create_publisher(String, '/detect/targets', 10)
        # 使用 message_filters 同步 RGB 和深度图，同步容忍度设置为20毫秒。
        self.rgb_sub = message_filters.Subscriber(self,Image,'/rgb_camera',qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self,Image,'/depth_camera_bestef',qos_profile_sensor_data)
        self.rgb_depth_sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub],10,0.02)
        self.rgb_depth_sync.registerCallback(self.rgb_depth_callback)

    # 缓存机体在世界坐标系下的位置和姿态。
    def odom_callback(self, odom_msg):
        self.world_position = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
        ])

        q = odom_msg.pose.pose.orientation
        self.world_rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        self.has_odom = True
    
    # 根据类别名、平面坐标和图像内容保存目标图片，并返回对应记录。
    def update_target_image(self, target_id, cls_name, point_xy, image_bgr, center_distance, bbox, conf, image_path=None):
        # 首次保存时按 target_id、类别、平面 xy 和置信度命名，后续同一目标只覆盖这张图。
        if image_path is None:
            image_name = (
                f'{target_id}_{cls_name}_x_{point_xy[0]:.2f}_y_{point_xy[1]:.2f}_conf_{conf:.1f}'
                .replace('/', '_')
                .replace(' ', '_')
                + '.jpg'
            )
            image_path = self.save_dir / image_name

        save_image = image_bgr.copy()
        x1, y1, x2, y2 = (int(round(value)) for value in bbox.tolist())
        cv2.rectangle(save_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imwrite(str(image_path), save_image)
        return {
            'target_id': target_id,
            'point_xy': point_xy.copy(),
            'center_distance': center_distance,
            'image_path': image_path,
        }

    # 检测到新目标发给n8n。
    def notify_n8n_new_target(self, target_id, cls_name, image_path):
        with image_path.open('rb') as image_file:
            try:
                response = requests.post(
                    self.n8n_webhook_url,
                    data={
                        'target_id': target_id,
                        'category': cls_name,
                    },
                    files={
                        'data': (image_path.name, image_file, 'image/jpeg'),
                    },
                    timeout=3.0,
                )
                response.raise_for_status()
            except requests.RequestException as exc:
                self.get_logger().warning(f'n8n webhook 调用失败: {exc}')

    # 同类目标与已保存坐标太近就视为重复；如果新目标更靠近画面中心，就更新保存图片。
    def save_target_if_new(self, cls_name, point_in_world, image_bgr, u_rgb, v_rgb, bbox, conf):
        point_xy = point_in_world[:2]
        center_distance = np.linalg.norm(np.array([
            u_rgb - image_bgr.shape[1] * 0.5,
            v_rgb - image_bgr.shape[0] * 0.5,
        ]))
        saved_targets = self.saved_targets.setdefault(cls_name, [])
        for saved_target in saved_targets:
            # 平面坐标距离超过0.7米就认为是不同目标，继续搜索。
            if np.linalg.norm(point_xy - saved_target['point_xy']) >= 0.7:
                continue
            # 距离中心位置变化小于20像素，不更新。
            if saved_target['center_distance'] - center_distance < 20.0:
                return

            saved_target.update(self.update_target_image(saved_target['target_id'], cls_name, point_xy, image_bgr, center_distance, bbox, conf, saved_target['image_path']))
            # self.get_logger().info(f'目标 {cls_name} 更靠近画面中心，已更新图片，平面坐标: {point_xy}')
            return

        #设置target_id为全局递增编号
        target_id = str(sum(len(targets) for targets in self.saved_targets.values()) + 1)
        saved_target = self.update_target_image(target_id, cls_name, point_xy, image_bgr, center_distance, bbox, conf)
        saved_targets.append(saved_target)
        self.notify_n8n_new_target(saved_target['target_id'], cls_name, saved_target['image_path'])
        self.get_logger().info(f'检测到新目标 {cls_name}，target_id: {saved_target["target_id"]}，平面坐标: {point_xy}')

    # 发布压缩图像和检测结果(json格式)。
    def publish_detect_outputs(self, image_msg, detect_image, detections):
        ok, encoded_image = cv2.imencode('.jpg', detect_image)
        if ok:
            detect_image_compressed = CompressedImage()
            detect_image_compressed.header = image_msg.header
            detect_image_compressed.format = 'jpeg'
            detect_image_compressed.data = encoded_image.tobytes()
            self.detect_image_pub.publish(detect_image_compressed)

        detect_result_msg = String()
        detect_result_msg.data = json.dumps({
            'stamp': {
                'sec': image_msg.header.stamp.sec,
                'nanosec': image_msg.header.stamp.nanosec,
            },
            'frame_id': image_msg.header.frame_id,
            'image_width': int(detect_image.shape[1]),
            'image_height': int(detect_image.shape[0]),
            'detections': detections,
        }, ensure_ascii=False)
        self.detect_result_pub.publish(detect_result_msg)

    # 用同步后的 RGB 和深度图做检测，并逐个计算这一帧里所有目标的世界坐标。
    def rgb_depth_callback(self, image_msg, depth_msg):
        if not self.has_odom:
            self.get_logger().warning('没有里程计数据，无法进行目标检测。')
            return

        image_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        result = self.model.predict(
            source=image_bgr,
            verbose=False,
            conf=0.5,
            device=self.device,
        )[0]

        detect_image = result.plot()

        detections = []
        boxes = result.boxes
        fx, fy, cx, cy = self.rgb_intrinsics
        depth_fx, depth_fy, depth_cx, depth_cy = self.depth_intrinsics

        if boxes is not None:
            for box in boxes:
                bbox = box.xyxy[0]
                cls = int(box.cls.item())
                cls_name = result.names[cls]
                conf = float(box.conf.item())

                # 用检测框中心点代表目标像素，并先在 RGB 光学坐标系里构造视线。
                u_rgb = float(((bbox[0] + bbox[2]) * 0.5).item())
                v_rgb = float(((bbox[1] + bbox[3]) * 0.5).item())
                ray_x = (u_rgb - cx) / fx
                ray_y = (v_rgb - cy) / fy

                detection = {
                    'class_name': cls_name,
                    'confidence': conf,
                    'bbox': [float(value) for value in bbox.tolist()],
                    'center_uv': [u_rgb, v_rgb],
                }

                depth_u = int(round(depth_fx * ray_x + depth_cx))
                depth_v = int(round(depth_fy * ray_y + depth_cy))
                if (
                    0 <= depth_u < depth_image.shape[1] and
                    0 <= depth_v < depth_image.shape[0]
                ):
                    depth_value = float(depth_image[depth_v, depth_u])
                    if np.isfinite(depth_value) and depth_value > 0.0:
                        # ROS optical frame 是 x 右、y 下、z 前；当前 Gazebo 相机链路是 x 前、y 左、z 上。
                        point_in_base = self.camera_to_base_rotation @ np.array([
                            depth_value,
                            -ray_x * depth_value,
                            -ray_y * depth_value,
                        ]) + self.camera_to_base_translation

                        point_in_world = self.world_rotation @ point_in_base + self.world_position
                        detection['world'] = [float(value) for value in point_in_world.tolist()]
                        detection['depth'] = depth_value
                        self.save_target_if_new(cls_name, point_in_world, image_bgr, u_rgb, v_rgb, bbox, conf)

                detections.append(detection)

        self.publish_detect_outputs(image_msg, detect_image, detections)


# 启动目标识别节点。
def main():
    rclpy.init()
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
