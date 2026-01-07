
# ====================
# 편집자 : 이원종, 정규민, 김민서
# 최종 수정일  : 2026-01-06
# 작업 상태 : 진행완료
# 역할 : 이미지 입력(USB 카메라/ROS Image) 통합 처리 + YOLOv8 추론 + 신호등 상태 토픽 발행 파이프라인 구축
# ====================

"""
ROS2 노드: 이미지 입력 + YOLOv8 추론 + 신호등 상태 퍼블리시 + 오버레이 이미지 퍼블리시

클래스(7):
["red","green","green and green arrow","green arrow","yellow","red and green arrow","unknown"]

퍼블리시:
- /traffic_light/red     (Bool)
- /traffic_light/green   (Bool)
- /traffic_light/orange  (Bool)
- /traffic_light/left    (Bool)
- /traffic_light/state   (String)
- /traffic_light/annotated (Image)
"""

import os
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String

from ultralytics import YOLO
try:
    import torch
    _HAS_TORCH = True
except Exception:
    _HAS_TORCH = False


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        # ───────────────────────── 입력/출력 파라미터 ─────────────────────────
        self.declare_parameter('image_in', '/camera1/image_raw')
        self.declare_parameter('image_out', '/traffic_light/annotated')
        self.declare_parameter('model_path', '/home/leewonjong/unita_ws_1_6/traffic_light.pt')
        self.declare_parameter('device_index', 4)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('show_window', False)

        image_in = self.get_parameter('image_in').get_parameter_value().string_value
        image_out = self.get_parameter('image_out').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        idx = int(self.get_parameter('device_index').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        # ───────────────────────── YOLO 설정 ─────────────────────────
        self.YOLO_CFG = {
            "model_path": model_path,
            "class_names": [
                "red",
                "green",
                "green and green arrow",
                "green arrow",
                "yellow",
                "red and green arrow",
                "unknown",
            ],
            "conf": 0.5,
            "iou": 0.45,
            "imgsz": 640,
        }

        if _HAS_TORCH and torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"

        self.bridge = CvBridge()

        # ───────────────────────── 입력 소스 설정 ─────────────────────────
        self.use_image_sub = bool(image_in)
        if self.use_image_sub:
            self.sub_img = self.create_subscription(Image, image_in, self.image_cb, 10)
        else:
            self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error(f'Cannot open camera index {idx} (e.g., /dev/video{idx})')
                raise RuntimeError('Camera open failed')

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        # ───────────────────────── 퍼블리셔 ─────────────────────────
        self.pub_img = self.create_publisher(Image, image_out, 10)
        self.pub_red = self.create_publisher(Bool, '/traffic_light/red', 10)
        self.pub_green = self.create_publisher(Bool, '/traffic_light/green', 10)
        self.pub_orange = self.create_publisher(Bool, '/traffic_light/orange', 10)
        self.pub_left = self.create_publisher(Bool, '/traffic_light/left', 10)
        self.pub_state = self.create_publisher(String, '/traffic_light/state', 10)

        # ───────────────────────── YOLO 로드 ─────────────────────────
        if not os.path.exists(self.YOLO_CFG["model_path"]):
            self.get_logger().error(f"YOLO model not found: {self.YOLO_CFG['model_path']}")
            raise FileNotFoundError(f"Missing model file: {self.YOLO_CFG['model_path']}")

        self.model = YOLO(self.YOLO_CFG["model_path"])

        self.class_names = self.YOLO_CFG["class_names"]
        self.cls_to_name = {i: name for i, name in enumerate(self.class_names)}

        # 시각화 색상
        self.color_map = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "yellow": (0, 255, 255),
            "green arrow": (255, 128, 0),
            "green and green arrow": (0, 200, 200),
            "red and green arrow": (200, 0, 200),
            "unknown": (180, 180, 180),
        }

        # UI
        self.window_name = 'traffic_light_node (press q to quit)'
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, width, height)

        # FPS
        self._t_last = time.time()
        self._fps_smooth = 0.0

        # 타이머(카메라 모드에서만 사용)
        if not self.use_image_sub:
            period = 1.0 / float(fps) if fps > 0 else 0.03
            self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            (f'Input: image_in={image_in}' if self.use_image_sub else f'Started camera: /dev/video{idx}, {width}x{height}@{fps}, show_window={self.show_window}') + "\n"
            f'YOLOv8: {self.YOLO_CFG["model_path"]}, device={self.device}, conf={self.YOLO_CFG["conf"]}, iou={self.YOLO_CFG["iou"]}\n'
            f'Image out: {image_out}'
        )

    # ───────────────────────── 클래스→신호 매핑 ─────────────────────────
    def _apply_mapping(self, name: str, present: dict):
        if name == "red":
            present["red"] = True
        elif name == "green":
            present["green"] = True
        elif name == "yellow":
            present["orange"] = True
        elif name in ("green arrow", "green and green arrow", "red and green arrow"):
            present["left"] = True

    def _draw_label(self, frame, x1, y1, text, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thick = 2
        y = max(20, y1 - 6)
        cv2.putText(frame, text, (x1, y), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, text, (x1, y), font, scale, color, thick, cv2.LINE_AA)

    def _draw_status_panel(self, frame, top_label, top_conf, present):
        font = cv2.FONT_HERSHEY_SIMPLEX
        panel_w = min(260, frame.shape[1] - 20)
        panel_h = 70
        x0, y0 = 10, 10
        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        state_color = self.color_map.get(top_label, (180, 180, 180))
        cv2.circle(frame, (x0 + 14, y0 + 20), 8, state_color, -1)

        if top_conf >= 0:
            label = f"{top_label} {top_conf:.2f}"
        else:
            label = "none"
        cv2.putText(frame, label, (x0 + 32, y0 + 25), font, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        items = [
            ("R", "red", (0, 0, 255)),
            ("G", "green", (0, 255, 0)),
            ("O", "orange", (0, 255, 255)),
            ("L", "left", (255, 128, 0)),
        ]
        bx = x0 + 10
        by = y0 + 38
        for ch, key, color in items:
            fill = color if present[key] else (60, 60, 60)
            cv2.rectangle(frame, (bx, by), (bx + 18, by + 18), fill, -1)
            cv2.rectangle(frame, (bx, by), (bx + 18, by + 18), (255, 255, 255), 1)
            txt_color = (0, 0, 0) if present[key] else (200, 200, 200)
            cv2.putText(frame, ch, (bx + 3, by + 14), font, 0.5, txt_color, 1, cv2.LINE_AA)
            bx += 22

    # ───────────────────────── 프레임 처리 ─────────────────────────
    def _handle_frame(self, frame, header=None):
        # (A) YOLO 추론
        t0 = time.time()
        results = self.model.predict(
            frame,
            imgsz=self.YOLO_CFG["imgsz"],
            conf=self.YOLO_CFG["conf"],
            iou=self.YOLO_CFG["iou"],
            device=self.device,
            verbose=False
        )
        infer_dt = time.time() - t0

        # (B) present 집계 + 최상위 state 선정
        present = {"red": False, "green": False, "orange": False, "left": False}
        top_label = "none"
        top_conf = -1.0

        if len(results) > 0:
            r0 = results[0]
            boxes = r0.boxes
            if boxes is not None and boxes.xyxy is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy().astype(np.int32)
                conf = boxes.conf.cpu().numpy()
                cls = boxes.cls.cpu().numpy().astype(int)

                for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                    name = self.cls_to_name.get(k, f"cls{k}")

                    # 매핑
                    self._apply_mapping(name, present)

                    # 최상위 라벨
                    if c > top_conf and name in self.class_names:
                        top_conf = c
                        top_label = name

                    # 바운딩박스
                    color = self.color_map.get(name, (0, 180, 255))
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(frame.shape[1] - 1, x2)
                    y2 = min(frame.shape[0] - 1, y2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"{name} {c:.2f}"
                    self._draw_label(frame, x1, y1, label, color)

        # (C) OSD
        t = time.time()
        dt = t - self._t_last
        self._t_last = t
        if dt > 0:
            inst = 1.0 / dt
            alpha = 0.1
            self._fps_smooth = (1 - alpha) * self._fps_smooth + alpha * inst if self._fps_smooth else inst

        cv2.putText(frame,
                    f"FPS:{self._fps_smooth:.1f}  INF:{(infer_dt*1000):.1f}ms",
                    (10, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2, cv2.LINE_AA)

        # (D) 상태 패널
        self._draw_status_panel(frame, top_label, top_conf, present)

        # (E) 이미지 퍼블리시 (오버레이 프레임)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
        self.pub_img.publish(msg)

        # (F) Bool/String 퍼블리시
        self.pub_red.publish(Bool(data=present["red"]))
        self.pub_green.publish(Bool(data=present["green"]))
        self.pub_orange.publish(Bool(data=present["orange"]))
        self.pub_left.publish(Bool(data=present["left"]))
        self.pub_state.publish(String(data=top_label))

        # (G) 미리보기
        if self.show_window:
            cv2.imshow(self.window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quit requested by user (q)')
                rclpy.shutdown()

    # ───────────────────────── 카메라 타이머 콜백 ─────────────────────────
    def timer_cb(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame grab failed')
            return
        self._handle_frame(frame)

    # ───────────────────────── 이미지 구독 콜백 ─────────────────────────
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge conversion failed: {e}')
            return
        self._handle_frame(frame, header=msg.header)

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()
            if self.show_window:
                try:
                    cv2.destroyWindow(self.window_name)
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
