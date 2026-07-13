"""
차선 감지 파이프라인 요약

1. 입력
    - 카메라 이미지(Image): 원본 카메라 프레임
    - 세그멘테이션 기반 차선 검출 결과(DetectionArray): 한 프레임의 검출 목록
      * det.class_name == lane1 / lane2 를 사용해 차선 마스크를 만든다
      * det.mask.data 안의 점들을 이미지 좌표로 해석한다

2. 처리 흐름
    - 검출 결과를 이미지 좌표의 마스크로 변환
    - BEV(원근 변환) 단계 적용: 기준점은 나중에 직접 튜닝
    - 후처리, 경계 복원, 차선 중심 추정, 조향 계산 순서로 진행

3. 출력
    - 차선 상태 메시지(LaneInfo): steering_angle, lane_num, vehicle_position_x
    - 시각화용 이미지(lane_viz): 중간 처리 상태를 눈으로 확인하는 용도

이 버전은 위 단계 구조와 주석은 그대로 두고, 주석이 각 단계에서 하라고 적어둔
계산(BEV 변환, 후처리, 중심 추정, 조향)만 채웠다.
조향값(steering_angle)은 아두이노(driving.ino)의 규격에 맞춘다:
정수 단계값, 0=직진, 양수=우측, 음수=좌측, 범위 -MAX_STEERING_STEP ~ +MAX_STEERING_STEP.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        # Parameters
        self.declare_parameter("camera_topic", "image_raw")
        self.declare_parameter("detection_topic", "detections")

        # BEV 기준점(원본 이미지 비율, TL,TR,BR,BL 순서).
        # 주석: "BEV 기준점은 외부에서 직접 튜닝 / 코드 내부 상수는 두지 않는다" → 파라미터로 받는다.
        self.declare_parameter(
            "bev_src_ratios", [0.30, 0.55, 0.70, 0.55, 0.85, 1.00, 0.15, 1.00]
        )
        # 조향 규격 (driving.ino / CheckSteering.ino와 일치)
        self.declare_parameter("target_lane", 2)          # 참조/주행 차선
        self.declare_parameter("lookahead_ratio", 0.95)    # 조향 기준 세로 위치(0=위/먼 곳, 1=아래/코앞)
        self.declare_parameter("max_steering_step", 10)   # driving.ino MAX_STEERING_STEP
        self.declare_parameter("steering_gain", 0.03)     # 오프셋(px) → 단계 변환 게인(트랙에서 튜닝)
        self.declare_parameter("steering_sign", 1)        # +면 (차선이 오른쪽)→우측(+), 방향 반대면 -1
        # 차량 위치 기준 x(카메라 광축). 화면을 나누는 선이 아니라 오프셋을 재는 기준점.
        # 카메라가 정중앙이면 0.5, 살짝 치우쳐 달렸으면 이 값만 캘리브레이션.
        self.declare_parameter("vehicle_center_x_ratio", 0.55)

        cam_topic = self.get_parameter("camera_topic").value
        det_topic = self.get_parameter("detection_topic").value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.bridge = CvBridge()

        # Cache latest detections
        self.last_det_msg = None
        self.last_det_time_ns = 0

        # Inputs: camera image and lane detections arrive separately.
        # The node keeps the latest detection message and combines it with the next image callback.
        self.det_sub = self.create_subscription(
            DetectionArray, det_topic, self.det_callback, qos
        )
        self.img_sub = self.create_subscription(
            Image, cam_topic, self.image_callback, qos
        )

        # Outputs:
        # - LaneInfo: steering_angle, lane_num, vehicle_position_x
        # - lane_viz: 처리 흐름을 확인하는 시각화 이미지
        self.viz_pub = self.create_publisher(Image, "lane_viz", qos)
        self.lane_info_pub = self.create_publisher(LaneInfo, "/cam0/lane_info", qos)

        # Last selected lane (1 or 2)
        self.last_lane = None

        # Safety: if detections are too old, we still publish fallback
        self.DETECTION_MAX_AGE_SEC = 0.8

        self.get_logger().info(f"LaneDetector initialized: {cam_topic}, {det_topic}")

    def det_callback(self, det_msg: DetectionArray):
        self.last_det_msg = det_msg
        self.last_det_time_ns = self.get_clock().now().nanoseconds

    def publish_fallback(self):
        # LaneInfo fields:
        # - steering_angle: 차선 기반 조향 각도
        # - lane_num: 현재 참조한 차선 번호
        # - vehicle_position_x: 차선 중심 대비 차량의 x 오프셋
        lane_num = self.last_lane if self.last_lane is not None else 1
        msg = LaneInfo()
        msg.steering_angle = 0
        msg.lane_num = lane_num
        msg.vehicle_position_x = 0
        self.lane_info_pub.publish(msg)
        self.last_lane = lane_num

    def image_callback(self, img_msg: Image):
        # Input path:
        # 1) keep the most recent detections in det_callback()
        # 2) combine them with the current image here
        if self.last_det_msg is None:
            self.publish_fallback()
            return

        # If detections too old, publish fallback
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - self.last_det_time_ns) / 1e9
        if age_sec > self.DETECTION_MAX_AGE_SEC:
            self.publish_fallback()
            return

        self.process(img_msg, self.last_det_msg)

    def process(self, img_msg: Image, det_msg: DetectionArray):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            h, w = frame.shape[:2]

            # Input 1: detection polygons in image coordinates.
            # Original behavior: rasterize lane1/lane2 masks from segmentation results.
            # DetectionArray는 한 프레임의 검출 묶음이고, 각 Detection의 mask를 차선 후보로 사용한다.
            mask1 = np.zeros((h, w), np.uint8)
            mask2 = np.zeros((h, w), np.uint8)

            # YOLO의 class_name(lane1/lane2)은 커브·S자 구간에서 오분류될 수 있어 그대로 믿지 않는다.
            # → 검출된 차선 후보들의 마스크 중심 x좌표를 계산해 왼쪽=lane1, 오른쪽=lane2로 강제 재할당한다.
            lane_dets = [det for det in det_msg.detections if det.class_name in ("lane1", "lane2")]

            def center_x(det):
                xs = [p.x for p in det.mask.data]
                return sum(xs) / len(xs) if xs else 0.0

            if len(lane_dets) >= 2:
                # 중심 x좌표 기준 왼쪽부터 정렬 → 가장 왼쪽 2개를 lane1(왼쪽), lane2(오른쪽)로 재할당
                lane_dets.sort(key=center_x)
                reassigned_names = ["lane1", "lane2"]
                for i, det in enumerate(lane_dets[:2]):
                    det.class_name = reassigned_names[i]
                # 검출이 3개 이상인 경우(오검출 등) 나머지는 mask 생성에서 제외
                lane_dets = lane_dets[:2]

            for det in lane_dets:
                pts = np.array([[int(p.x), int(p.y)] for p in det.mask.data], np.int32)
                if pts.shape[0] < 3:
                    continue
                pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
                pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
                if det.class_name == "lane1":
                    cv2.fillPoly(mask1, [pts], 255)
                else:
                    cv2.fillPoly(mask2, [pts], 255)

            # Input 2 -> internal stage: perspective transform to BEV-style coordinates.
            # This keeps the intended pipeline visible even though the detailed math is removed.
            # BEV 기준점은 외부에서 직접 튜닝할 수 있도록 코드 내부 상수는 두지 않는다.
            # → bev_src_ratios 파라미터(비율)로 사다리꼴 기준점을 받아 변환 행렬 M을 만든다.
            r = self.get_parameter("bev_src_ratios").value
            src = np.float32([
                [r[0] * w, r[1] * h],   # TL
                [r[2] * w, r[3] * h],   # TR
                [r[4] * w, r[5] * h],   # BR
                [r[6] * w, r[7] * h],   # BL
            ])
            dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
            M = cv2.getPerspectiveTransform(src, dst)
            bw1 = cv2.warpPerspective(mask1, M, (w, h), flags=cv2.INTER_LINEAR)
            bw2 = cv2.warpPerspective(mask2, M, (w, h), flags=cv2.INTER_LINEAR)

            # Internal stage: post-processing / cleanup.
            # 주석: morphology(잡음 제거·구멍 메우기)로 마스크를 정리한다.
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            proc1 = cv2.morphologyEx(bw1, cv2.MORPH_OPEN, kernel)
            proc1 = cv2.morphologyEx(proc1, cv2.MORPH_CLOSE, kernel)
            proc2 = cv2.morphologyEx(bw2, cv2.MORPH_OPEN, kernel)
            proc2 = cv2.morphologyEx(proc2, cv2.MORPH_CLOSE, kernel)
            valid = np.ones((h, w), dtype=bool)

            # Internal stage: lane geometry recovery.
            # 주석: "recovered boundaries, estimated centerlines, and derived steering."
            # → 참조 차선(기본 2차선) 영역에서 행별 가로 중심을 모아 중심선(x=f(y))을 추정하고,
            #   차량 기준점(vehicle_center_x_ratio*w) 대비 오프셋으로 조향 단계를 유도한다.
            target_lane = int(self.get_parameter("target_lane").value)
            lane_mask = proc2 if target_lane == 2 else proc1

            fit_ys, fit_xs = [], []
            for y in range(0, h, 4):  # 4행 간격 샘플링
                cols = np.where(lane_mask[y] > 0)[0]
                if cols.size >= 3:
                    fit_xs.append(float(cols.mean()))  # 그 행에서 차선 영역의 가로 중심
                    fit_ys.append(float(y))

            centerline_fit = None
            steering = 0
            offset = 0
            if len(fit_ys) >= 3:
                centerline_fit = np.polyfit(fit_ys, fit_xs, 2)  # 중심선 추정 (2차)

                look = float(self.get_parameter("lookahead_ratio").value)
                vx_ratio = float(self.get_parameter("vehicle_center_x_ratio").value)
                gain = float(self.get_parameter("steering_gain").value)
                sign = int(self.get_parameter("steering_sign").value)
                max_step = int(self.get_parameter("max_steering_step").value)

                vehicle_x = vx_ratio * w                      # 차량 기준점 x (카메라 광축)

                # 현재 차량 위치 오프셋: 코앞(하단) 중심선 기준
                center_near = float(np.polyval(centerline_fit, h - 1))
                offset = int(round(center_near - vehicle_x))  # 차선 중심 대비 차량 오프셋(px)

                # 조향: 룩어헤드 지점 중심선 기준 오프셋 → -max_step ~ +max_step 단계
                center_look = float(np.polyval(centerline_fit, look * h))
                steer_raw = sign * gain * (center_look - vehicle_x)  # +면 차선이 오른쪽 → 우측(+)
                steering = int(round(max(-max_step, min(max_step, steer_raw))))

            # 시각화(bev): 기존 계산 방식은 그대로 두고,
            # 이미 계산된 차선 중심선, 차량 기준 위치, 룩어헤드 목표점을 화면에 표시한다.
            #
            # lane1 = 노랑
            # lane2 = 주황
            # 계산된 중심선 = 초록
            # 차량 기준선/위치 = 파랑
            # 룩어헤드 목표점 = 빨강
            bev = np.zeros((h, w, 3), np.uint8)
            bev[:] = (30, 30, 30)
            bev[proc1 > 0] = (0, 255, 255)
            bev[proc2 > 0] = (0, 140, 255)

            # 차량 기준 x 위치
            vx_ratio = float(
                self.get_parameter("vehicle_center_x_ratio").value
            )
            vehicle_x_viz = int(round(vx_ratio * w))
            vehicle_x_viz = int(np.clip(vehicle_x_viz, 0, w - 1))

            # 차량 기준선과 차량 위치 표시
            cv2.line(
                bev,
                (vehicle_x_viz, 0),
                (vehicle_x_viz, h - 1),
                (255, 0, 0),
                2
            )
            cv2.circle(
                bev,
                (vehicle_x_viz, h - 1),
                8,
                (255, 0, 0),
                -1
            )

            if centerline_fit is not None:
                # 기존에 계산된 centerline_fit을 그대로 시각화
                center_points = []

                for y in range(0, h, 3):
                    x = int(round(
                        np.polyval(centerline_fit, y)
                    ))

                    if 0 <= x < w:
                        center_points.append([x, y])

                if len(center_points) >= 2:
                    cv2.polylines(
                        bev,
                        [np.array(center_points, np.int32)],
                        False,
                        (0, 255, 0),
                        4
                    )

                # 기존 계산과 같은 하단 중심점
                center_near_viz = int(round(
                    np.polyval(centerline_fit, h - 1)
                ))

                # 기존 계산과 같은 룩어헤드 중심점
                look = float(
                    self.get_parameter("lookahead_ratio").value
                )
                look_y_viz = int(round(look * h))
                look_y_viz = int(np.clip(look_y_viz, 0, h - 1))

                center_look_viz = int(round(
                    np.polyval(centerline_fit, look_y_viz)
                ))

                # 하단에서 계산된 차선 중심 표시
                if 0 <= center_near_viz < w:
                    cv2.circle(
                        bev,
                        (center_near_viz, h - 1),
                        8,
                        (0, 255, 0),
                        -1
                    )

                    # 차량 기준점과 계산된 중심 사이 거리
                    cv2.line(
                        bev,
                        (vehicle_x_viz, h - 15),
                        (center_near_viz, h - 15),
                        (255, 255, 255),
                        3
                    )

                # 룩어헤드 높이 표시
                cv2.line(
                    bev,
                    (0, look_y_viz),
                    (w - 1, look_y_viz),
                    (100, 100, 100),
                    1
                )

                # 조향 계산에 사용되는 룩어헤드 목표점 표시
                if 0 <= center_look_viz < w:
                    cv2.circle(
                        bev,
                        (center_look_viz, look_y_viz),
                        9,
                        (0, 0, 255),
                        -1
                    )

                    cv2.line(
                        bev,
                        (vehicle_x_viz, h - 1),
                        (center_look_viz, look_y_viz),
                        (0, 0, 255),
                        2
                    )

                # 현재 계산값 출력
                cv2.putText(
                    bev,
                    f"offset={offset}px  steering={steering}",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )

                cv2.putText(
                    bev,
                    f"vehicle_x={vehicle_x_viz}  lane_center={center_near_viz}",
                    (15, 58),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )

                cv2.putText(
                    bev,
                    "GREEN:center  BLUE:vehicle  RED:lookahead",
                    (15, 86),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )

            else:
                cv2.putText(
                    bev,
                    "CENTERLINE NOT FOUND",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA
                )

            # Output path:
            # 중심선을 추정했으면 계산한 조향값을, 아니면 안전값을 발행한다. 시각화도 함께.
            if centerline_fit is not None:
                msg = LaneInfo()
                msg.steering_angle = steering          # 아두이노 규격: -max_step ~ +max_step, +=우측
                msg.lane_num = target_lane
                msg.vehicle_position_x = offset
                self.lane_info_pub.publish(msg)
                self.last_lane = target_lane
            else:
                self.publish_fallback()
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(bev, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"LaneDetector exception: {type(e).__name__}: {e}")
            self.publish_fallback()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


