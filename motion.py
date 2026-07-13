"""
motion 노드 (예선용: 2차선 유지 주행)

동작 개요
   - 차선 유지 : lane_detect가 준 steering_angle(아두이노 규격, -10~+10, +우/-좌)을 그대로 사용
   - 속도      : driving.ino가 속도 크기를 무시하고 부호(방향)만 사용하므로,
                 여기서는 항상 전진(양수 고정값)만 사용
   - 신호등    : 예선엔 신호등 무관하게 계속 주행하므로 관련 로직 제거
   - 차선 변경 : 예선엔 불필요하므로 상태 구조(스캐폴드)만 남기고 기본 비활성

입력
   - LaneInfo(/cam0/lane_info): lane_num, steering_angle, vehicle_position_x

출력
   - MotionCommand(motion_command): steering, left_speed, right_speed
     (left_speed/right_speed는 driving.ino가 부호만 사용하므로 방향 지시값)

튜닝 파라미터
   - max_steering_step : 방어적 클램프 (driving.ino / lane_detect 와 동일: 10)
   - enable_lane_change : 차선 변경 스캐폴드 사용 여부 (예선 기본 False)

※ base_speed / min_speed / curve_slowdown_k / diff_gain 은 driving.ino가
   속도 값의 크기(magnitude)를 무시하고 방향(부호)만 사용하도록 변경되어
   더 이상 하드웨어 동작에 영향을 주지 않으므로 삭제함.
"""

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import LaneInfo, MotionCommand


# driving.ino는 부호만 사용하므로, "전진"을 나타내는 양수 신호값
FORWARD_SIGNAL = 1


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # ---------------- Parameters ----------------
        self.declare_parameter(
            'max_steering_step',
            10
        )

        # 차선 변경 (예선 기본 비활성)
        self.declare_parameter(
            'enable_lane_change',
            False
        )

        # Inputs
        self.create_subscription(
            LaneInfo,
            '/cam0/lane_info',
            self.lane_info_callback,
            10
        )

        # Output
        self.motion_pub = self.create_publisher(
            MotionCommand,
            'motion_command',
            10
        )

        self.last_lane_info = None

        # 주행 상태
        self.state = 'LANE_KEEP'

        # ==================================================
        # 3초마다 확인할 최근 값
        # ==================================================
        self.latest_lane_num = None
        self.latest_vehicle_position_x = None
        self.latest_lane_steering = None

        self.latest_final_steering = None
        self.latest_left_speed = None
        self.latest_right_speed = None

        # LaneInfo 메시지 수신 횟수
        self.lane_info_count = 0

        # 3초마다 상태 출력
        self.create_timer(
            3.0,
            self.print_debug_status
        )

        self.get_logger().info('MotionNode initialized')

    # ---------------- Lane change scaffold ----------------
    def _lane_change_steer_bias(self):
        """
        차선 변경 스캐폴드 (예선 기본 비활성).
        """

        if not self.get_parameter(
            'enable_lane_change'
        ).value:
            return 0

        return 0

    # ---------------- Main ----------------
    def lane_info_callback(self, msg):
        self.last_lane_info = msg
        self.lane_info_count += 1

        # ==================================================
        # lane_detect에서 받은 값 저장
        # ==================================================
        self.latest_lane_num = msg.lane_num
        self.latest_vehicle_position_x = msg.vehicle_position_x
        self.latest_lane_steering = msg.steering_angle

        max_step = int(
            self.get_parameter(
                'max_steering_step'
            ).value
        )

        # 1) 차선 유지
        steering_before_clamp = (
            int(msg.steering_angle)
            + self._lane_change_steer_bias()
        )

        # -10 ~ +10 범위로 제한
        steering = max(
            -max_step,
            min(
                max_step,
                steering_before_clamp
            )
        )

        # 2) 항상 전진
        speed = FORWARD_SIGNAL

        cmd = MotionCommand()
        cmd.steering = steering
        cmd.left_speed = int(speed)
        cmd.right_speed = int(speed)

        self.motion_pub.publish(cmd)

        # ==================================================
        # 실제 publish한 최종값 저장
        # ==================================================
        self.latest_final_steering = cmd.steering
        self.latest_left_speed = cmd.left_speed
        self.latest_right_speed = cmd.right_speed

    def print_debug_status(self):
        """
        3초마다 lane_detect에서 받은 값과
        motion에서 최종 publish한 값을 출력한다.
        """

        if self.latest_lane_num is None:
            self.get_logger().warn(
                "\n"
                "=============== 3초 Motion 상태 ===============\n"
                "LaneInfo 수신 안 됨\n"
                "확인 명령:\n"
                "ros2 topic echo /cam0/lane_info\n"
                "==============================================="
            )
            return

        if self.latest_lane_num == 2:
            lane_2_status = 'YES'
        else:
            lane_2_status = 'NO'

        self.get_logger().warn(
            "\n"
            "=============== 3초 Motion 상태 ===============\n"
            f"LaneInfo 수신 횟수       : {self.lane_info_count}\n"
            f"인식된 lane 번호         : {self.latest_lane_num}\n"
            f"Lane 2 인식 여부         : {lane_2_status}\n"
            f"vehicle_position_x       : {self.latest_vehicle_position_x}\n"
            f"lane_detect 조향값       : {self.latest_lane_steering}\n"
            "-----------------------------------------------\n"
            f"motion 최종 조향값       : {self.latest_final_steering}\n"
            f"motion 왼쪽 속도         : {self.latest_left_speed}\n"
            f"motion 오른쪽 속도       : {self.latest_right_speed}\n"
            f"현재 상태                : {self.state}\n"
            "==============================================="
        )


def main(args=None):
    rclpy.init(args=args)

    node = MotionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'MotionNode interrupted'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
