import rclpy
from rclpy.node import Node
import serial
import struct
from geometry_msgs.msg import Twist


class UARTPublisher(Node):
    def __init__(self):
        super().__init__("uart_publisher")
        # UART 설정
        self.uart = serial.Serial("/dev/ttyTHS1", 115200, timeout=1)

        # cmd_vel 구독
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # 주기적으로 모터 제어 명령을 보내는 타이머
        self.timer = self.create_timer(0.25, self.send_message)

        # 최신 cmd_vel 메시지를 저장
        self.latest_cmd_vel = Twist()

        # differential drive 관련 파라미터
        self.track_width = 0.60  # [m] - README.md에 명시된 바퀴 사이 거리
        self.max_pwm = 255  # 모터에 줄 수 있는 최대 제어 값
        
        # 속도-모터 입력 변환 계수 (README.md 기반)
        self.a = 0.00443  # 선형 회귀 계수
        self.b = 0.13113  # 선형 회귀 계수
        
        # 최대 속도 제한 (m/s)
        self.max_velocity = 1.5  # 최대 속도 제한
        self.max_angular_velocity = 2.0  # 최대 각속도 제한 (rad/s)
        
        # 최소 회전 각속도 임계값 (모터 입력값 1에 해당하는 각속도)
        # v = a|u| + b = 0.00443 * |1| + 0.13113 = 0.13556 m/s
        # ω = 2v/L = 2 * 0.13556 / 0.60 = 0.45187 rad/s
        self.min_angular_velocity_threshold = 0.45  # rad/s

    def velocity_to_motor_input(self, velocity: float) -> int:
        """
        속도를 모터 입력값으로 변환
        README.md의 선형 모델 v = a|u| + b 공식을 역으로 풀어서 계산
        
        Args:
            velocity: 변환할 속도 값 (m/s)
        Returns:
            모터 입력값 (-255 ~ 255)
        """
        if abs(velocity) < 0.001:  # 매우 작은 속도는 0으로 처리
            return 0
        
        # 최대 속도 제한 적용
        sign_v = 1 if velocity > 0 else -1
        limited_velocity = sign_v * min(abs(velocity), self.max_velocity)
            
        # v = a|u| + b 공식을 역으로 풀어서 |u| = (v - b) / a
        abs_u = (abs(limited_velocity) - self.b) / self.a
        motor_input = int(sign_v * abs_u)
        
        # 모터 입력값 범위 제한
        return max(min(motor_input, self.max_pwm), -self.max_pwm)
    
    def angular_velocity_to_motor_input(self, angular_velocity: float) -> int:
        """
        각속도를 모터 입력값으로 변환
        README.md의 회전 모델 ω = 2v/L 공식에서 v = a|u| + b를 대입하여 역산
        
        Args:
            angular_velocity: 변환할 각속도 값 (rad/s)
        Returns:
            모터 입력값 (-255 ~ 255)
        """
        # 최소 각속도 임계값보다 작은 값은 무시
        # if abs(angular_velocity) < self.min_angular_velocity_threshold:
        #     return 0
        
        # 최대 각속도 제한 적용
        sign_omega = 1 if angular_velocity > 0 else -1
        limited_angular_velocity = sign_omega * min(abs(angular_velocity), self.max_angular_velocity)
        
        # ω = 2v/L 공식에서 v = a|u| + b를 대입하여 역산
        # ω = 2(a|u| + b)/L -> |u| = (ω*L/2 - b)/a
        abs_u = (abs(limited_angular_velocity) * self.track_width / 2 - self.b) / self.a
        motor_input = int(sign_omega * abs_u)
        
        # 모터 입력값 범위 제한
        return max(min(motor_input, self.max_pwm), -self.max_pwm)

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    def calculate_checksum(self, data: bytes) -> int:
        """Calculate checksum (start byte 제외 XOR 연산)."""
        checksum = 0
        for byte in data[1:]:  # start byte 제외하고 계산
            checksum ^= byte
        return checksum & 0xFF

    def send_message(self):
        start_byte = 0xAA  # ✅ 패킷 시작 바이트

        # 최신 cmd_vel 값으로부터 모터 제어 값을 계산
        # linear.x를 모터 입력값으로 변환 (직선 이동)
        motor_input = self.velocity_to_motor_input(self.latest_cmd_vel.linear.x)
        
        # angular.z를 모터 입력값으로 변환 (회전)
        angular_input = self.angular_velocity_to_motor_input(self.latest_cmd_vel.angular.z)
        
        # 양쪽 모터에 직선 이동과 회전을 합성
        x = motor_input - angular_input  # 왼쪽 모터
        y = motor_input + angular_input  # 오른쪽 모터 (회전 방향 반대)

        # 모터 입력값의 방향을 유지하면서 크기만 제한
        max_motor = max(abs(x), abs(y))
        if max_motor > self.max_pwm:
            scale = self.max_pwm / max_motor
            x = int(x * scale)
            y = int(y * scale)

        # ✅ 패킷 패키징 (little-endian: <Bhh -> 1+2+2 bytes)
        message = struct.pack("<Bhh", start_byte, x, y)

        # ✅ checksum 계산 (start byte 제외)
        checksum = self.calculate_checksum(message)

        # ✅ checksum 추가 (1 byte)
        message_with_checksum = message + struct.pack("B", checksum)

        # ✅ UART 송신
        self.uart.write(message_with_checksum)
        self.get_logger().info(
            f"Sent - start: {hex(start_byte)}, x: {x}, y: {y}, checksum: {checksum}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = UARTPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
