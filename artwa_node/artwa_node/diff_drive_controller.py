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
        self.track_width = 0.72  # [m]
        self.max_pwm = 255  # 모터에 줄 수 있는 최대 제어 값

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
        x = int(self.latest_cmd_vel.linear.x)
        y = int(self.latest_cmd_vel.linear.x)

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
