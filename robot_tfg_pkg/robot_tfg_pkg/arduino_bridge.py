import math
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Int16MultiArray

# --- BINARY PROTOCOL CONSTANTS ---
START_BYTE = 0xAA
END_BYTE   = 0x55
ID_ENCODERS = 0x02
ID_CMD_MOTOR_SERVO = 0x10

DIR_BACKWARD = 0
DIR_FORWARD  = 1
DIR_RELEASE  = 2

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # 1. SUBSCRIBER TO CONTROLLER (Receives signed PWMs: e.g., [150, -150])
        self.sub_pwm = self.create_subscription(Int16MultiArray, '/pwm_setpoints', self.pwm_callback, 10)
        
        # 2. STANDARD ENCODERS PUBLISHER
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        
        # 3. SUBSCRIBERS FOR SERVOS
        self.sub_camera = self.create_subscription(Int32, '/set_camera_angle', self.camera_callback, 10)
        self.sub_lidar = self.create_subscription(Int32, '/set_lidar_angle', self.lidar_callback, 10)
        
        # Physical variables
        self.dir_left = DIR_RELEASE
        self.pwm_left = 0
        self.dir_right = DIR_RELEASE
        self.pwm_right = 0
        self.camera_angle = 90
        self.lidar_angle = 90
        self.ticks_to_rads_sec = math.pi
        
        # Communication variables
        self.rx_state = 'WAIT_START'
        self.rx_id = 0
        self.rx_len = 0
        self.rx_checksum = 0
        self.rx_buffer = []

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
            self.get_logger().info('"Dumb" Arduino Bridge (Hardware Level) Started')
        except Exception as e:
            self.get_logger().error(f'Connection error: {e}')
            self.ser = None

        self.timer_read = self.create_timer(0.01, self.read_serial)
        self.timer_send = self.create_timer(0.1, self.send_packet)

    # ========================================================
    # RECEIVE INSTRUCTIONS FROM THE BRAIN
    # ========================================================
    def pwm_callback(self, msg):
        if len(msg.data) == 2:
            val_left = msg.data[0]
            val_right = msg.data[1]

            # Left Wheel
            if val_left > 0:
                self.dir_left = DIR_FORWARD
                self.pwm_left = min(255, val_left)
            elif val_left < 0:
                self.dir_left = DIR_BACKWARD
                self.pwm_left = min(255, abs(val_left))
            else:
                self.dir_left = DIR_RELEASE
                self.pwm_left = 0

            # Right Wheel
            if val_right > 0:
                self.dir_right = DIR_FORWARD
                self.pwm_right = min(255, val_right)
            elif val_right < 0:
                self.dir_right = DIR_BACKWARD
                self.pwm_right = min(255, abs(val_right))
            else:
                self.dir_right = DIR_RELEASE
                self.pwm_right = 0

    def camera_callback(self, msg):
        self.camera_angle = max(0, min(180, msg.data))

    def lidar_callback(self, msg):
        self.lidar_angle = max(0, min(180, msg.data))

    # ========================================================
    # SERIAL COMMUNICATION
    # ========================================================
    def send_packet(self):
        if not self.ser: return
        data = [
            self.dir_left, self.pwm_left, 
            self.dir_right, self.pwm_right, 
            self.camera_angle, self.lidar_angle
        ]
        length = len(data)
        checksum = ID_CMD_MOTOR_SERVO ^ length
        for b in data: checksum ^= b
        packet = bytes([START_BYTE, ID_CMD_MOTOR_SERVO, length] + data + [checksum, END_BYTE])
        self.ser.write(packet)

    def read_serial(self):
        if not self.ser: return
        while self.ser.in_waiting > 0:
            byte_in = self.ser.read(1)[0]
            if self.rx_state == 'WAIT_START':
                if byte_in == START_BYTE: self.rx_state = 'READ_ID'
            elif self.rx_state == 'READ_ID':
                self.rx_id = byte_in; self.rx_checksum = byte_in; self.rx_state = 'READ_LEN'
            elif self.rx_state == 'READ_LEN':
                self.rx_len = byte_in; self.rx_checksum ^= byte_in; self.rx_buffer = []
                if self.rx_len > 20: self.rx_state = 'WAIT_START'
                else: self.rx_state = 'READ_DATA'
            elif self.rx_state == 'READ_DATA':
                self.rx_buffer.append(byte_in); self.rx_checksum ^= byte_in
                if len(self.rx_buffer) >= self.rx_len: self.rx_state = 'READ_CHK'
            elif self.rx_state == 'READ_CHK':
                if byte_in == self.rx_checksum: self.rx_state = 'WAIT_END'
                else: self.rx_state = 'WAIT_START'
            elif self.rx_state == 'WAIT_END':
                if byte_in == END_BYTE: self.process_packet(self.rx_id, self.rx_buffer)
                self.rx_state = 'WAIT_START'

    def process_packet(self, packet_id, data):
        if packet_id == ID_ENCODERS and len(data) == 4:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
            msg.velocity = [
                float(data[0]) * self.ticks_to_rads_sec, 
                float(data[1]) * self.ticks_to_rads_sec, 
                float(data[2]) * self.ticks_to_rads_sec, 
                float(data[3]) * self.ticks_to_rads_sec
            ]
            self.pub_joints.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
