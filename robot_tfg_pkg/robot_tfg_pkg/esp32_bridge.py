import math
import rclpy
import serial
import struct
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu, MagneticField

# --- BINARY PROTOCOL CONSTANTS ---
START_BYTE = 0xAA
END_BYTE   = 0x55
ID_IMU     = 0x03
ID_LIDAR   = 0x04

class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_sensors')
        
        # Official ROS 2 Publishers
        self.pub_lidar = self.create_publisher(Range, '/sensor_distancia', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10) 

        # State machine variables
        self.rx_state = 'WAIT_START'
        self.rx_id = 0
        self.rx_len = 0
        self.rx_checksum = 0
        self.rx_buffer = bytearray()

        # Basic orientation variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Serial Connection
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.01)
            self.get_logger().info('Binary Connection with ESP32 Established')
        except Exception as e:
            self.get_logger().error(f'Error connecting to ESP32: {e}')
            self.ser = None

        # Very fast read timer (200 Hz) to prevent losing bytes
        self.timer = self.create_timer(0.005, self.read_serial)

        # Variables for gyroscope auto-calibration
        self.is_calibrating = True
        self.calibration_samples_gz = []
        self.gz_offset = 0.0
        self.MAX_SAMPLES = 200 # We will take 200 samples (~1 second at 200Hz)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def read_serial(self):
        if not self.ser: return
        
        waiting = self.ser.in_waiting
        while waiting > 0:
            # Read in blocks
            chunk = self.ser.read(waiting)
            
            for byte_in in chunk:
                # --- BINARY STATE MACHINE ---
                if self.rx_state == 'WAIT_START':
                    if byte_in == START_BYTE: 
                        self.rx_state = 'READ_ID'
                        
                elif self.rx_state == 'READ_ID':
                    self.rx_id = byte_in
                    self.rx_checksum = byte_in
                    self.rx_state = 'READ_LEN'
                    
                elif self.rx_state == 'READ_LEN':
                    self.rx_len = byte_in
                    self.rx_checksum ^= byte_in
                    self.rx_buffer = bytearray()
                    if self.rx_len > 40: # If length is unrealistic, discard
                        self.rx_state = 'WAIT_START'
                    else: 
                        self.rx_state = 'READ_DATA'
                        
                elif self.rx_state == 'READ_DATA':
                    self.rx_buffer.append(byte_in)
                    self.rx_checksum ^= byte_in
                    if len(self.rx_buffer) >= self.rx_len: 
                        self.rx_state = 'READ_CHK'
                        
                elif self.rx_state == 'READ_CHK':
                    if byte_in == self.rx_checksum: 
                        self.rx_state = 'WAIT_END'
                    else: 
                        self.rx_state = 'WAIT_START' # Checksum failed
                        
                elif self.rx_state == 'WAIT_END':
                    if byte_in == END_BYTE: 
                        self.process_packet(self.rx_id, self.rx_buffer)
                    self.rx_state = 'WAIT_START'

    def process_packet(self, packet_id, data):
        now = self.get_clock().now()
        
        # ==========================================
        # LiDAR PACKET (ID 0x04)
        # ==========================================
        if packet_id == ID_LIDAR and len(data) == 2:
            # '<H' means: Little-Endian (<), Unsigned Short Integer (H, 2 bytes)
            distance_cm = struct.unpack('<H', data)[0]
            distance_m = distance_cm / 100.0
            
            msg_range = Range()
            msg_range.header.stamp = now.to_msg()
            msg_range.header.frame_id = "tf_luna_link"
            msg_range.radiation_type = Range.INFRARED
            msg_range.min_range = 0.0
            msg_range.max_range = 4.0
            msg_range.range = distance_m
            
            self.pub_lidar.publish(msg_range)

        # ==========================================
        # IMU + MAGNETOMETER PACKET (ID 0x03)
        # ==========================================
        elif packet_id == ID_IMU and len(data) == 36:
            # '<9f' means: Little-Endian (<), 9 Floats (f, 4 bytes each = 36 bytes)
            values = struct.unpack('<9f', data)
            ax, ay, az, gx, gy, gz, mx, my, mz = values

            # ---------------------------------------
            # 1. AUTO-CALIBRATION PHASE (On startup)
            # ---------------------------------------
            if self.is_calibrating:
                self.calibration_samples_gz.append(gz)
                if len(self.calibration_samples_gz) >= self.MAX_SAMPLES:
                    # Calculate the average error at rest
                    self.gz_offset = sum(self.calibration_samples_gz) / self.MAX_SAMPLES
                    self.is_calibrating = False
                    self.get_logger().info(f'IMU Calibrated! Z Offset calculated: {self.gz_offset:.5f} rad/s')
                return # Do not publish odometry until calibration finishes
            
            # ---------------------------------------
            # 2. APPLY CALIBRATION
            # ---------------------------------------
            # Subtract the calculated systematic error
            gz_calibrated = gz - self.gz_offset

            # Calculate dt for the gyroscope
            dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
            self.last_time = now

            # Basic orientation calculations
            self.roll = math.atan2(ay, az)
            self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            
            # Now the dead zone can be tiny, only for white noise
            if abs(gz_calibrated) < 0.005: 
                gz_calibrated = 0.0
                
            self.yaw += gz_calibrated * dt
            # ---------------------------------------
            
            qx, qy, qz, qw = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

            # 1. Publish Standard IMU
            msg_imu = Imu()
            msg_imu.header.stamp = self.get_clock().now().to_msg()
            msg_imu.header.frame_id = "imu_link"
            
            msg_imu.orientation.x = qx
            msg_imu.orientation.y = qy
            msg_imu.orientation.z = qz
            msg_imu.orientation.w = qw
            
            msg_imu.linear_acceleration.x = ax
            msg_imu.linear_acceleration.y = ay
            msg_imu.linear_acceleration.z = az
            
            msg_imu.angular_velocity.x = gx
            msg_imu.angular_velocity.y = gy
            msg_imu.angular_velocity.z = gz_calibrated
            
            # --- Covariance bypass for RViz2 ---
            msg_imu.orientation_covariance[0] = -1.0
            msg_imu.linear_acceleration_covariance[0] = -1.0
            msg_imu.angular_velocity_covariance[0] = -1.0
            
            self.pub_imu.publish(msg_imu)

            # 2. Publish Standard Magnetometer
            msg_mag = MagneticField()
            msg_mag.header.stamp = self.get_clock().now().to_msg()
            msg_mag.header.frame_id = "imu_link"
            # ROS 2 expects Teslas, sensor provides microTeslas. Multiply by 1e-6
            msg_mag.magnetic_field.x = mx / 100.0 * 1e-6
            msg_mag.magnetic_field.y = my / 100.0 * 1e-6
            msg_mag.magnetic_field.z = mz / 100.0 * 1e-6
            
            self.pub_mag.publish(msg_mag)

def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
