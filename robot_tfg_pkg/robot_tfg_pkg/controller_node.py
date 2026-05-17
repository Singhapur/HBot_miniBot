import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState, Imu
from sensor_msgs.msg import Range

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.use_imu = True
        
        # Subscriptions
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.sub_lidar = self.create_subscription(Range, '/sensor_distancia', self.distance_callback, 10)
        
        # Publisher
        self.pub_pwm = self.create_publisher(Int16MultiArray, '/pwm_setpoints', 10)
        
        # Physical parameters
        self.wheelbase = 0.145
        self.max_speed_ms = 0.5

        # Smooth acceleration
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        # Smooth acceleration variables 
        self.max_accel_v = 0.8 # m/s
        self.max_accel_w = 2.0 # rads/s
        self.dt = 0.1 # timer runs at 0.1 segundos
        
        # IMU variables & sensor variables
        self.imu_yaw_rate = 0.0
        self.current_distance = 100.0 # Meter
        self.min_distance = 0.15 # M  
        
        # State variables
        self.base_pwm_left = 0
        self.base_pwm_right = 0
        self.dir_left = 2 # 2=Release, 1=Forward, 0=Backward
        self.dir_right = 2
        self.straight_line_mode = False
        self.last_cmd_time = self.get_clock().now()
        self.last_pwm_left = 0
        self.last_pwm_right = 0
        
        
        # Memory for the Integral term
        self.error_fwd = 0.0
        self.error_bwd = 0.0
        
        # Timer to send PWM commands constantly
        self.timer = self.create_timer(0.1, self.publish_pwm)
        
        # PD Controller parameters
        self.kp = 4.5
        self.ki = 1.0
        
        self.get_logger().info('Trajectory PID Controller Started')
        
    def distance_callback(self, msg):
        self.current_distance = msg.range
       
    def imu_callback(self, msg):
        # angular_velocity.z indicates rotation speed in rad/s
        self.imu_yaw_rate = msg.angular_velocity.z

    def joint_callback(self, msg):
        # Apply PID if going in a straight line
        if self.straight_line_mode:
            if self.use_imu:
                # IMU error. Inverted to correct left/right drift
                error = -self.imu_yaw_rate * 10.0
            else:
                rl_rads = abs(msg.velocity[2])
                rr_rads = abs(msg.velocity[3])
                
                # Encoder error
                error = float(rl_rads) - float(rr_rads)
            
            integral_term = 0.0
            
            # Save error for Anti-Windup
            if self.dir_left == 1 and self.dir_right == 1:
                self.error_fwd += error
                self.error_fwd = max(-50.0, min(50.0, self.error_fwd))
                integral_term = self.error_fwd
                
            elif self.dir_left == 0 and self.dir_right == 0:
                self.error_bwd += error
                self.error_bwd = max(-50.0, min(50.0, self.error_bwd))
                integral_term = self.error_bwd

            # Correction Calculation: Proportional + Integral
            correction = int((error * self.kp) + (integral_term * self.ki))
            
            # Apply correction to the base PWM
            if self.last_pwm_left != 0 and self.last_pwm_right != 0:
                self.base_pwm_left = self.last_pwm_left - correction
                self.base_pwm_right = self.last_pwm_right + correction
            else:
                self.base_pwm_left -= correction
                self.base_pwm_right += correction
            
            self.base_pwm_left = max(95, min(255, self.base_pwm_left))
            self.last_pwm_left = self.base_pwm_left       
            self.base_pwm_right = max(95, min(255, self.base_pwm_right))
            self.last_pwm_right = self.base_pwm_right

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        v = msg.linear.x
        w = msg.angular.z
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

        if self.current_distance <= self.min_distance and self.target_v > 0:
            self.target_v = 0.0
            self.target_w = 0.0
            self.current_v = 0.0
            self.current_w = 0.0

    def assign_vel(self):
        # Smooth acceleration logic
        # Calculamos la diferencia entre lo que queremos y lo que tenemos
        dv = self.target_v - self.current_v
        dw = self.target_w - self.current_w
        
        # Cuánto podemos cambiar como máximo en este ciclo (0.1s)
        max_step_v = self.max_accel_v * self.dt
        max_step_w = self.max_accel_w * self.dt
        
        # Aplicar rampa lineal
        if abs(dv) > max_step_v:
            self.current_v += max_step_v if dv > 0 else -max_step_v
        else:
            self.current_v = self.target_v
            
        # Aplicar rampa angular
        if abs(dw) > max_step_w:
            self.current_w += max_step_w if dw > 0 else -max_step_w
        else:
            self.current_w = self.target_w


        # 3. Set speed
        v = self.current_v
        w = self.current_w

        # Straight line
        if abs(v) > 0.01 and abs(w) < 0.01:
            self.straight_line_mode = True
        else:
            self.straight_line_mode = False

        turn_factor = 1.0

        v_left = v - (w * turn_factor * self.wheelbase / 2.0)
        v_right = v + (w * turn_factor * self.wheelbase / 2.0)

        pwm_l = int(abs(v_left) * (255.0 / self.max_speed_ms))
        pwm_r = int(abs(v_right) * (255.0 / self.max_speed_ms))

        # Deadband compensation
        MIN_PWM_STRAIGHT = 110
        MIN_PWM_TURN = 135
        
        if self.straight_line_mode:      
            if 0 < pwm_l < MIN_PWM_STRAIGHT: pwm_l = MIN_PWM_STRAIGHT
            if 0 < pwm_r < MIN_PWM_STRAIGHT: pwm_r = MIN_PWM_STRAIGHT
        else:
            if 0 < pwm_l < MIN_PWM_TURN: pwm_l = MIN_PWM_TURN
            if 0 < pwm_r < MIN_PWM_TURN: pwm_r = MIN_PWM_TURN

        # Save the BASE from the kinematic calculation
        self.base_pwm_left = min(255, pwm_l)
        self.base_pwm_right = min(255, pwm_r)

        # Directions
        if v_left > 0.01: self.dir_left = 1
        elif v_left < -0.01: self.dir_left = 0
        else: self.dir_left = 2

        if v_right > 0.01: self.dir_right = 1
        elif v_right < -0.01: self.dir_right = 0
        else: self.dir_right = 2

    def publish_pwm(self):
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9       
        # Watchdog: Stop robot if no command received in 0.2s
        if time_since_last_cmd > 0.2:
            self.base_pwm_left = 0
            self.base_pwm_right = 0
            self.dir_left = 2  # Release
            self.dir_right = 2 # Release
            self.straight_line_mode = False
            self.error_fwd = 0.0
            self.error_bwd = 0.0

        self.assign_vel() 
        # Convert absolute values to signed values (positive/negative)
        val_left = self.base_pwm_left if self.dir_left == 1 else -self.base_pwm_left
        if self.dir_left == 2: val_left = 0
        
        val_right = self.base_pwm_right if self.dir_right == 1 else -self.base_pwm_right
        if self.dir_right == 2: val_right = 0
        
        msg_pwm = Int16MultiArray()
        msg_pwm.data = [val_left, val_right]
        self.pub_pwm.publish(msg_pwm)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
