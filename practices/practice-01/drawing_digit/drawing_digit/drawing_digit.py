import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class TurtleDigitDrawer(Node):
    def __init__(self):
        super().__init__('drawing_digit')
        self.declare_parameter('turtle_name', 'turtle2')
        self.declare_parameter('digit', 1)

        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        self.digit = self.get_parameter('digit').get_parameter_value().integer_value

        self.publisher = self.create_publisher(
            Twist,
            f'/{self.turtle_name}/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtle_name}/pose',
            self.pose_callback,
            10
        )
        
        self.state = 0
        
        self.current_pose = None
        self.x_start = None
        self.y_start = None
        self.is_moving = False
        
        self.target_distance = 3.0
        self.target_angle = math.radians(90.0)
        self.angular_speed = 1.0
        self.angle_tolerance = 0.02
        self.linear_speed = 1.0


        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'Нод запущен для черепахи [{self.turtle_name}], цифра: {self.digit}')
        

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f'Получены координаты: x={msg.x:.2f}, y={msg.y:.2f}')
        self.current_pose = msg
        if self.x_start is None and self.y_start is None:
            self.x_start = msg.x
            self.y_start = msg.y
            self.is_moving = True
            self.get_logger().info(f'Старт зафиксирован: x={self.x_start:.2f}, y={self.y_start:.2f}')
            
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))        
            

    def control_loop(self):
        self.get_logger().info('Таймер сработал!')
        if self.current_pose is None or self.x_start is None or self.y_start is None or not self.is_moving:
            return
    
            
        
        msg = Twist()
        
        if self.state == 0:
            angle_diff = self.normalize_angle(self.target_angle - self.current_pose.theta)

            if abs(angle_diff) <= self.angle_tolerance:
                msg.angular.z = 0.0
                self.publisher.publish(msg)

                self.x_start = self.current_pose.x
                self.y_start = self.current_pose.y
                
                self.get_logger().info(f'Поворот завершен (theta={self.current_pose.theta:.2f})')
                self.state = 1
            else:
                msg.angular.z = math.copysign(self.angular_speed, angle_diff)
                self.publisher.publish(msg)
        
        elif self.state == 1:
            dx = self.current_pose.x - self.x_start
            dy = self.current_pose.y - self.y_start
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist < self.target_distance:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.get_logger().info(f'Пройдено: {dist:.2f} / {self.target_distance:.2f} м',
                        throttle_duration_sec=0.5
                )
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.is_moving = False
                self.get_logger().info(f'Цель достигнута!')
            
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleDigitDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
