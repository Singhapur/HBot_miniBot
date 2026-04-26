import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty

msg = """
Robot Commandos
---------------------------
Move:      Camera:   Radar:
   w           q   e   l
 a s d 
   

Stop or Hand brake: Espacio o x
CTRL-C for close program
"""

class TecladoNode(Node):
    def __init__(self):
        super().__init__('nodo_teclado')
        self.publisher_ = self.create_publisher(String, '/robot_command', 10)

    def send_command(self, key):
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)

def get_key(settings):
    # Get press key from keyboard
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = TecladoNode()
    
    settings = termios.tcgetattr(sys.stdin)
    
    print(msg)
    
    try:
        while True:
            key = get_key(settings)
            if key in ['w', 'a', 's', 'd', 'x', ' ','q', 'e', 'l']:
                node.send_command(key)
                print(f"Tecla: {key}")
            if key == '\x03': # CTRL+C
                break
    except Exception as e:
        print(e)
    finally:
        node.send_command('x') # Send stop command
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
