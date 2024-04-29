import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time

qr_code = ""

class qrcode(Node):

    def __init__(self):
        super().__init__('cafe')
        self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10)
        self.pub_way = self.create_publisher(String,'my_way',10)
        self.timer = self.create_timer(0.5,self.publish_way)


    def qr_callback(self, msg):
        global qr_code
        qr_code = msg.data


    def publish_way(self):
        global qr_code
        print(qr_code)
        qr_data = String()
        qr_data.data = qr_code
        self.pub_way.publish(qr_data)


def main(args=None):
    rclpy.init(args=args)
    node= qrcode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
