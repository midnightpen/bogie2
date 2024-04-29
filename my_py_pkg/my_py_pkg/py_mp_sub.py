#!/usr/bin/python3
import multiprocessing as mp
import ctypes
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

class my_cafe():
    def __init__(self,mp):
        self.cafeManager = mp.Manager()
        self.Buff = self.cafeManager.list()
        self.Buff_SSend = self.cafeManager.list()
        self.qr =  mp.Value('i',0)

def cafe_run(obj_cafe): 
    while(1):
        print(obj_cafe.qr.value)

class SubQR_data(Node):
    def __init__(self,obj_cafe:my_cafe):
        super().__init__('cafe')
        self.obj_cafe = obj_cafe
        self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10)

    def qr_callback(self, msg):
        if msg.data == "xxx":
            self.obj_cafe.qr.value = 0
        else:
            self.obj_cafe.qr.value = 1         

def subQR(obj_cafe):
    rclpy.init()
    sub_QR = SubQR_data(obj_cafe)
    rclpy.spin(sub_QR)
    sub_QR.destroy_node()
    rclpy.shutdown()    

def main():
    try:
        obj_cafe = my_cafe(mp)   
        p_sub_qr  = mp.Process(target=subQR,args=(obj_cafe,))
        p_runcafe = mp.Process(target=cafe_run,args=(obj_cafe,))
        p_sub_qr.start()
        p_runcafe.start()
        while(1):
            1   
    except KeyboardInterrupt:
        p_sub_qr.kill()
        p_runcafe.kill()

    finally:
        print("\n\nAll process is shutdown.")

if __name__ == '__main__':
    main()