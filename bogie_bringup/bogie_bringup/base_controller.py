#!/usr/bin/python3
import numpy as np
import serial
import multiprocessing as mp
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Uart():
    def __init__(self,mp):
        self.mana = mp.Manager()
        self.Buff = self.mana.list()

        self.flag_send =  mp.Value('i',0)
        

        self.Buff_SSend = self.mana.list()

        self.port = "/dev/bogie"
        self.baudrate = 38400
        self.data_preview = ""
            
        # motor param
        self.speed_ratio = mp.Value('d',0.000031)   # unit: m/encode #0.0000415
        self.wheel_distance = mp.Value('d',0.29)   # unit: m 0.29
        self.encode_sampling_time = mp.Value('d',0.04)  # unit: s
        self.cmd_vel_linear_max = mp.Value('d',0.8)  # unit: m/s
        self.cmd_vel_angular_max = mp.Value('d',1.0)  #  unit: rad/s

        # odom result
        self.position_x = mp.Value('d',0.0)       # unit: m
        self.position_y = mp.Value('d',0.0)       # unit: m
        self.oriention = mp.Value('d',0.0)       # unit: rad
        self.velocity_linear = mp.Value('d',0.0)       # unit: m/s
        self.velocity_angular = mp.Value('d',0.0)       # unit: rad/s

        # speed result
        self.left_wheel_speed = mp.Value('d',0.0)
        self.right_wheel_speed = mp.Value('d',0.0)

        self.ser = self.check_port_open()
        self.target_l = mp.Value('d',0.0)
        self.target_r = mp.Value('d',0.0)
        self.feedback_l = mp.Value('d',0.0)
        self.feedback_r = mp.Value('d',0.0)

    def check_port_open(self):
        try:
            ser = serial.Serial(self.port,self.baudrate, timeout=1000 ,stopbits=1)
            print(self.port + ': port is Open.')
            return ser
        except:
            print(self.port + ': open port Fail.')
        return 0


   
def Receive_uart(Obj_uart): 
    print("Start Receive UART")
    header_check=0
    dada_amount = 0
    incoming_data = []
    while(1):
        try:
            s = Obj_uart.ser.read(1)
            ss=int.from_bytes(s, byteorder="big",signed=0)
            # print(ss,state)
            if(header_check == 0 and ss == 204):
                incoming_data.append(ss)
                header_check = 1
            elif(header_check == 1 and ss == 10):
                incoming_data.append(ss)
                dada_amount=0
                header_check = 2 
            elif(header_check ==2 ):
                if(dada_amount <=8 ):
                    incoming_data.append(ss)
                if(dada_amount==8 ):
                    # print(incoming_data)
                    Obj_uart.Buff.append(incoming_data)
                    header_check = 0
                    dada_amount=0
                    incoming_data=[]
                # print("u",ss)
                dada_amount += 1

        except:
            Obj_uart.ser = Obj_uart.check_port_open()

def Transmit_uart(Obj_uart): 
    while(1):
        if(len(Obj_uart.Buff_SSend)>0):
            try:
                Obj_uart.ser.write(bytearray(Obj_uart.Buff_SSend[0]))
                Obj_uart.Buff_SSend.pop(0)

            except:
                Obj_uart.ser = Obj_uart.check_port_open()

def cal_uart(Obj_uart:Uart):
    buffer = []
    while(1):
        if(len(Obj_uart.Buff)>0):
            # print(Obj_uart.Buff)
            buffer.extend(Obj_uart.Buff[0])
            Obj_uart.Buff.pop(0)
         
            if(len(buffer)==11):
                #print("[Recv from Serial]: ",buffer)
                while(1):
                    # print("[Recv from Serial]: ",buffer)
                    # print(buffer[0] == 0xCC and buffer[1] == 0X0A )
                    # print(sum(buffer[0:9]) == buffer[-1])
                    if(buffer[0] == 0xCC and buffer[1] == 0X0A ):
                        checksum = sum(buffer[0:10])
                        checksum = checksum & 0XFF
                        if(checksum != buffer[-1]):
                            #print("Wrong data checksum (",buffer[-1],"!=",checksum,")")
                            buffer =[]
                            break
                        #print("[Recv from Serial]: ",buffer)
                        if (buffer[2] > 0):
                            multiplier = 1
                        else:
                            multiplier = -1
                        delta_encode_left_temp = multiplier  * ((int(buffer[3]) << 16) | (int(buffer[4]) << 8) | int(buffer[5]))
                        if (buffer[6] > 0):
                            multiplier = 1
                        else:
                            multiplier = -1
                        delta_encode_right_temp = multiplier  * ((int(buffer[7]) << 16) | (int(buffer[8]) << 8) | int(buffer[9]))

                        delta_d_left = delta_encode_left_temp * Obj_uart.speed_ratio.value
                        delta_d_right = delta_encode_right_temp * Obj_uart.speed_ratio.value
                        delta_d = 0.5 * (delta_d_left + delta_d_right)
                        delta_theta = (delta_d_right - delta_d_left) / Obj_uart.wheel_distance.value
                        delta_x = delta_d * math.cos(Obj_uart.oriention.value + delta_theta * 0.5)
                        delta_y = delta_d * math.sin(Obj_uart.oriention.value + delta_theta * 0.5)

                        Obj_uart.position_x.value = Obj_uart.position_x.value + delta_x
                        Obj_uart.position_y.value = Obj_uart.position_y.value + delta_y
                        Obj_uart.oriention.value = Obj_uart.oriention.value + delta_theta
                        Obj_uart.velocity_linear.value = delta_d / Obj_uart.encode_sampling_time.value
                        Obj_uart.velocity_angular.value = delta_theta / Obj_uart.encode_sampling_time.value

                        Obj_uart.left_wheel_speed.value = delta_encode_left_temp * Obj_uart.speed_ratio.value / Obj_uart.encode_sampling_time.value
                        Obj_uart.right_wheel_speed.value = delta_encode_right_temp * Obj_uart.speed_ratio.value / Obj_uart.encode_sampling_time.value
                        #print("Encoder (left" ,delta_encode_left_temp,"right ",delta_encode_right_temp)
                        Obj_uart.feedback_l.value =  delta_encode_left_temp
                        Obj_uart.feedback_r.value =  delta_encode_right_temp
                        #print("Encoder (left" ,Obj_uart.feedback_l,"right ",Obj_uart.feedback_r)
                        #print( "left target=",int(Obj_uart.target_l.value),"left feedback=",int(Obj_uart.feedback_l.value),
                         #      "right target=", int(Obj_uart.target_r.value),"right feedback=",int(Obj_uart.feedback_r.value))


                        buffer=[]
                        break

                    else:
                        print("Wrong data header (",buffer[0],"!= CC or ",buffer[1]," != 0A)")
                        buffer =[]
                        break


def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class Pubodom(Node):
    def __init__(self,Obj_uart:Uart):
        super().__init__('odom_publisher')
        self.Obj_uart = Obj_uart

        self.odom_pub_ = self.create_publisher(Odometry, 'odom/raw', 20)
        self.wheel_left_speed_pub_ = self.create_publisher(Float32, 'wheel_left_speed', 20)
        self.wheel_right_speed_pub_ = self.create_publisher(Float32, 'wheel_right_speed', 20)
    
        self.tf_broadcaster = TransformBroadcaster(self)


        timer_period = 0.1   
        self.timer = self.create_timer(timer_period, self.timer_callback)
 

        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,20)
        self.flag_time =False
        self.timer_ = self.create_timer(0.1,self.timer_callback_vel)
        self.timer_pre = time.time()

    def timer_callback(self):
        self.get_logger().info("left target = %d left feedback = %d right target = %d right feedback = %d" % (self.Obj_uart.target_l.value, self.Obj_uart.feedback_l.value,self.Obj_uart.target_r.value, self.Obj_uart.feedback_r.value))

        odom_frame_id = "odom"
        odom_child_frame_id = "base_footprint"

        t=TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = odom_frame_id
        t.child_frame_id = odom_child_frame_id
        t.transform.translation.x = self.Obj_uart.position_x.value
        t.transform.translation.y = self.Obj_uart.position_y.value
        t.transform.translation.z = 0.0
        q=get_quaternion_from_euler(0.0,0.0,self.Obj_uart.oriention.value)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = odom_frame_id
        odom.child_frame_id = odom_child_frame_id
        odom.pose.pose.position.x = self.Obj_uart.position_x.value
        odom.pose.pose.position.y = self.Obj_uart.position_y.value
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.Obj_uart.velocity_linear.value
        odom.twist.twist.angular.z = self.Obj_uart.velocity_angular.value

        covariance = [0.01, 0, 0, 0, 0, 0,  # covariance on gps_x
                        0, 0.01, 0, 0, 0, 0,  # covariance on gps_y
                        0, 0, 99999, 0, 0, 0, # covariance on gps_z
                        0, 0, 0, 99999, 0, 0, # large covariance on rot x
                        0, 0, 0, 0, 99999, 0, # large covariance on rot y
                        0, 0, 0, 0, 0, 0.01] # large covariance on rot z

        odom.pose.covariance = covariance

        self.odom_pub_.publish(odom)

        wheel_left_speed_msg =Float32()
        wheel_left_speed_msg.data = self.Obj_uart.left_wheel_speed.value

        wheel_right_speed_msg = Float32()
        wheel_right_speed_msg.data = self.Obj_uart.right_wheel_speed.value

        self.wheel_left_speed_pub_.publish(wheel_left_speed_msg)
        self.wheel_right_speed_pub_.publish(wheel_right_speed_msg)
        

    def timer_callback_vel(self):
        if(self.flag_time and time.time()-self.timer_pre>=0.5):
            print("[Send to Serial]: Stop")
            write_buff = [0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
            write_buff[10] = write_buff[0] + write_buff[1] + write_buff[2] + write_buff[3] + write_buff[4] + write_buff[5] + write_buff[6] + write_buff[7] + write_buff[8] + write_buff[9] & 0XFF
            self.Obj_uart.Buff_SSend.append(write_buff)
            self.flag_time = False

    def listener_callback(self,msg):
        # print(msg)
        angular_temp = np.clip(msg.angular.z, -self.Obj_uart.cmd_vel_angular_max.value, self.Obj_uart.cmd_vel_angular_max.value)
  
        linear_temp = np.clip(msg.linear.x,  -self.Obj_uart.cmd_vel_linear_max.value,  self.Obj_uart.cmd_vel_linear_max.value)


        delta_encode_left_temp = (linear_temp - 0.5 * (self.Obj_uart.wheel_distance.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
        delta_encode_right_temp = (linear_temp + 0.5 * (self.Obj_uart.wheel_distance.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)

        #print( "left target=",int(delta_encode_left_temp),"left feedback=",self.Obj_uart.feedback_l.value,"right target=", int(delta_encode_right_temp),"right feedback=",self.Obj_uart.feedback_r.value)
        #print(self.Obj_uart.feedback_r.value)
        self.Obj_uart.target_l.value = delta_encode_left_temp
        self.Obj_uart.target_r.value = delta_encode_right_temp

        write_buff=[0,0,0,0,0,0,0,0,0,0,0]
        write_buff[0]= 0xFF
        write_buff[1]= 0xFF
        if(delta_encode_left_temp >=0):
            write_buff[2]= 0x01
        else:
            write_buff[2]=0x00
        write_buff[3]= int(abs(delta_encode_left_temp)) >> 16  & 0xFF
        write_buff[4] = (int(abs(delta_encode_left_temp)) >> 8) & 0xFF
        write_buff[5] = int(abs(delta_encode_left_temp)) & 0xFF

        if(delta_encode_right_temp >=0):
            write_buff[6]= 0x01
        else:
            write_buff[6]=0x00
        write_buff[7] = int(abs(delta_encode_right_temp)) >> 16 & 0xFF
        write_buff[8] = (int(abs(delta_encode_right_temp)) >> 8) & 0xFF
        write_buff[9] = int(abs(delta_encode_right_temp)) & 0xFF
        write_buff[10] = (write_buff[0] + write_buff[1] + write_buff[2] + write_buff[3] + write_buff[4] + write_buff[5] + write_buff[6] + write_buff[7] + write_buff[8] + write_buff[9]) & 0xFF
        #print("Data: ",write_buff)
        self.Obj_uart.Buff_SSend.append(write_buff)
        #print(self.Obj_uart.Buff_SSend)
        self.timer_pre = time.time()
        self.flag_time = True

def pub(Obj_uart):
    rclpy.init()
    pub_odom = Pubodom(Obj_uart)

    #print("Publish Odom")
    rclpy.spin(pub_odom)
    pub_odom.destroy_node()
    rclpy.shutdown()

def main():
    try:
        Obj_uart = Uart(mp)   

        if(Obj_uart.ser!=0):
            p  = mp.Process(target=Receive_uart,args=(Obj_uart,))
            p2 = mp.Process(target=Transmit_uart,args=(Obj_uart,))
            #p3 = mp.Process(target=subCMD,args=(Obj_uart,))
            p4 = mp.Process(target=cal_uart,args=(Obj_uart,))
            p5 = mp.Process(target=pub,args=(Obj_uart,))

            p.start()
            p2.start()
            #p3.start()
            p4.start()
            p5.start()
            while(1):
                1
        
    except KeyboardInterrupt:
        if(Obj_uart.ser!=0):
            p.kill()
            p2.kill()
            #p3.kill()
            p4.kill()
            p5.kill()

    finally:
        print("\n\nAll process is shutdown.")



if __name__ == '__main__':
    main()