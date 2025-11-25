#!/usr/bin/env python3
#!coding=utf-8
import rospy
import numpy as np
import time, datetime, os, json, logging
from typing import Dict
import socket
import sys
import select

# 服务器配置
SERVER_IP = "192.168.1.134"
SERVER_PORT = 8886
MAX_RETRY = 900000000
RETRY_INTERVAL = 3  # 重试间隔(秒)

class TCPClient:
    def __init__(self):
        self.sockfd = None
        self.connected = False
        # 使用特殊字符作为消息分隔符
        self.MESSAGE_DELIMITER = b'\n'  # 换行符作为消息分隔符
        self.BUFFER_SIZE = 1024  # 接收缓冲区大小
        self.receive_timeout = 5.0  # 接收超时时间(秒)[1,3](@ref)
        self.receive_buffer = b""  # 接收缓冲区，用于处理不完整的消息
        
    def connect_to_server(self, max_retry=MAX_RETRY):
        retry_count = 0
        
        while retry_count < max_retry and not self.connected:
            try:
                # 创建TCP套接字
                self.sockfd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # 设置套接字选项，避免地址占用错误
                self.sockfd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # 设置接收超时[1,6](@ref)
                self.sockfd.settimeout(self.receive_timeout)
                
                # 尝试连接服务器
                print(f"尝试连接到服务器 {SERVER_IP}:{SERVER_PORT}...")
                self.sockfd.connect((SERVER_IP, SERVER_PORT))
                self.connected = True
                print(f"成功连接到服务器 {SERVER_IP}:{SERVER_PORT}")
                return True
                
            except (socket.timeout, socket.error, ConnectionRefusedError) as e:
                print(f"连接失败，尝试重连 ({(retry_count + 1)}/{max_retry})... 错误: {e}")
                if self.sockfd:
                    self.sockfd.close()
                    self.sockfd = None
                retry_count += 1
                time.sleep(RETRY_INTERVAL)
            except Exception as e:
                print(f"连接过程中发生未知错误: {e}")
                return False
        
        return False
    
    def disconnect(self):
        if self.sockfd:
            self.sockfd.close()
            self.sockfd = None
        self.connected = False
        self.receive_buffer = b""  # 清空接收缓冲区
        print("已断开与服务器的连接")
    
    def send_data(self, message_str, add_delimiter=False):
        """发送字符串到服务器，可选择是否添加分隔符"""
        if not self.connected:
            if not self.connect_to_server():
                print("无法发送数据：未连接到服务器")
                return False
        
        try:
            # 根据参数决定是否添加分隔符
            if add_delimiter:
                message_to_send = message_str.encode('utf-8') + b'\n'
            else:
                message_to_send = message_str.encode('utf-8')
            
            # 发送数据
            self.sockfd.sendall(message_to_send)
            hex_representation = ' '.join([f'{b:02X}' for b in message_to_send])
            print(f"发送消息: {message_str} (十六进制: {hex_representation})")
            return True
            
        except (socket.error, BrokenPipeError, socket.timeout) as e:
            print(f"发送数据失败: {e}")
            self.connected = False
            return False
        except UnicodeEncodeError as e:
            print(f"消息编码失败: {e}")
            return False
    
    def receive_data(self):
        """从服务器接收数据，支持超时和灵活的消息处理[1,6](@ref)"""
        if not self.connected:
            print("未连接到服务器，无法接收数据")
            return None
        
        try:
            # 使用select检查是否有数据可读，避免永久阻塞[4](@ref)
            ready = select.select([self.sockfd], [], [], self.receive_timeout)
            if not ready[0]:
                print(f"接收超时，{self.receive_timeout}秒内未收到数据")
                return None
            
            # 有数据可读，开始接收
            received_data = b""
            start_time = time.time()
            
            while True:
                # 检查是否超时
                if time.time() - start_time > self.receive_timeout:
                    print("接收过程超时")
                    break
                    
                try:
                    # 设置每次接收的超时
                    self.sockfd.settimeout(1.0)  # 每次recv操作最多等待1秒
                    chunk = self.sockfd.recv(self.BUFFER_SIZE)
                    
                    if not chunk:
                        print("连接已关闭")
                        self.connected = False
                        return None
                    
                    received_data += chunk
                    
                    # 检查是否收到完整消息（包含分隔符）
                    if self.MESSAGE_DELIMITER in received_data:
                        # 将新数据添加到缓冲区
                        self.receive_buffer += received_data
                        
                        # 分割消息
                        messages = self.receive_buffer.split(self.MESSAGE_DELIMITER)
                        
                        # 如果至少有一个完整消息
                        if len(messages) > 1:
                            # 提取第一个完整消息
                            complete_message = messages[0]
                            # 保留剩余的不完整数据在缓冲区中
                            self.receive_buffer = self.MESSAGE_DELIMITER.join(messages[1:])
                            break
                    
                    # 如果数据量很小，可能已经收到完整消息（即使没有分隔符）
                    if len(chunk) < self.BUFFER_SIZE:
                        # 可能已经收到完整消息
                        self.receive_buffer += received_data
                        if self.receive_buffer:
                            complete_message = self.receive_buffer
                            self.receive_buffer = b""
                            print(f"收到短消息，长度: {len(complete_message)}")
                            break
                        else:
                            continue
                            
                except socket.timeout:
                    # 单次recv超时，检查是否已经收到一些数据
                    if received_data:
                        # 将已接收的数据作为完整消息返回
                        self.receive_buffer += received_data
                        complete_message = self.receive_buffer
                        self.receive_buffer = b""
                        print(f"接收超时，返回已接收的数据，长度: {len(complete_message)}")
                        break
                    else:
                        # 继续等待
                        continue
            
            # 解码接收到的消息
            if 'complete_message' in locals():
                try:
                    message_str = complete_message.decode('utf-8').rstrip('\r\n')
                    print(f"接收消息: {message_str} (长度: {len(message_str)})")
                    return message_str
                except UnicodeDecodeError as e:
                    print(f"消息解码失败: {e}")
                    # 尝试其他编码或返回原始字节
                    return complete_message
            else:
                print("未收到完整消息")
                return None
                
        except socket.timeout:
            print("接收数据超时")
            return None
        except socket.error as e:
            print(f"接收数据失败: {e}")
            self.connected = False
            return None
        except Exception as e:
            print(f"接收过程中发生未知错误: {e}")
            return None
    
    def receive_data_simple(self, timeout=5.0):
        """简化的接收方法，不依赖分隔符[6](@ref)"""
        if not self.connected:
            return None
        
        try:
            self.sockfd.settimeout(timeout)
            received_data = b""
            
            # 尝试接收数据
            chunk = self.sockfd.recv(self.BUFFER_SIZE)
            if not chunk:
                self.connected = False
                return None
            
            received_data += chunk
            
            # 短暂等待是否有更多数据
            self.sockfd.settimeout(0.5)
            try:
                while True:
                    chunk = self.sockfd.recv(self.BUFFER_SIZE)
                    if not chunk:
                        break
                    received_data += chunk
            except socket.timeout:
                pass  # 没有更多数据是正常的
            
            message = received_data.decode('utf-8').rstrip('\n\r')
            print(f"接收消息: {message}")
            return message
            
        except socket.timeout:
            print("接收超时")
            return None
        except Exception as e:
            print(f"接收失败: {e}")
            self.connected = False
            return None
    
    def is_connected(self):
        """检查连接状态，并尝试检测是否真的连接"""
        if not self.connected or self.sockfd is None:
            return False
        
        try:
            # 通过发送空数据来测试连接是否有效
            self.sockfd.send(b'')
            return True
        except (socket.error, OSError):
            self.connected = False
            return False
     
class XianDjRobotgo1LighterDisplayer:
    def __init__(self):
        #TCP参数
        self.client = TCPClient()
        self.tcp_init()
        
        self.counter = 0
        
        # GPIO全局编号
        self.GPIO_green_lighter = 83  # 引脚29, 控制绿灯
        self.GPIO_red_lighter = 496  # 引脚15, 控制红灯
        self.GPIO_yellow_lighter = 495  # 引脚13, 控制黄灯
        
        self.xian_dj_robotgo1_green_ligher_cmd = 0
        self.xian_dj_robotgo1_red_ligher_cmd = 0
        self.xian_dj_robotgo1_yellow_ligher_cmd = 0
        self.xian_dj_robotgo1_displayer_cmd = 0

        #引脚使能，并设置为输出模式
        self.gpio_export(self.GPIO_green_lighter)
        self.gpio_set_direction(self.GPIO_green_lighter, "out")
    
        self.gpio_export(self.GPIO_red_lighter)
        self.gpio_set_direction(self.GPIO_red_lighter, "out")

        self.gpio_export(self.GPIO_yellow_lighter)
        self.gpio_set_direction(self.GPIO_yellow_lighter, "out")

    # def xian_heat_beat_callback(self, event):
    #     rospy.set_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat", self.counter)
    #     xian_dj_robotgo1_lighter_displayer_heart_beat = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat")
    #     if self.counter>1000:
    #         self.counter = 0
    #     self.counter += 1
    #     print("xian_dj_robotgo1_lighter_displayer_heart_beat:", xian_dj_robotgo1_lighter_displayer_heart_beat)
    
    
    # def xian_dj_tobotgo1_green_lighter_func(self, event):
    #     self.xian_dj_robotgo1_green_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd")
    #     print("green:", self.xian_dj_robotgo1_green_ligher_cmd)
    #     self.gpio_set_value(self.GPIO_green_lighter, self.xian_dj_robotgo1_green_ligher_cmd)
    

    # def xian_dj_tobotgo1_red_lighter_func(self, event):
    #     self.xian_dj_robotgo1_red_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd")
    #     print("red:", self.xian_dj_robotgo1_red_ligher_cmd)
    #     self.gpio_set_value(self.GPIO_red_lighter, self.xian_dj_robotgo1_red_ligher_cmd)
        

    # def xian_dj_tobotgo1_yellow_lighter_func(self, event):
    #     self.xian_dj_robotgo1_yellow_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd")
    #     print("yellow:", self.xian_dj_robotgo1_yellow_ligher_cmd)
    #     self.gpio_set_value(self.GPIO_yellow_lighter, self.xian_dj_robotgo1_yellow_ligher_cmd)
        

    def xian_dj_tobotgo1_displayer_func(self, event):
        # 心跳
        rospy.set_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat", self.counter)
        xian_dj_robotgo1_lighter_displayer_heart_beat = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat")
        if self.counter>1000:
            self.counter = 0
        self.counter += 1
        print("xian_dj_robotgo1_lighter_displayer_heart_beat:", xian_dj_robotgo1_lighter_displayer_heart_beat)

        # 绿灯
        self.xian_dj_robotgo1_green_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd")
        print("green:", self.xian_dj_robotgo1_green_ligher_cmd)
        self.gpio_set_value(self.GPIO_green_lighter, self.xian_dj_robotgo1_green_ligher_cmd)

        # 红灯
        self.xian_dj_robotgo1_red_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd")
        print("red:", self.xian_dj_robotgo1_red_ligher_cmd)
        self.gpio_set_value(self.GPIO_red_lighter, self.xian_dj_robotgo1_red_ligher_cmd)

        # 黄灯
        self.xian_dj_robotgo1_yellow_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd")
        print("yellow:", self.xian_dj_robotgo1_yellow_ligher_cmd)
        self.gpio_set_value(self.GPIO_yellow_lighter, self.xian_dj_robotgo1_yellow_ligher_cmd)

        self.xian_dj_robotgo1_displayer_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd")
        print("self.xian_dj_robotgo1_displayer_cmd:", self.xian_dj_robotgo1_displayer_cmd)
        if not self.client.is_connected():
            print("连接已断开，尝试重新连接...")
            if not self.client.connect_to_server():
                time.sleep(RETRY_INTERVAL)
                return -1
        
        # 准备发送不同长度的消息
        message_to_send = self.xian_dj_robotgo1_displayer_cmd
        
        
        # 发送数据
        if not self.client.send_data(message_to_send, add_delimiter=False):
            print("发送失败，等待后继续...")
            return -1

    def tcp_init(self):
        # 初始连接
        if not self.client.connect_to_server():
            print("无法连接到服务器，退出程序")
            return -1
        
    def gpio_export(self,pin):# 使能引脚
        export_path = f"/sys/class/gpio/gpio{pin}"
        if not os.path.exists(export_path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(pin))
            time.sleep(0.1)  # 等待系统创建目录
    
    def gpio_unexport(self, pin): # 复位引脚
        export_path = f"/sys/class/gpio/gpio{pin}"
        if os.path.exists(export_path):
            with open("/sys/class/gpio/unexport", "w") as f:
                f.write(str(pin))
    
    def gpio_set_direction(self,pin, direction):# 设置引脚为输出模式
        with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
            f.write(direction)

    def gpio_set_value(self,pin, value):# 设置引脚的高低电平
        with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
            f.write(str(value))



if __name__ == '__main__':
    try:
        tt = XianDjRobotgo1LighterDisplayer()
        rospy.init_node('xian_dj_robotgo1_lighter_displayer', anonymous=True)  # 初始化ROS节点
        # rospy.Timer(rospy.Duration(1), tt.xian_heat_beat_callback, oneshot=False) # 心跳线程
        # rospy.Timer(rospy.Duration(0.8), tt.xian_dj_tobotgo1_green_lighter_func, oneshot=False) # 绿灯线程
        # rospy.Timer(rospy.Duration(0.8), tt.xian_dj_tobotgo1_red_lighter_func, oneshot=False) # 红灯线程
        # rospy.Timer(rospy.Duration(0.8), tt.xian_dj_tobotgo1_yellow_lighter_func, oneshot=False) # 黄灯线程
        rospy.Timer(rospy.Duration(1), tt.xian_dj_tobotgo1_displayer_func, oneshot=False) # 数显模块线程
        rospy.spin()  # 添加这行确保节点持续运行

    except rospy.ROSInterruptException:
        pass