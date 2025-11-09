# -*- coding: utf-8 -*-
import socket
import time
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

def main():
    client = TCPClient()
    
    # 初始连接
    if not client.connect_to_server():
        print("无法连接到服务器，退出程序")
        return -1
    
    message_counter = 0
    
    try:
        while True:
            time.sleep(2)  # 50 ms延迟
            
            if not client.is_connected():
                print("连接已断开，尝试重新连接...")
                if not client.connect_to_server():
                    time.sleep(RETRY_INTERVAL)
                    continue
            
            # 准备发送不同长度的消息
            message_to_send = "$001,123.45#"
            
            
            # 发送数据
            if not client.send_data(message_to_send, add_delimiter=False):
                print("发送失败，等待后继续...")
                continue
            
            # # 接收响应
            # received_message = client.receive_data()
            # if received_message is not None:
            #     print(f"服务器返回消息长度: {len(received_message)}")
                
            #     # 验证长度是否匹配（可选）
            #     if len(received_message) == len(message_to_send):
            #         print("✓ 长度验证通过")
            #     else:
            #         print(f"⚠ 长度不匹配: 发送{len(message_to_send)}字节, 接收{len(received_message)}字节")
            # else:
            #     print("未收到服务器响应")
            
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        client.disconnect()
        print("程序退出")

if __name__ == "__main__":
    main()
