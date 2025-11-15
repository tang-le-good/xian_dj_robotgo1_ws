#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <sensor_msgs/Joy.h>

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>

#define SERVER_IP "192.168.1.135"
#define SERVER_PORT 8886
#define MAX_RETRY 900000000 
#define RETRY_INTERVAL 3 // 重试间隔(秒)     3秒重连1次，900000000次，时间大概是10年

// client发送的数据
struct client2server 
{
    unsigned char address_0; 
    unsigned char address_1;
    unsigned char address_2; 
    unsigned char address_3; 
    unsigned char address_4;
    unsigned char address_5; 
    unsigned char address_6; 
    unsigned char address_7;
};

// 接收到的server的数据
struct server2client
{
    unsigned char address_0; 
    unsigned char address_1;
    unsigned char address_2; 
    unsigned char address_3; 
    unsigned char address_4;
    unsigned char address_5; 
    unsigned char address_6; 
};

class TCPClient 
{
    private:
        int sockfd;
        struct sockaddr_in server_addr;
        bool connected;
        
    public:
        TCPClient() : sockfd(-1), connected(false) 
        {
            memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(SERVER_PORT);
            
            if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) 
            {
                std::cerr << "无效的地址或地址不支持" << std::endl;
            }
        }
        
        ~TCPClient() 
        {
            disconnect();
        }
        
        bool connectToServer(int max_retry = MAX_RETRY) 
        {
            int retry_count = 0;
            
            while (retry_count < max_retry && !connected) 
            {
                // 创建套接字
                if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
                {
                    std::cerr << "创建套接字失败" << std::endl;
                    return false;
                }
                
                // 尝试连接
                if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) 
                {
                    std::cerr << "连接失败，尝试重连 (" << (retry_count + 1) << "/" << max_retry << ")..." << std::endl;
                    close(sockfd);
                    sockfd = -1;
                    retry_count++;
                    std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL));
                    continue;
                }
                
                connected = true;
                std::cout << "成功连接到服务器 " << SERVER_IP << ":" << SERVER_PORT << std::endl;
                return true;
            }
            
            return false;
        }
        
        void disconnect() 
        {
            if (sockfd != -1) 
            {
                close(sockfd);
                sockfd = -1;
            }
            connected = false;
        }
        
        bool sendData(const client2server& client_data) 
        {
            if (!connected && !connectToServer()) 
            {
                std::cerr << "无法发送数据：未连接到服务器" << std::endl;
                return false;
            }
            
            if (send(sockfd, &client_data, sizeof(client_data), 0) < 0) 
            {
                std::cerr << "发送数据失败" << std::endl;
                connected = false;
                return false;
            }
            
            return true;
        }
        
        bool receiveData(server2client& server_data) 
        {
            
            // 设置文件描述符集合
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sockfd, &readfds);
            
            // 设置3秒超时
            struct timeval timeout;
            timeout.tv_sec = 3;
            timeout.tv_usec = 0;
            
            // 等待socket可读
            int select_result = select(sockfd + 1, &readfds, NULL, NULL, &timeout);
            
            if (select_result == 0) {
                std::cerr << "接收数据超时（3秒）" << std::endl;
                // connected = false;
                return false;
            } else if (select_result < 0) {
                std::cerr << "select错误: " << strerror(errno) << std::endl;
                // connected = false;
                return false;
            }
            
            // 有数据可读，正常接收
            int recv_len = recv(sockfd, &server_data, sizeof(server_data), 0);
            
            if (recv_len <= 0) {
                std::cerr << "接收数据失败或连接已关闭" << std::endl;
                connected = false;
                return false;
            }
            
            return true;
        }
        
        bool isConnected() const 
        {
            return connected;
        }
};
class XianDjRobotgo1BatteryLevel
{
    public:
        XianDjRobotgo1BatteryLevel()
        {
            // 创建一个ROS节点句柄
            // ros::NodeHandle nh;
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            date_recive_counter_pre = date_recive_counter_cur;
            date_recive_counter_cur = date_recive_counter;

            if(date_recive_counter_pre == date_recive_counter_cur)
            {
                timeout_counter += 1;
                timeout_counter = timeout_counter > 1000 ? 5 : (timeout_counter + 1);
            }
            else
            {
                timeout_counter = 0;
            }
            if(timeout_counter >= 5)
            {
                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level_heart_beat", xian_dj_robotgo1_battery_level_heart_beat); 
                std::cout << "xian_dj_robotgo1_battery_level_heart_beat: " << xian_dj_robotgo1_battery_level_heart_beat << std::endl;
                counter = 0;
                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level_heart_beat", counter);  // 自行替换
            }
            else
            {
                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level_heart_beat", xian_dj_robotgo1_battery_level_heart_beat); 
                std::cout << "xian_dj_robotgo1_battery_level_heart_beat: " << xian_dj_robotgo1_battery_level_heart_beat << std::endl;
                counter = counter > 1000 ? 0 : (counter + 1);
                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level_heart_beat", counter);  // 自行替换
            }

            
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            // 电压读取
            client_address_0 = 0x01; 
            client_address_1 = 0x03;
            client_address_2 = 0x00; 
            client_address_3 = 0x28; 
            client_address_4 = 0x00;
            client_address_5 = 0x01;
            
            // client_address_6 = 0x5C;
            // client_address_7 = 0x9D; 
            // tcp_server_date = this->command_callback(client_address_0, client_address_1,client_address_2,
            //                                          client_address_3, client_address_4,client_address_5,
            //                                          client_address_6, client_address_7);
            tcp_server_date = this->command_callback(client_address_0, client_address_1,client_address_2,
                                                     client_address_3, client_address_4,client_address_5);
            if(tcp_server_date!=nullptr)
            {
                unsigned char battery_level = *(tcp_server_date+4);
                // printf("电池电量： %u  \n",battery_level);
                xian_dj_robotgo1_battery_level = std::to_string(battery_level);
                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level", battery_level);
                std::cout << "xian_dj_robotgo1_battery_level: " << xian_dj_robotgo1_battery_level << std::endl; 
            }
            
        }

    private:
        // 心跳
        int counter = 0;
        int xian_dj_robotgo1_battery_level_heart_beat = 0;
        std::string xian_dj_robotgo1_battery_level;
        int date_recive_counter = 0;
        int date_recive_counter_cur = 0;
        int date_recive_counter_pre = 0;
        int timeout_counter = 0;

        unsigned char client_address_0;
        unsigned char client_address_1;
        unsigned char client_address_2;
        unsigned char client_address_3;
        unsigned char client_address_4;
        unsigned char client_address_5;
        unsigned char client_address_6;
        unsigned char client_address_7;
        unsigned char * tcp_server_date;

        TCPClient client;
        struct client2server client_data;
        struct server2client server_data;
        
        unsigned char* command_callback(unsigned char address_0, unsigned char address_1, unsigned char address_2, 
                                        unsigned char address_3, unsigned char address_4, unsigned char address_5
                                        )
        {
            static unsigned char tcp_date[7] ; 
            if (!client.isConnected()) 
            {
                std::cout << "连接已断开，尝试重新连接..." << std::endl;
                if (!client.connectToServer()) 
                {
                    std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL));
                    return 0x00;
                }
            }
                
            // 发送数据
            client_data.address_0 = client_address_0;
            client_data.address_1 = client_address_1;
            client_data.address_2 = client_address_2;
            client_data.address_3 = client_address_3;
            client_data.address_4 = client_address_4;
            client_data.address_5 = client_address_5;
            // client_data.address_6 = client_address_6;
            // client_data.address_7 = client_address_7;

            // 计算CRC校验值
            uint16_t crc = CalculateStructCRC(client_data);
            
            unsigned char high_byte, low_byte;
            SplitInt16ToBytes(crc, high_byte, low_byte);
            // 校验位
            client_data.address_6 = low_byte;
            client_data.address_7 = high_byte;
            
            printf(" Client send: address_0=%X, address_1=%X, address_2=%X, address_3=%X, address_4=%X, address_5=%X, address_6=%X, address_7=%X \n", 
                   client_data.address_0,
                   client_data.address_1,
                   client_data.address_2,
                   client_data.address_3,
                   client_data.address_4,
                   client_data.address_5,
                   client_data.address_6,
                   client_data.address_7);

            if (!client.sendData(client_data)) 
            {
                return 0x00;
            }
            
            // // 接收响应
            // if (client.receiveData(server_data)) 
            // {
            //     date_recive_counter = date_recive_counter > 1000 ? 0 : (date_recive_counter + 1);
            //     printf("Recived from server:  address_0=%X, address_1=%X, address_2=%X, address_3=%X, address_4=%X, address_5=%X, address_6=%X \n", 
            //             server_data.address_0,
            //             server_data.address_1,
            //             server_data.address_2,
            //             server_data.address_3,
            //             server_data.address_4,
            //             server_data.address_5,
            //             server_data.address_6
            //             );
            // }
            // else
            // {
            //     printf("ERROR!!!");
            //     return 0x00;
            // }
            // tcp_date[0] = server_data.address_0;
            // tcp_date[1] = server_data.address_1;
            // tcp_date[2] = server_data.address_2;
            // tcp_date[3] = server_data.address_3;
            // tcp_date[4] = server_data.address_4;
            // tcp_date[5] = server_data.address_5;
            // tcp_date[6] = server_data.address_6;
        
            // return tcp_date;


            // 接收响应（带3秒超时）
            if (client.receiveData(server_data)) 
            {
                date_recive_counter = date_recive_counter > 1000 ? 0 : (date_recive_counter + 1);
                printf("date_recive_counter:%d \n", date_recive_counter);
                
                tcp_date[0] = server_data.address_0;
                tcp_date[1] = server_data.address_1;
                tcp_date[2] = server_data.address_2;
                tcp_date[3] = server_data.address_3;
                tcp_date[4] = server_data.address_4;
                tcp_date[5] = server_data.address_5;
                tcp_date[6] = server_data.address_6;
                return tcp_date;
            } 
            else 
            {
                // 3秒超时后会执行到这里                
                return nullptr;
            }







        }


        // CRC校验
        // uint16_t ModbusGetCRC(uint8_t* data, uint8_t count)
        // {
        //     uint8_t i, j, xdabit;
        //     uint16_t reg_crc = 0xFFFF;  // CRC初始值
            
        //     // 遍历每个数据字节
        //     for (i = 0; i < count; i++)
        //     {
        //         reg_crc ^= (uint16_t)data[i];  // 与当前字节进行异或
                
        //         // 处理每个字节的8个位
        //         for (j = 0; j < 8; j++)
        //         {
        //             xdabit = (uint8_t)(reg_crc & 0x01);  // 获取最低位
        //             reg_crc >>= 1;  // 右移一位
                    
        //             // 如果最低位为1，进行多项式异或
        //             if (xdabit == 1)
        //             {
        //                 reg_crc ^= 0xA001;  // Modbus CRC16多项式
        //             }
        //         }
        //     }
            
        //     return reg_crc;
        // }
        // CRC校验（使用图片中的计算方法）
        uint16_t ModbusGetCRC(uint8_t* data, uint8_t count)
        {
            uint8_t i, j;
            uint16_t wCRC = 0xFFFF;  // 使用与图片相同的变量名
            
            while (count--)
            {
                wCRC ^= (uint16_t)(*data++);  // 完全模仿图片的指针操作
                
                for (i = 0; i < 8; i++)
                {
                    if (wCRC & 0x0001)  // 直接检查最低位
                    {
                        wCRC = (wCRC >> 1) ^ 0xA001;
                    }
                    else
                    {
                        wCRC = wCRC >> 1;  // 与图片完全一致
                    }
                }
            }
            return wCRC;
        }
        /**
        * 为client2server结构体计算CRC16校验值
        * @param data 结构体引用
        * @return CRC16校验值
        */
        uint16_t CalculateStructCRC(const client2server& data)
        {
            // 将结构体转换为字节数组进行CRC计算
            return ModbusGetCRC((uint8_t*)&data, 6);
        }

        /**
        * 将16位拆分为高字节和低字节
        * @param crc_value 16位CRC值
        * @param high_byte 返回的高字节
        * @param low_byte 返回的低字节
        */
        void SplitInt16ToBytes(uint16_t crc_value, unsigned char& high_byte, unsigned char& low_byte)
        {
            // 方法1：使用位操作（推荐）
            high_byte = (crc_value >> 8) & 0xFF;  // 获取高8位
            low_byte = crc_value & 0xFF;          // 获取低8位
        }

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_battery_level");
    XianDjRobotgo1BatteryLevel xian_dj_robotgo1_battery_level;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_robotgo1_battery_level.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1BatteryLevel::m_timer_heart_beat_func, &xian_dj_robotgo1_battery_level);
    xian_dj_robotgo1_battery_level.m_timer_control = nh_2.createWallTimer(ros::WallDuration(2), &XianDjRobotgo1BatteryLevel::m_timer_control_func, &xian_dj_robotgo1_battery_level);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}