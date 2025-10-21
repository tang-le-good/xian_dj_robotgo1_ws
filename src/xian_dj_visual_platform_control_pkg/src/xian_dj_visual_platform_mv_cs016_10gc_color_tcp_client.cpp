#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

#define SERVER_IP "192.168.1.12" // 替换为服务器实际IP
#define SERVER_PORT 4068
#define MAX_RETRY 900000000 
#define RETRY_INTERVAL 3 // 重试间隔(秒)

// 图像传输协议头
struct ImageHeader 
{
    uint32_t frame_size;
    uint32_t frame_number;
    uint64_t timestamp;
};

class TCPClient {
private:
    int sockfd;
    struct sockaddr_in server_addr;
    bool connected;
    
public:
    TCPClient() : sockfd(-1), connected(false) {
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        
        if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
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
        
        while (retry_count < max_retry && !connected) {
            // 创建套接字
            if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                std::cerr << "创建套接字失败" << std::endl;
                return false;
            }
            
            // 尝试连接
            if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
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
    
    void disconnect() {
        if (sockfd != -1) {
            close(sockfd);
            sockfd = -1;
        }
        connected = false;
    }
    
    bool receiveFrame(cv::Mat &frame) {
        if (!connected) {
            return false;
        }
        
        // 1. 接收协议头
        ImageHeader header;
        int received = recv(sockfd, &header, sizeof(header), MSG_WAITALL);
        if (received <= 0) {
            std::cerr << "接收协议头失败或连接已关闭" << std::endl;
            connected = false;
            return false;
        }
        
        // 2. 接收图像数据
        std::vector<uchar> buffer(header.frame_size);
        int total_received = 0;
        
        while (total_received < header.frame_size) {
            int bytes_received = recv(sockfd, buffer.data() + total_received, 
                                     header.frame_size - total_received, 0);
            if (bytes_received <= 0) {
                std::cerr << "接收图像数据失败" << std::endl;
                connected = false;
                return false;
            }
            total_received += bytes_received;
        }
        
        // 3. 解码图像
        frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if (frame.empty()) {
            std::cerr << "图像解码失败" << std::endl;
            return false;
        }
        
        std::cout << "接收帧 #" << header.frame_number 
                  << ", 大小: " << header.frame_size 
                  << "字节, 时间戳: " << header.timestamp << std::endl;
        
        return true;
    }
    
    bool isConnected() const {
        return connected;
    }
};

class XianDjVisualPlatformMvcs01610gcColorRosClient
{
public:
    TCPClient client;
    uint32_t frame_count = 0;
    XianDjVisualPlatformMvcs01610gcColorRosClient()
    {
        ros::NodeHandle nh;
        // 创建定时器用于发布心跳信号[10](@ref)
        // timer_heart_beat_ = nh.createWallTimer(ros::WallDuration(1.0), 
        //                                       &XianDjVisualPlatformMvcs01610gcColorRosClient::m_timer_heart_beat_func, this);
    }
    
    ros::WallTimer m_timer_heart_beat;
    ros::WallTimer m_timer_control;

    // void timerHeartBeatCallback(const ros::WallTimerEvent& event)
    // {
    //     // 发布心跳信号到参数服务器[10](@ref)
    //     // private_nh_.setParam("xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat", heart_beat_counter_);
    //     heart_beat_counter_ = heart_beat_counter_ > 1000 ? 0 : (heart_beat_counter_ + 1);
    //     private_nh_.setParam("xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat", heart_beat_counter_);
    //     ROS_INFO_THROTTLE(1.0, "heart xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat = %d", xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat);
    // }

    void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
    {
        ros::param::get("/xian_dj_visual_platform_params_server/xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat", xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat); 
        std::cout << "xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat: " << xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat << std::endl;
        counter = counter > 1000 ? 0 : (counter + 1);
        ros::param::set("/xian_dj_visual_platform_params_server/xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat", counter);  // 自行替换
    }

    void m_timer_control_func(const ros::WallTimerEvent& event)
    {
        if (!client.isConnected()) 
        {
            std::cout << "连接已断开，尝试重新连接..." << std::endl;
            if (!client.connectToServer()) {
                std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL));
                return ;
            }
        }
        
        // 接收帧
        cv::Mat frame;
        if (client.receiveFrame(frame)) 
        {
            frame_count++;

            printf("heigt: %d, width: %d channel: %d \n", frame.rows, frame.cols, frame.channels());
            
            // 按ESC退出
            if (cv::waitKey(1) == 27) {
                ros::shutdown();
            }
        } 
        else 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    ros::NodeHandle private_nh_;
    ros::WallTimer timer_heart_beat_;

    int counter = 0;
    int xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client_heart_beat = 0;
};

int main(int argc, char** argv)
{
    // 初始化ROS节点[4](@ref)
    ros::init(argc, argv, "xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client");
    XianDjVisualPlatformMvcs01610gcColorRosClient xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client;
    
    // // 初始连接
    // if (!client.connectToServer()) {
    //     std::cerr << "无法连接到服务器，退出程序" << std::endl;
    //     return -1;
    // }
    
    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjVisualPlatformMvcs01610gcColorRosClient::m_timer_heart_beat_func, &xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client);
    xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjVisualPlatformMvcs01610gcColorRosClient::m_timer_control_func, &xian_dj_visual_platform_mv_cs016_10gc_color_tcp_client);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
    // while (ros::ok()) 
    // {
    //     if (!client.isConnected()) 
    //     {
    //         std::cout << "连接已断开，尝试重新连接..." << std::endl;
    //         if (!client.connectToServer()) {
    //             std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL));
    //             continue;
    //         }
    //     }
        
    //     // 接收帧
    //     cv::Mat frame;
    //     if (client.receiveFrame(frame)) 
    //     {
    //         frame_count++;

    //         printf("heigt: %d, width: %d channel: %d \n", frame.rows, frame.cols, frame.channels());
            
    //         // 按ESC退出
    //         if (cv::waitKey(1) == 27) {
    //             break;
    //         }
    //     } 
    //     else 
    //     {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }
    
    return 0;
}