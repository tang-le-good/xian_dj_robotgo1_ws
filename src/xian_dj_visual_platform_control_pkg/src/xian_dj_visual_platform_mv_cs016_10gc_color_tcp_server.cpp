#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>  // 包含fcntl函数和相关宏的定义


#define PORT 4068
#define BACKLOG 128
#define FRAME_WIDTH 1440
#define FRAME_HEIGHT 1080

// 图像传输协议头
struct ImageHeader 
{
    uint32_t frame_size;    // 帧数据大小
    uint32_t frame_number;  // 帧序号
    uint64_t timestamp;     // 时间戳(微秒)
};

class TCPServer 
{
    private:
        int server_fd;
        int client_fd;
        struct sockaddr_in server_addr;
        struct sockaddr_in client_addr;
        bool is_running;
        int num_frames = 1;
        
        
    public:
        TCPServer() : server_fd(-1), client_fd(-1), is_running(false) 
        {
            memset(&server_addr, 0, sizeof(server_addr));
            memset(&client_addr, 0, sizeof(client_addr));
        }
        
        ~TCPServer() 
        {
            stop();
        }
        
        bool init() 
        {
            // 1. 创建套接字
            if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
            {
                std::cerr << "创建套接字失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            // 2. 设置SO_REUSEADDR选项
            int opt = 1;
            if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) 
            {
                std::cerr << "设置SO_REUSEADDR失败: " << strerror(errno) << std::endl;
                close(server_fd);
                return false;
            }
            
            // 3. 配置服务器地址
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = INADDR_ANY;
            server_addr.sin_port = htons(PORT);
            
            
            
            return true;
        }
        
        bool start() 
        {
            if (server_fd == -1 && !init()) 
            {
                return false;
            }
            
            // 绑定套接字到端口
            if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) 
            {
                std::cerr << "绑定失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            // 开始监听
            if (listen(server_fd, BACKLOG) < 0) 
            {
                std::cerr << "监听失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            is_running = true;
            std::cout << "服务器启动成功，监听端口 " << PORT << "..." << std::endl;
            
            return true;
        }
        
        void stop() 
        {
            is_running = false;
            
            if (client_fd != -1) 
            {
                close(client_fd);
                client_fd = -1;
            }
            
            if (server_fd != -1) 
            {
                close(server_fd);
                server_fd = -1;
            }
            
            std::cout << "服务器已停止" << std::endl;
        }
        
        bool acceptClient() 
        {
            socklen_t client_len = sizeof(client_addr);
            
            // 设置非阻塞模式
            int flags = fcntl(server_fd, F_GETFL, 0);
            fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);
            
            while (is_running) 
            {
                client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
                if (client_fd < 0) 
                {
                    if (errno == EWOULDBLOCK || errno == EAGAIN) 
                    {
                        // 没有客户端连接，等待1秒后重试
                        std::cout << "服务端已启动，等待客户端连接！" << std::endl;
                        sleep(1);
                        continue;
                    }
                    else 
                    {
                        std::cerr << "接受连接失败: " << strerror(errno) << std::endl;
                        return false;
                    }
                }
                
                // 恢复阻塞模式
                fcntl(server_fd, F_SETFL, flags);
                
                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
                std::cout << "客户端连接成功：" << client_ip << ":" << ntohs(client_addr.sin_port) << std::endl;
                
                return true;
            }
            
            return false;
        }
        
        bool isClientConnected() 
        {
            if (client_fd == -1) 
            {
                return false;
            }
            
            // 通过发送空数据检测连接状态
            char buf[1];
            ssize_t ret = recv(client_fd, buf, 1, MSG_PEEK | MSG_DONTWAIT);
            if (ret == 0) 
            {
                // 客户端正常关闭
                return false;
            } else if (ret == -1) 
            {
                if (errno != EAGAIN && errno != EWOULDBLOCK) 
                {
                    // 连接异常
                    return false;
                }
            }
            return true;
        }
        
        bool sendFrame(cv::Mat &frame) 
        {
            if (!isClientConnected()) 
            {
                return false;
            }
            
            // 缩放图像到指定分辨率
            // cv::Mat resized_frame;
            // cv::resize(frame, resized_frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
            
            // 编码为JPEG格式
            std::vector<uchar> buffer;
            std::vector<int> params;
            params.push_back(cv::IMWRITE_JPEG_QUALITY);
            params.push_back(95);
            
            if (!cv::imencode(".jpg", frame, buffer, params)) 
            {
                std::cerr << "图像编码失败" << std::endl;
                return false;
            }
            
            // 准备协议头
            ImageHeader header;
            header.frame_size = buffer.size();
            header.frame_number = num_frames;
            header.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            // 先发送协议头
            if (send(client_fd, &header, sizeof(header), MSG_NOSIGNAL) < 0) 
            {
                std::cerr << "发送协议头失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            // 发送图像数据
            int total_sent = 0;
            while (total_sent < buffer.size()) 
            {
                int sent = send(client_fd, buffer.data() + total_sent, buffer.size() - total_sent, MSG_NOSIGNAL);
                if (sent < 0) 
                {
                    std::cerr << "发送图像数据失败: " << strerror(errno) << std::endl;
                    return false;
                }
                total_sent += sent;
            }
            
            return true;
        }
        
        void processVideo(cv::Mat& frame) 
        {
            
            if(is_running && !frame.empty()) 
            {
                if (!sendFrame(frame)) 
                {
                    std::cerr << "客户端连接已断开" << std::endl;
                    return;
                }


                if(num_frames > 2094967296)
                {
                    num_frames = 1;
                }
                std::cout << "已发送帧: " << num_frames << std::endl;
                num_frames++;
                
                // 控制帧率
                usleep(1000000 / 30); // 约30fps
            }
        }
        
        void closeClient() 
        {
            if (client_fd != -1) 
            {
                close(client_fd);
                client_fd = -1;
                std::cout << "警告！相机故障！" << std::endl;
                std::cout << "客户端连接已关闭" << std::endl;
            }

        }

        bool isRunning() const {
            return is_running;
        }
};

class XianDjVisualPlatformMvcs01610gcColorRosServer
{
public:
    XianDjVisualPlatformMvcs01610gcColorRosServer() : g_bExit(false)
    {
        // 初始化节点句柄
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 从参数服务器获取私有参数[10](@ref)
        private_nh.param<int>("camera_index", camera_index_, 0);
        
        ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosServer initialized with camera_index: %d", 
                 camera_index_);
        
        // 初始化相机
        if (!initializeCamera()) {
            ROS_ERROR("Failed to initialize camera!");
            return;
        }

        if (!server.start()) 
        {
            return;
        }
        
        // 创建定时器用于发布心跳信号[10](@ref)
        timer_heart_beat_ = nh.createWallTimer(ros::WallDuration(1.0), 
                                              &XianDjVisualPlatformMvcs01610gcColorRosServer::timerHeartBeatCallback, this);
        
        // 创建定时器用于图像采集和发布
        timer_capture_ = nh.createWallTimer(ros::WallDuration(0.033),  // 约30Hz
                                         &XianDjVisualPlatformMvcs01610gcColorRosServer::timerCaptureCallback, this);
    }
    
    ~XianDjVisualPlatformMvcs01610gcColorRosServer()
    {
        cleanupCamera();
    }
    
    void timerHeartBeatCallback(const ros::WallTimerEvent& event)
    {
        // // 发布心跳信号到参数服务器[10](@ref)
        // private_nh_.setParam("xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat", heart_beat_counter_);
        // heart_beat_counter_ = heart_beat_counter_ > 1000 ? 0 : (heart_beat_counter_ + 1);
        ros::param::get("/xian_dj_visual_platform_params_server/xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat", xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat); 
        std::cout << "xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat: " << xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat << std::endl;
        counter = counter > 1000 ? 0 : (counter + 1);
        ros::param::set("/xian_dj_visual_platform_params_server/xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat", counter);  // 自行替换
    }
    
    void timerCaptureCallback(const ros::WallTimerEvent& event)
    {
        if (g_bExit || camera_handle_ == nullptr) {
            return;
        }
        
        // 获取图像帧
        MV_FRAME_OUT stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
        
        int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);
        if (nRet == MV_OK) {
            // 转换为cv::Mat
            cv::Mat imageMat = convertToMat(&stImageInfo);
            
            if (!imageMat.empty()) 
            {
                // 发布ROS图像消息[6,8](@ref)
                // publishImage(imageMat);
                
                // ROS_DEBUG("Published image: Width[%d], Height[%d]", 
                //          imageMat.cols, imageMat.rows);
                if(server.isRunning())
                {
                    // 接受客户端连接
                    if (!server.isClientConnected()) 
                    {
                        if (!server.acceptClient()) 
                        {
                            return;
                        }
                    }

                    // 发送图像
                    server.processVideo(imageMat);
                }
            }
            
            MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
        } else {
            ROS_WARN_THROTTLE(5.0, "Failed to get image buffer: 0x%x", nRet);
        }
    }
    
    void run()
    {
        ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosServer started successfully");
        ros::AsyncSpinner spinner(2);  // 使用2个线程
        spinner.start();
        ros::waitForShutdown();
    }

private:
    // 申明图像发送TCP server
    TCPServer server;
    int counter = 0;
    int xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server_heart_beat = 0;

    bool initializeCamera()
    {
        int nRet = MV_OK;
        
        do {
            // 初始化SDK
            nRet = MV_CC_Initialize();
            if (MV_OK != nRet) {
                ROS_ERROR("Initialize SDK failed! nRet [0x%x]", nRet);
                return false;
            }
            
            // 枚举设备
            MV_CC_DEVICE_INFO_LIST stDeviceList;
            memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
            
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            if (MV_OK != nRet) {
                ROS_ERROR("MV_CC_EnumDevices failed! nRet [0x%x]", nRet);
                return false;
            }
            
            if (stDeviceList.nDeviceNum == 0) {
                ROS_ERROR("No camera devices found!");
                return false;
            }
            
            if (camera_index_ >= stDeviceList.nDeviceNum) {
                ROS_ERROR("Camera index %d is out of range. Only %d devices found.", 
                         camera_index_, stDeviceList.nDeviceNum);
                return false;
            }
            
            // 打印设备信息
            for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
                ROS_INFO("[device %d]:", i);
                printDeviceInfo(stDeviceList.pDeviceInfo[i]);
            }
            
            // 创建句柄
            nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[camera_index_]);
            if (MV_OK != nRet) {
                ROS_ERROR("MV_CC_CreateHandle failed! nRet [0x%x]", nRet);
                return false;
            }
            
            // 打开设备
            nRet = MV_CC_OpenDevice(camera_handle_);
            if (MV_OK != nRet) {
                ROS_ERROR("MV_CC_OpenDevice failed! nRet [0x%x]", nRet);
                return false;
            }
            
            // 设置网络包大小（仅对GigE相机有效）
            if (stDeviceList.pDeviceInfo[camera_index_]->nTLayerType == MV_GIGE_DEVICE) {
                int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
                if (nPacketSize > 0) {
                    nRet = MV_CC_SetIntValueEx(camera_handle_, "GevSCPSPacketSize", nPacketSize);
                    if (nRet != MV_OK) {
                        ROS_WARN("Set Packet Size failed nRet [0x%x]", nRet);
                    }
                }
            }
            
            // 设置触发模式为off（连续采集）
            nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
            if (MV_OK != nRet) {
                ROS_ERROR("Set TriggerMode failed! nRet [0x%x]", nRet);
                return false;
            }
            
            // 开始取流
            nRet = MV_CC_StartGrabbing(camera_handle_);
            if (MV_OK != nRet) {
                ROS_ERROR("MV_CC_StartGrabbing failed! nRet [0x%x]", nRet);
                return false;
            }
            
            ROS_INFO("Camera initialized successfully");
            return true;
            
        } while (0);
        
        return false;
    }
    
    void cleanupCamera()
    {
        if (camera_handle_ != nullptr) {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
        }
        MV_CC_Finalize();
    }
    
    cv::Mat convertToMat(MV_FRAME_OUT* pFrame)
    {
        if (pFrame == nullptr || pFrame->pBufAddr == nullptr) {
            return cv::Mat();
        }
        
        int width = pFrame->stFrameInfo.nWidth;
        int height = pFrame->stFrameInfo.nHeight;
        cv::Mat result;
        
        switch (pFrame->stFrameInfo.enPixelType) {
            case PixelType_Gvsp_Mono8:
                result = cv::Mat(height, width, CV_8UC1, pFrame->pBufAddr);
                break;
                
            case PixelType_Gvsp_RGB8_Packed:
                result = cv::Mat(height, width, CV_8UC3, pFrame->pBufAddr);
                cv::cvtColor(result, result, cv::COLOR_RGB2BGR);
                break;
                
            case PixelType_Gvsp_BGR8_Packed:
                result = cv::Mat(height, width, CV_8UC3, pFrame->pBufAddr);
                break;
                
            case PixelType_Gvsp_BayerRG8:
                {
                    cv::Mat bayerMat(height, width, CV_8UC1, pFrame->pBufAddr);
                    cv::cvtColor(bayerMat, result, cv::COLOR_BayerRG2BGR);
                }
                break;
                
            default:
                ROS_WARN("Unsupported pixel format: 0x%lx", pFrame->stFrameInfo.enPixelType);
                return cv::Mat();
        }
        
        return result;
    }
    
    // void publishImage(const cv::Mat& image)
    // {
    //     if (image.empty()) {
    //         return;
    //     }
        
    //     try {
    //         // 使用cv_bridge转换图像[6,8](@ref)
    //         std_msgs::Header header;
    //         header.stamp = ros::Time::now();
    //         header.frame_id = "hik_camera";
            
    //         sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    //         image_pub_.publish(msg);
    //     }
    //     catch (cv_bridge::Exception& e) {
    //         ROS_ERROR("cv_bridge exception: %s", e.what());
    //     }
    // }
    
    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    {
        if (pstMVDevInfo == nullptr) {
            return false;
        }
        
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
            
            ROS_INFO("Device Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
            ROS_INFO("CurrentIp: %d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
            ROS_INFO("UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            ROS_INFO("Device Model Name: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
            ROS_INFO("UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        
        return true;
    }

private:
    ros::NodeHandle private_nh_;
    ros::WallTimer timer_heart_beat_;
    ros::WallTimer timer_capture_;
    
    void* camera_handle_ = nullptr;
    bool g_bExit;
    
    // 参数[10](@ref)
    int camera_index_ = 0;

    int heart_beat_counter_ = 0;
};

int main(int argc, char** argv)
{
    // 初始化ROS节点[4](@ref)
    ros::init(argc, argv, "xian_dj_visual_platform_mv_cs016_10gc_color_tcp_server");
    
    try 
    {
        XianDjVisualPlatformMvcs01610gcColorRosServer camera_node;
        camera_node.run();
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR("XianDjVisualPlatformMvcs01610gcColorRosServer exception: %s", e.what());
        return -1;
    }
    
    ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosServer shutdown successfully");
    return 0;
}