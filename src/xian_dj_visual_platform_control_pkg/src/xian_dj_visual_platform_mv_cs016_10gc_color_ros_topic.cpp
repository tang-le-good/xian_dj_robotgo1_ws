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


class XianDjVisualPlatformMvcs01610gcColorRosTopic
{
public:
    XianDjVisualPlatformMvcs01610gcColorRosTopic() : g_bExit(false)
    {
        // 初始化节点句柄
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 从参数服务器获取私有参数[10](@ref)
        private_nh.param<int>("camera_index", camera_index_, 0);
        private_nh.param<std::string>("topic_name", topic_name_, "hik_camera/image_raw");
        
        ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosTopic initialized with camera_index: %d, topic_name: %s", 
                 camera_index_, topic_name_.c_str());
        
        // 初始化图像传输[6,8](@ref)
        image_transport::ImageTransport it(nh);
        image_pub_ = it.advertise(topic_name_, 1);
        
        // 初始化相机
        if (!initializeCamera()) {
            ROS_ERROR("Failed to initialize camera!");
            return;
        }
        
        // 创建定时器用于发布心跳信号[10](@ref)
        timer_heart_beat_ = nh.createWallTimer(ros::WallDuration(1.0), 
                                              &XianDjVisualPlatformMvcs01610gcColorRosTopic::timerHeartBeatCallback, this);
        
        // 创建定时器用于图像采集和发布
        timer_capture_ = nh.createWallTimer(ros::WallDuration(0.033),  // 约30Hz
                                         &XianDjVisualPlatformMvcs01610gcColorRosTopic::timerCaptureCallback, this);
    }
    
    ~XianDjVisualPlatformMvcs01610gcColorRosTopic()
    {
        cleanupCamera();
    }
    
    void timerHeartBeatCallback(const ros::WallTimerEvent& event)
    {
        // 发布心跳信号到参数服务器[10](@ref)
        private_nh_.setParam("xian_dj_visual_platform_mv_cs016_10gc_color_ros_topic_heart_beat", heart_beat_counter_);
        heart_beat_counter_ = heart_beat_counter_ > 1000 ? 0 : (heart_beat_counter_ + 1);
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
            
            if (!imageMat.empty()) {
                // 发布ROS图像消息[6,8](@ref)
                publishImage(imageMat);
                
                ROS_DEBUG("Published image: Width[%d], Height[%d]", 
                         imageMat.cols, imageMat.rows);
            }
            
            MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
        } else {
            ROS_WARN_THROTTLE(5.0, "Failed to get image buffer: 0x%x", nRet);
        }
    }
    
    void run()
    {
        ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosTopic started successfully");
        ros::AsyncSpinner spinner(2);  // 使用2个线程
        spinner.start();
        ros::waitForShutdown();
    }

private:
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
    
    void publishImage(const cv::Mat& image)
    {
        if (image.empty()) {
            return;
        }
        
        try {
            // 使用cv_bridge转换图像[6,8](@ref)
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "hik_camera";
            
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
            image_pub_.publish(msg);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    
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
    image_transport::Publisher image_pub_;
    ros::WallTimer timer_heart_beat_;
    ros::WallTimer timer_capture_;
    
    void* camera_handle_ = nullptr;
    bool g_bExit;
    
    // 参数[10](@ref)
    int camera_index_ = 0;
    std::string topic_name_ = "hik_camera/image_raw";
    int heart_beat_counter_ = 0;
};

int main(int argc, char** argv)
{
    // 初始化ROS节点[4](@ref)
    ros::init(argc, argv, "xian_dj_visual_platform_mv_cs016_10gc_color_ros_topic");
    
    try {
        XianDjVisualPlatformMvcs01610gcColorRosTopic camera_node;
        camera_node.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("XianDjVisualPlatformMvcs01610gcColorRosTopic exception: %s", e.what());
        return -1;
    }
    
    ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosTopic shutdown successfully");
    return 0;
}