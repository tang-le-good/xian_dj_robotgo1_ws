#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<iostream>
#include<sys/types.h>

class XianDjVisualPlatformMvcs01610gcColorRosSub
{
public:
    XianDjVisualPlatformMvcs01610gcColorRosSub() : nh_("~"), it_(nh_)
    {
        // 使用私有节点句柄，避免命名冲突
        std::string topic_name;
        nh_.param<std::string>("image_topic", topic_name, "xian_dj_hk_camera_0");
        
        // 增加队列大小，减少连接压力
        image_sub_ = it_.subscribe(topic_name, 1, &XianDjVisualPlatformMvcs01610gcColorRosSub::imageCallback, this);
        
        ROS_INFO("XianDjVisualPlatformMvcs01610gcColorRosSub initialized, subscribing to: %s", topic_name.c_str());
    }

    void timerCallback(const ros::WallTimerEvent& event)
    {
        counter_ = (counter_ > 1000) ? 0 : (counter_ + 1);
        ROS_DEBUG("xian_spreader_images_show_heart_beat: %d", counter_);
    }

private:      
    int counter_ = 0;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            
            if(image.empty()) {
                ROS_WARN("Received empty image");
                return;
            }
            
            // 显示图像
            cv::imshow("Spreader Image View", image);
            cv::waitKey(1);
            printf("w: %d, h: %d \n", image.cols, image.rows);
            
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    } 
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xian_dj_visual_platform_mv_cs016_10gc_color_ros_sub");
    
    // 增加初始化日志
    ROS_INFO("Initializing xian_dj_visual_platform_mv_cs016_10gc_color_ros_sub...");
    
    try {
        XianDjVisualPlatformMvcs01610gcColorRosSub image_processor;
        
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(2); // 使用2个线程，而不是0个
        spinner.start();
        
        // 创建定时器
        auto timer = nh.createWallTimer(ros::WallDuration(1.0), 
                                      &XianDjVisualPlatformMvcs01610gcColorRosSub::timerCallback, 
                                      &image_processor);
        
        ROS_INFO("Node started successfully");
        ros::waitForShutdown();
        
    } catch (const std::exception& e) {
        ROS_FATAL("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}