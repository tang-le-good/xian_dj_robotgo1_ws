#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <sensor_msgs/Joy.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_local_controller.h"
#include <std_msgs/UInt16.h>

class XianDjRobotgo1LocalController
{
    public:
        XianDjRobotgo1LocalController()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            controller_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XianDjRobotgo1LocalController::controller_callback, this);
            xian_dj_robotgo1_local_controller_state_pub = nh.advertise<std_msgs::UInt16>("xian_dj_robotgo1_local_controller_state_msg", 1);
            xian_dj_robotgo1_local_controller_pub = nh.advertise<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_local_controller>("xian_dj_robotgo1_local_controller_msg", 1); 
        }

        ros::WallTimer m_timer_heart_beat;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            xian_dj_robotgo1_local_controller_heart_beat = xian_dj_robotgo1_local_controller_heart_beat > 1000 ? 0 : (xian_dj_robotgo1_local_controller_heart_beat + 1);
            std::cout << "xian_dj_robotgo1_local_controller_heart_beat: " << xian_dj_robotgo1_local_controller_heart_beat << std::endl;
            heart_beat_msg.data = xian_dj_robotgo1_local_controller_heart_beat;
            xian_dj_robotgo1_local_controller_state_pub.publish(heart_beat_msg);
        }


        void controller_callback(const sensor_msgs::Joy::ConstPtr &Joy)
        {
            axes_6_common = Joy->axes[6];
            switch (axes_6_common)
            {
                case 0:
                    xian_dj_local_controller_left_cmd = 0;
                    xian_dj_local_controller_right_cmd = 0;
                    break;
                case 1:
                    xian_dj_local_controller_left_cmd = 1;
                    xian_dj_local_controller_right_cmd = 0;
                    break;  
                case -1:
                    xian_dj_local_controller_left_cmd = 0;
                    xian_dj_local_controller_right_cmd = 1;
                    break;
            }

            axes_7_common = Joy->axes[7];
            switch (axes_7_common)
            {
                case 0:
                    xian_dj_local_controller_up_cmd = 0;
                    xian_dj_local_controller_down_cmd = 0;
                    break;
                case 1:
                    xian_dj_local_controller_up_cmd = 1;
                    xian_dj_local_controller_down_cmd = 0;
                    break;  
                case -1:
                    xian_dj_local_controller_up_cmd = 0;
                    xian_dj_local_controller_down_cmd = 1;
                    break;
            }
            // xian_dj_local_controller_left_cmd = Joy->axes[6];
            // xian_dj_local_controller_right_cmd = Joy->axes[6];
            // xian_dj_local_controller_up_cmd = Joy->axes[7];
            // xian_dj_local_controller_down_cmd = Joy->axes[7];
            xian_dj_local_controller_x_cmd = Joy->buttons[3];
            xian_dj_local_controller_b_cmd = Joy->buttons[1];
            xian_dj_local_controller_y_cmd = Joy->buttons[4];
            xian_dj_local_controller_a_cmd = Joy->buttons[0];
            xian_dj_local_controller_left_rocker_x_cmd = Joy->axes[0];
            xian_dj_local_controller_left_rocker_y_cmd = Joy->axes[1];
            xian_dj_local_controller_right_rocker_x_cmd = Joy->axes[2];
            xian_dj_local_controller_right_rocker_y_cmd = Joy->axes[3];
            xian_dj_local_controller_r1_cmd = Joy->buttons[7];
            xian_dj_local_controller_r2_cmd = Joy->buttons[9];
            xian_dj_local_controller_l1_cmd = Joy->buttons[6];
            xian_dj_local_controller_l2_cmd = Joy->buttons[8];

            pub_msg.xian_dj_local_controller_left_cmd = xian_dj_local_controller_left_cmd ;
            pub_msg.xian_dj_local_controller_right_cmd = xian_dj_local_controller_right_cmd ;
            pub_msg.xian_dj_local_controller_up_cmd = xian_dj_local_controller_up_cmd ;
            pub_msg.xian_dj_local_controller_down_cmd= xian_dj_local_controller_down_cmd;
            pub_msg.xian_dj_local_controller_x_cmd = xian_dj_local_controller_x_cmd ;
            pub_msg.xian_dj_local_controller_b_cmd = xian_dj_local_controller_b_cmd ;
            pub_msg.xian_dj_local_controller_y_cmd = xian_dj_local_controller_y_cmd ;
            pub_msg.xian_dj_local_controller_a_cmd = xian_dj_local_controller_a_cmd ;
            pub_msg.xian_dj_local_controller_left_rocker_x_cmd = xian_dj_local_controller_left_rocker_x_cmd ;
            pub_msg.xian_dj_local_controller_left_rocker_y_cmd = xian_dj_local_controller_left_rocker_y_cmd ;
            pub_msg.xian_dj_local_controller_right_rocker_x_cmd = xian_dj_local_controller_right_rocker_x_cmd ;
            pub_msg.xian_dj_local_controller_right_rocker_y_cmd = xian_dj_local_controller_right_rocker_y_cmd ;
            pub_msg.xian_dj_local_controller_r1_cmd = xian_dj_local_controller_r1_cmd ;
            pub_msg.xian_dj_local_controller_r2_cmd = xian_dj_local_controller_r2_cmd ;
            pub_msg.xian_dj_local_controller_l1_cmd = xian_dj_local_controller_l1_cmd ;
            pub_msg.xian_dj_local_controller_l2_cmd = xian_dj_local_controller_l2_cmd ;
            xian_dj_robotgo1_local_controller_pub.publish(pub_msg);

            printf("hello \n");
            
        }
        void someMethod() 
        {
            auto now = std::chrono::system_clock::now();
            auto secs = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) % 1000;
            std::cout << "HH:MM:SS.mmm: "
                    << std::put_time(std::localtime(&secs), "%H:%M:%S")
                    << '.' << std::setfill('0') << std::setw(3) << ms.count() << '\n';
        }

    private:
        ros::Subscriber controller_sub;
        ros::Publisher xian_dj_robotgo1_local_controller_state_pub;
        ros::Publisher xian_dj_robotgo1_local_controller_pub;
        xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_local_controller pub_msg;
        std_msgs::UInt16 heart_beat_msg;
        int xian_dj_robotgo1_local_controller_heart_beat = 0;

        int xian_dj_local_controller_left_cmd = 0;
        int xian_dj_local_controller_right_cmd = 0;
        int xian_dj_local_controller_up_cmd = 0;
        int xian_dj_local_controller_down_cmd =0;
        int xian_dj_local_controller_x_cmd = 0;
        int xian_dj_local_controller_b_cmd = 0;
        int xian_dj_local_controller_y_cmd = 0;
        int xian_dj_local_controller_a_cmd = 0;

        double xian_dj_local_controller_left_rocker_x_cmd = 0;
        double xian_dj_local_controller_left_rocker_y_cmd = 0;
        double xian_dj_local_controller_right_rocker_x_cmd = 0;
        double xian_dj_local_controller_right_rocker_y_cmd = 0;

        int xian_dj_local_controller_r1_cmd = 0;
        int xian_dj_local_controller_r2_cmd = 0;
        int xian_dj_local_controller_l1_cmd = 0;
        int xian_dj_local_controller_l2_cmd = 0;

        int axes_6_common = 0;
        int axes_7_common = 0;
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_local_controller");
    XianDjRobotgo1LocalController xian_dj_robotgo1_local_controller;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_robotgo1_local_controller.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1LocalController::m_timer_heart_beat_func, &xian_dj_robotgo1_local_controller);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}