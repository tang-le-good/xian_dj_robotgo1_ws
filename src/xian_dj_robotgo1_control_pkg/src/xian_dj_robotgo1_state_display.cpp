#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <string>
#include <sstream>
#include <iomanip>
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_state_display.h"
#include <std_msgs/UInt16.h>
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_lighter_displayer.h"


class XianDjRobotgo1StateDisplay
{
    public:
        XianDjRobotgo1StateDisplay()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            xian_dj_robotgo1_state_display_sub = nh.subscribe<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_state_display>("xian_dj_robotgo1_state_display_msg", 10, &XianDjRobotgo1StateDisplay::controller_callback, this);
            xian_dj_robotgo1_state_display_state_pub = nh.advertise<std_msgs::UInt16>("xian_dj_robotgo1_state_display_state_msg", 1);
            xian_dj_robotgo1_state_display_pub = nh.advertise<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_lighter_displayer>("xian_dj_robotgo1_lighter_displayer_msg", 1); 
        }


        ros::WallTimer m_timer_control;

        void controller_callback(const xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_state_display::ConstPtr &data)
        {
            xian_dj_robotgo1_display_mode = data->xian_dj_robotgo1_display_mode;
            xian_dj_robotgo1_error_code = data->xian_dj_robotgo1_error_code;
            xian_dj_robotgo1_m_system_mode = data->xian_dj_robotgo1_m_system_mode;
            xian_dj_robotgo1_s_system_mode = data->xian_dj_robotgo1_s_system_mode;
            xian_dj_robotgo1_battery_level = data->xian_dj_robotgo1_battery_level;
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {    
            // 发布心跳
            xian_dj_robotgo1_state_display_heart_beat = xian_dj_robotgo1_state_display_heart_beat > 1000 ? 0 : (xian_dj_robotgo1_state_display_heart_beat + 1);
            std::cout << "xian_dj_robotgo1_state_display_heart_beat: " << xian_dj_robotgo1_state_display_heart_beat << std::endl;
            heart_beat_msg.data = xian_dj_robotgo1_state_display_heart_beat;
            xian_dj_robotgo1_state_display_state_pub.publish(heart_beat_msg);

            light_timer_counter++;
            display_timer_counter++;

            // 电量格式化输出
            std::string format_battery_level = "$001,P" + xian_dj_robotgo1_battery_level + "#";

            // 故障代码格式化输出
            double error_code = xian_dj_robotgo1_error_code / 100;
            std::string format_error_code = "$001,E" + doubleToFixedWidthString(error_code, 2, 2) + "#";

            std::string format_m_mode = intToFixedWidthString(xian_dj_robotgo1_m_system_mode, 2);
            std::string format_s_mode = intToFixedWidthString(xian_dj_robotgo1_s_system_mode, 2);

            printf("xian_dj_robotgo1_display_mode: %d \n",xian_dj_robotgo1_display_mode);
            switch(xian_dj_robotgo1_display_mode)
            {
                case 0:
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_battery_level); // std::string
                    // printf("xian_dj_robotgo1_battery_level: %s \n", format_battery_level.c_str());
                    pub_msg.xian_dj_robotgo1_green_ligher_cmd = 1;
                    pub_msg.xian_dj_robotgo1_displayer_cmd = format_battery_level.c_str();
                    printf("case0 \n");
                    break;
                    
                
                case 1: //红灯间隔一秒闪烁，电量、报错间隔五秒交替闪烁
                    printf("case1 \n");
                    
                    if(light_timer_counter > 1)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 1);  
                            pub_msg.xian_dj_robotgo1_red_ligher_cmd = 1;
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            pub_msg.xian_dj_robotgo1_red_ligher_cmd = 0;
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 2)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_battery_level);
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_battery_level.c_str();
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_error_code);
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_error_code.c_str();
                        }
                        display_state =!display_state;
                    }
                    break;
                
                case 2: //绿灯间隔一秒闪烁，电量、模式间隔五秒闪烁
                    printf("case2 \n");
                    if(light_timer_counter > 1)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                            pub_msg.xian_dj_robotgo1_green_ligher_cmd = 1;
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            pub_msg.xian_dj_robotgo1_green_ligher_cmd = 0;
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 2)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            //  ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_battery_level);
                             pub_msg.xian_dj_robotgo1_displayer_cmd = format_battery_level.c_str();
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "$001,A" + format_m_mode + "." + format_s_mode + "#");
                            std::string format_m_s_str = "$001,A" + format_m_mode + "." + format_s_mode + "#";
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_m_s_str.c_str();
                        }
                        display_state =!display_state;
                    }
                    break;
                
                case 3: //黄灯间隔一秒闪烁，电量、模式间隔五秒闪烁
                    printf("case3 \n");
                    if(light_timer_counter > 1)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 1); 
                            pub_msg.xian_dj_robotgo1_green_ligher_cmd = 0;
                            pub_msg.xian_dj_robotgo1_red_ligher_cmd = 0;
                            pub_msg.xian_dj_robotgo1_yellow_ligher_cmd = 1;
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                            pub_msg.xian_dj_robotgo1_green_ligher_cmd = 0;
                            pub_msg.xian_dj_robotgo1_red_ligher_cmd = 0;
                            pub_msg.xian_dj_robotgo1_yellow_ligher_cmd = 0;
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 2)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_battery_level);
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_battery_level.c_str();
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "$001,C" + format_m_mode + "." + format_s_mode + "#");
                            std::string format_m_s_str = "$001,C" + format_m_mode + "." + format_s_mode + "#";
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_m_s_str.c_str();
                        }
                        display_state =!display_state;
                    }
                    break;

                case 4: //绿灯常亮，电量、模式间隔五秒闪烁
                    printf("case4 \n");
                    if(light_timer_counter > 1)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                            pub_msg.xian_dj_robotgo1_green_ligher_cmd = 1;
                            pub_msg.xian_dj_robotgo1_red_ligher_cmd = 0;
                            pub_msg.xian_dj_robotgo1_yellow_ligher_cmd = 0;
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                            pub_msg.xian_dj_robotgo1_green_ligher_cmd = 1;
                            pub_msg.xian_dj_robotgo1_red_ligher_cmd = 0;
                            pub_msg.xian_dj_robotgo1_yellow_ligher_cmd = 0;
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 2)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_battery_level);
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_battery_level.c_str();
                        }
                        else
                        {
                            // std::string format_m_mode = intToFixedWidthString(xian_dj_robotgo1_m_system_mode, 2);
                            // std::string format_s_mode = intToFixedWidthString(xian_dj_robotgo1_s_system_mode, 2);
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "$001,C" + format_m_mode + "." + format_s_mode + "#");
                            std::string format_m_s_str = "$001,C" + format_m_mode + "." + format_s_mode + "#";
                            pub_msg.xian_dj_robotgo1_displayer_cmd = format_m_s_str.c_str();
                        }
                        display_state =!display_state;
                    }
                    break;

                case 5: 
                    printf("case5 \n");
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 1);
                    pub_msg.xian_dj_robotgo1_red_ligher_cmd = 1;
                    break;

                case 6: 
                    printf("case6 \n");
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);
                    pub_msg.xian_dj_robotgo1_red_ligher_cmd = 0;
                    break;
            }
            xian_dj_robotgo1_state_display_pub.publish(pub_msg);
        }

        std::string intToFixedWidthString(int num, int width) 
        {
            std::ostringstream oss;
            oss << std::setw(width) << std::setfill('0') << num;
            return oss.str();
        }

        std::string doubleToFixedWidthString(double value, int integerWidth, int decimalWidth) 
        {
            std::ostringstream oss;
            
            // 设置固定小数表示和精度
            oss << std::fixed << std::setprecision(decimalWidth) << std::abs(value);
            
            std::string numberStr = oss.str();
            
            // 分离整数和小数部分
            size_t dotPos = numberStr.find('.');
            std::string intPart, decimalPart;
            
            if (dotPos != std::string::npos) {
                intPart = numberStr.substr(0, dotPos);
                decimalPart = numberStr.substr(dotPos + 1);
            } else {
                intPart = numberStr;
                decimalPart = "";
            }
            
            // 处理整数部分：用0填充到指定宽度
            if (value < 0) {
                intPart = std::string(integerWidth - intPart.length() - 1, '0') + intPart;
                intPart = "-" + intPart;
            } else {
                intPart = std::string(integerWidth - intPart.length(), '0') + intPart;
            }
            
            // 处理小数部分：用0填充到指定宽度
            if (decimalPart.length() < decimalWidth) {
                decimalPart += std::string(decimalWidth - decimalPart.length(), '0');
            } else if (decimalPart.length() > decimalWidth) {
                decimalPart = decimalPart.substr(0, decimalWidth);
            }
            
            return intPart + "." + decimalPart;
        }

    private:
        int xian_dj_robotgo1_state_display_heart_beat = 0;
        ros::Subscriber xian_dj_robotgo1_state_display_sub;
        ros::Publisher xian_dj_robotgo1_state_display_state_pub;
        ros::Publisher xian_dj_robotgo1_state_display_pub;
        xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_lighter_displayer pub_msg;
        std_msgs::UInt16 heart_beat_msg;

        int xian_dj_robotgo1_display_mode = 0;
        int xian_dj_robotgo1_error_code = 0;
        int xian_dj_robotgo1_m_system_mode = 0;
        int xian_dj_robotgo1_s_system_mode = 0;

        int light_timer_counter=0;
        int display_timer_counter=0;
        bool lighter_state=true;
        bool display_state=true;

        int xian_dj_robotgo1_red_ligher_cmd = 0;
        int xian_dj_robotgo1_green_ligher_cmd = 0;
        int xian_dj_robotgo1_yellow_ligher_cmd = 0;
        std::string xian_dj_robotgo1_displayer_cmd = "OFF";
        std::string xian_dj_robotgo1_battery_level = "0044";

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_state_display");
    XianDjRobotgo1StateDisplay xian_dj_robotgo1_state_display;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_robotgo1_state_display.m_timer_control = nh_2.createWallTimer(ros::WallDuration(1), &XianDjRobotgo1StateDisplay::m_timer_control_func, &xian_dj_robotgo1_state_display);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}