#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <string>
#include <sstream>
#include <iomanip>

class XianDjRobotgo1StateDisplay
{
    public:
        XianDjRobotgo1StateDisplay()
        {
            // 创建一个ROS节点句柄
            // ros::NodeHandle nh;
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_state_display_heart_beat", xian_dj_robotgo1_state_display_heart_beat); 
            std::cout << "xian_dj_robotgo1_state_display_heart_beat: " << xian_dj_robotgo1_state_display_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_state_display_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {   
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", xian_dj_robotgo1_display_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_error_code", xian_dj_robotgo1_error_code); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level", xian_dj_robotgo1_battery_level); 
            light_timer_counter++;
            display_timer_counter++;

            std::string format_battery_level = "$001,P" + xian_dj_robotgo1_battery_level + "#";
            printf("xian_dj_robotgo1_display_mode: %d \n",xian_dj_robotgo1_display_mode);
            switch(xian_dj_robotgo1_display_mode)
            {
                case 0:
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0);
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0); 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", format_battery_level); // std::string
                    printf("xian_dj_robotgo1_battery_level: %s \n", format_battery_level.c_str());
                case 1: //红灯间隔一秒闪烁，电量、报错间隔五秒交替闪烁
                    if(light_timer_counter > 50)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 1);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        else
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 249)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level);
                        }
                        else
                        {
                            double error_code = xian_dj_robotgo1_error_code / 100;
                            std::string format_error_code = doubleToFixedWidthString(error_code, 2, 2);
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "E" + format_error_code);
                        }
                        display_state =!display_state;
                    }
                    
                case 2: //绿灯间隔一秒闪烁，电量、模式间隔五秒闪烁
                    if(light_timer_counter > 50)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        else
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 249)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level);
                        }
                        else
                        {
                            std::string format_m_mode = intToFixedWidthString(xian_dj_robotgo1_m_system_mode, 2);
                            std::string format_s_mode = intToFixedWidthString(xian_dj_robotgo1_s_system_mode, 2);
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "A" + format_m_mode + "." + format_s_mode);
                        }
                        display_state =!display_state;
                    }
                
                case 3: //黄灯间隔一秒闪烁，电量、模式间隔五秒闪烁
                    if(light_timer_counter > 50)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 1); 
                        }
                        else
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 0); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 249)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level);
                        }
                        else
                        {
                            std::string format_m_mode = intToFixedWidthString(xian_dj_robotgo1_m_system_mode, 2);
                            std::string format_s_mode = intToFixedWidthString(xian_dj_robotgo1_s_system_mode, 2);
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "C" + format_m_mode + "." + format_s_mode);
                        }
                        display_state =!display_state;
                    }

                case 4: //绿灯常亮，电量、模式间隔五秒闪烁
                    if(light_timer_counter > 50)
                    {
                        light_timer_counter = 0;
                        if (lighter_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        else
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);  
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                        }
                        lighter_state =!lighter_state;
                    }
                    if(display_timer_counter > 249)
                    {
                        display_timer_counter = 0;
                        if (display_state)
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level);
                        }
                        else
                        {
                            std::string format_m_mode = intToFixedWidthString(xian_dj_robotgo1_m_system_mode, 2);
                            std::string format_s_mode = intToFixedWidthString(xian_dj_robotgo1_s_system_mode, 2);
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "C" + format_m_mode + "." + format_s_mode);
                        }
                        display_state =!display_state;
                    }

                case 5: 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 1);

                case 6: 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);
            }
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
        int counter = 0;
        int xian_dj_robotgo1_state_display_heart_beat = 0;

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

    xian_dj_robotgo1_state_display.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1StateDisplay::m_timer_heart_beat_func, &xian_dj_robotgo1_state_display);
    xian_dj_robotgo1_state_display.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjRobotgo1StateDisplay::m_timer_control_func, &xian_dj_robotgo1_state_display);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}