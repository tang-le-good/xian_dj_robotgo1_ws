#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <string>
#include <sstream>

class XianDjRobotgo1StateDisplay
{
    public:
        XianDjRobotgo1StateDisplay()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
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

            light_timer_counter++;
            display_timer_counter++;
            switch(xian_dj_robotgo1_display_mode)
            {
                case 0:
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", 1); 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", 0); 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level + "%"); 

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
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level+ "%");
                        }
                        else
                        {
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "E" + std::to_string(xian_dj_robotgo1_error_code) + "%");
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
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level + "%");
                        }
                        else
                        {
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "A" + std::to_string(xian_dj_robotgo1_m_system_mode) + std::to_string(xian_dj_robotgo1_s_system_mode));
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
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level + "%");
                        }
                        else
                        {
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "C" + std::to_string(xian_dj_robotgo1_m_system_mode) + std::to_string(xian_dj_robotgo1_s_system_mode));
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
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "P" + xian_dj_robotgo1_battery_level + "%");
                        }
                        else
                        {
                             ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", "C" + std::to_string(xian_dj_robotgo1_m_system_mode) + std::to_string(xian_dj_robotgo1_s_system_mode));
                        }
                        display_state =!display_state;
                    }

                case 5: 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 1);

                case 6: 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", 0);
            }
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
        std::string xian_dj_robotgo1_battery_level = "OFF";

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