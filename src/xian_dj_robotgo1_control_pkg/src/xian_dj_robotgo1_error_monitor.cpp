#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

class XianDjRobotgo1ErrorMonitor
{
    public:
        XianDjRobotgo1ErrorMonitor()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            xian_dj_car_chassis_state_sub = nh.subscribe<std_msgs::String>("xian_dj_car_chassis_error_monitor_msg", 10, &XianDjRobotgo1ErrorMonitor::car_chassis_error_monitor_callback, this);
            xian_dj_tele_op_error_monitor_sub = nh.subscribe<std_msgs::String>("xian_dj_tele_op_error_monitor_msg", 10, &XianDjRobotgo1ErrorMonitor::tele_op_error_monitor_callback, this);
            xian_dj_retractable_platform_error_monitor_sub = nh.subscribe<std_msgs::String>("xian_dj_retractable_platform_error_monitor_msg", 10, &XianDjRobotgo1ErrorMonitor::retractable_platform_error_monitor_callback, this);
            xian_dj_stewart_platform_error_monitor_sub = nh.subscribe<std_msgs::String>("xian_dj_stewart_platform_error_monitor_msg", 10, &XianDjRobotgo1ErrorMonitor::stewart_platform_error_monitor_callback, this);

            xian_dj_robotgo1_switch_mode_state_sub = nh.subscribe<std_msgs::UInt16>("xian_dj_robotgo1_switch_mode_state_msg", 10, &XianDjRobotgo1ErrorMonitor::xian_dj_robotgo1_switch_mode_state_callback, this);
            xian_dj_robotgo1_state_display_state_sub = nh.subscribe<std_msgs::UInt16>("xian_dj_robotgo1_state_display_state_msg", 10, &XianDjRobotgo1ErrorMonitor::xian_dj_robotgo1_state_display_state_callback, this);
            xian_dj_robotgo1_local_controller_state_sub = nh.subscribe<std_msgs::UInt16>("xian_dj_robotgo1_local_controller_state_msg", 10, &XianDjRobotgo1ErrorMonitor::xian_dj_robotgo1_local_controller_state_callback, this);
            xian_dj_robotgo1_lighter_displayer_state_sub = nh.subscribe<std_msgs::UInt16>("xian_dj_robotgo1_lighter_displayer_state_msg", 10, &XianDjRobotgo1ErrorMonitor::xian_dj_robotgo1_lighter_displayer_state_callback, this);
            xian_dj_robotgo1_control_state_sub = nh.subscribe<std_msgs::UInt16>("xian_dj_robotgo1_control_state_msg", 10, &XianDjRobotgo1ErrorMonitor::xian_dj_robotgo1_control_state_callback, this);
            // xian_dj_robotgo1_battery_level_state_sub = nh.subscribe<std_msgs::UInt16>("xian_dj_robotgo1_control_state_msg", 10, &XianDjRobotgo1ErrorMonitor::xian_dj_robotgo1_control_state_callback, this);

            xian_dj_robotgo1_error_monitor_pub = nh.advertise<std_msgs::String>("xian_dj_robotgo1_error_monitor_msg", 1);

        }

        ros::WallTimer m_timer_control;

        void car_chassis_error_monitor_callback(const std_msgs::String::ConstPtr &data)
        {
            std::stringstream ss(data->data);
            ss >> xian_dj_car_chassis_error_code;          // 只读第一个数字
            ROS_INFO("xian_dj_car_chassis_error_code = %d", xian_dj_car_chassis_error_code);
        }

        void tele_op_error_monitor_callback(const std_msgs::String::ConstPtr &data)
        {
            std::stringstream ss(data->data);
            ss >> xian_dj_tele_op_error_code;          // 只读第一个数字
            ROS_INFO("xian_dj_tele_op_error_code = %d", xian_dj_tele_op_error_code);
        }

        void retractable_platform_error_monitor_callback(const std_msgs::String::ConstPtr &data)
        {
            std::stringstream ss(data->data);
            ss >> xian_dj_retractable_platform_error_code;          // 只读第一个数字
            ROS_INFO("xian_dj_retractable_platform_error_code = %d", xian_dj_retractable_platform_error_code);
        }

        void stewart_platform_error_monitor_callback(const std_msgs::String::ConstPtr &data)
        {
            std::stringstream ss(data->data);
            ss >> xian_dj_stewart_platform_error_code;          // 只读第一个数字
            ROS_INFO("xian_dj_stewart_platform_error_code = %d", xian_dj_stewart_platform_error_code);
        }

        // 监控 xian_dj_robotgo1_switch_mode_heart_beat
        void xian_dj_robotgo1_switch_mode_state_callback(const std_msgs::UInt16::ConstPtr &data)
        {
            xian_dj_robotgo1_switch_mode_heart_beat = data->data;
            
            xian_dj_robotgo1_switch_mode_heart_beat_pre = xian_dj_robotgo1_switch_mode_heart_beat_cur;
            xian_dj_robotgo1_switch_mode_heart_beat_cur = xian_dj_robotgo1_switch_mode_heart_beat;
            if(xian_dj_robotgo1_switch_mode_heart_beat_pre == xian_dj_robotgo1_switch_mode_heart_beat_cur)
            {
                xian_dj_robotgo1_switch_mode_node_restart_flag ++;
            }
            else
            {
                xian_dj_robotgo1_switch_mode_node_restart_flag = 0;
            }
            if(xian_dj_robotgo1_switch_mode_node_restart_flag > time_out)
            {
                xian_dj_robotgo1_switch_mode_error = 1;
            }
            else
            {
                xian_dj_robotgo1_switch_mode_error = 0;
            }
        }

        // 监控 xian_dj_robotgo1_state_display_heart_beat
        void xian_dj_robotgo1_state_display_state_callback(const std_msgs::UInt16::ConstPtr &data)
        {
            xian_dj_robotgo1_state_display_heart_beat = data->data;
            xian_dj_robotgo1_state_display_heart_beat_pre = xian_dj_robotgo1_state_display_heart_beat_cur;
            xian_dj_robotgo1_state_display_heart_beat_cur = xian_dj_robotgo1_state_display_heart_beat;
            if(xian_dj_robotgo1_state_display_heart_beat_pre == xian_dj_robotgo1_state_display_heart_beat_cur)
            {
                xian_dj_robotgo1_state_display_node_restart_flag ++;
            }
            else
            {
                xian_dj_robotgo1_state_display_node_restart_flag = 0;
            }
            if(xian_dj_robotgo1_state_display_node_restart_flag > time_out)
            {
                xian_dj_robotgo1_state_display_error = 1;
            }
            else
            {
                xian_dj_robotgo1_state_display_error = 0;
            }
        }

        // 监控 xian_dj_robotgo1_local_controller_heart_beat
        void xian_dj_robotgo1_local_controller_state_callback(const std_msgs::UInt16::ConstPtr &data)
        {
            xian_dj_robotgo1_local_controller_heart_beat = data->data;
            xian_dj_robotgo1_local_controller_heart_beat_pre = xian_dj_robotgo1_local_controller_heart_beat_cur;
            xian_dj_robotgo1_local_controller_heart_beat_cur = xian_dj_robotgo1_local_controller_heart_beat;
            if(xian_dj_robotgo1_local_controller_heart_beat_pre == xian_dj_robotgo1_local_controller_heart_beat_cur)
            {
                xian_dj_robotgo1_local_controller_node_restart_flag ++;
            }
            else
            {
                xian_dj_robotgo1_local_controller_node_restart_flag = 0;
            }
            if(xian_dj_robotgo1_local_controller_node_restart_flag > time_out)
            {
                xian_dj_robotgo1_local_controller_error = 1;
            }
            else
            {
                xian_dj_robotgo1_local_controller_error = 0;
            }
        }

        // 监控 xian_dj_robotgo1_lighter_displayer_heart_beat
        void xian_dj_robotgo1_lighter_displayer_state_callback(const std_msgs::UInt16::ConstPtr &data)
        {
            xian_dj_robotgo1_lighter_displayer_heart_beat = data->data;
            xian_dj_robotgo1_lighter_displayer_heart_beat_pre = xian_dj_robotgo1_lighter_displayer_heart_beat_cur;
            xian_dj_robotgo1_lighter_displayer_heart_beat_cur = xian_dj_robotgo1_lighter_displayer_heart_beat;
            if(xian_dj_robotgo1_lighter_displayer_heart_beat_pre == xian_dj_robotgo1_lighter_displayer_heart_beat_cur)
            {
                xian_dj_robotgo1_lighter_displayer_node_restart_flag ++;
            }
            else
            {
                xian_dj_robotgo1_lighter_displayer_node_restart_flag = 0;
            }
            if(xian_dj_robotgo1_lighter_displayer_node_restart_flag > time_out)
            {
                xian_dj_robotgo1_lighter_displayer_error = 1;
            }
            else
            {
                xian_dj_robotgo1_lighter_displayer_error = 0;
            }

        }

        // 监控 xian_dj_robotgo1_control_heart_beat
        void xian_dj_robotgo1_control_state_callback(const std_msgs::UInt16::ConstPtr &data)
        {
            xian_dj_robotgo1_control_heart_beat = data->data;
            xian_dj_robotgo1_control_heart_beat_pre = xian_dj_robotgo1_control_heart_beat_cur;
            xian_dj_robotgo1_control_heart_beat_cur = xian_dj_robotgo1_control_heart_beat;
            if(xian_dj_robotgo1_control_heart_beat_pre == xian_dj_robotgo1_control_heart_beat_cur)
            {
                xian_dj_robotgo1_control_node_restart_flag ++;
            }
            else
            {
                xian_dj_robotgo1_control_node_restart_flag = 0;
            }
            if(xian_dj_robotgo1_control_node_restart_flag > time_out)
            {
                xian_dj_robotgo1_control_error =1;
            }
            else
            {
                xian_dj_robotgo1_control_error = 0;
            }
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {   
            error_sum = xian_dj_robotgo1_local_controller_error + xian_dj_robotgo1_state_display_error + xian_dj_robotgo1_switch_mode_error + xian_dj_robotgo1_control_error + xian_dj_robotgo1_lighter_displayer_error ;
            xian_dj_robotgo1_error_code = error_sum + 600;

            if(xian_dj_robotgo1_error_code != 600 )
            {
                std::string pub_sting = std::to_string(xian_dj_robotgo1_error_code) 
                                    + "   " + std::to_string(xian_dj_robotgo1_local_controller_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_state_display_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_switch_mode_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_control_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_lighter_displayer_error); 
                pub_msg.data = pub_sting.c_str();
                xian_dj_robotgo1_error_monitor_pub.publish(pub_msg);  
                return ;
            }
            else
            {
                if (xian_dj_car_chassis_error_code != 100)
                {
                    // printf("xian_dj_car_chassis_error_code: %d \n",xian_dj_car_chassis_error_code);
                    std::string pub_sting = std::to_string(xian_dj_car_chassis_error_code) 
                                    + "   " + std::to_string(xian_dj_robotgo1_local_controller_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_state_display_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_switch_mode_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_control_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_lighter_displayer_error); 
                    pub_msg.data = pub_sting.c_str();
                    xian_dj_robotgo1_error_monitor_pub.publish(pub_msg);  
                    return ;
                }
                else
                {
                    if(xian_dj_retractable_platform_error_code != 200)
                    {
                        std::string pub_sting = std::to_string(xian_dj_retractable_platform_error_code) 
                                    + "   " + std::to_string(xian_dj_robotgo1_local_controller_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_state_display_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_switch_mode_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_control_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_lighter_displayer_error); 
                        pub_msg.data = pub_sting.c_str();
                        xian_dj_robotgo1_error_monitor_pub.publish(pub_msg); 
                        return ;
                    }
                    else
                    {
                        if(xian_dj_stewart_platform_error_code != 300)
                        {
                            std::string pub_sting = std::to_string(xian_dj_stewart_platform_error_code) 
                                    + "   " + std::to_string(xian_dj_robotgo1_local_controller_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_state_display_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_switch_mode_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_control_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_lighter_displayer_error); 
                            pub_msg.data = pub_sting.c_str();
                            xian_dj_robotgo1_error_monitor_pub.publish(pub_msg); 
                            return ;
                        }
                        else
                        {
                            xian_dj_car_chassis_error_code = 0;
                            xian_dj_retractable_platform_error_code = 0;
                            xian_dj_stewart_platform_error_code = 0;
                            xian_dj_robotgo1_error_code = 0;
                            std::string pub_sting = std::to_string(xian_dj_stewart_platform_error_code) 
                                    + "   " + std::to_string(xian_dj_robotgo1_local_controller_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_state_display_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_switch_mode_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_control_error)
                                    + "   " + std::to_string(xian_dj_robotgo1_lighter_displayer_error); 
                            pub_msg.data = pub_sting.c_str();
                            xian_dj_robotgo1_error_monitor_pub.publish(pub_msg); 
                            return;
                        }
                    }
                }
            }
        }

    private:
        std::string command_kill_current_node = "";
        std::string command_restart_current_node = "";
        ros::Subscriber xian_dj_car_chassis_state_sub;
        ros::Subscriber xian_dj_tele_op_error_monitor_sub;
        ros::Subscriber xian_dj_retractable_platform_error_monitor_sub;
        ros::Subscriber xian_dj_stewart_platform_error_monitor_sub;
        ros::Subscriber xian_dj_robotgo1_switch_mode_state_sub;
        ros::Subscriber xian_dj_robotgo1_state_display_state_sub;
        ros::Subscriber xian_dj_robotgo1_local_controller_state_sub;
        ros::Subscriber xian_dj_robotgo1_lighter_displayer_state_sub;
        ros::Subscriber xian_dj_robotgo1_control_state_sub;

        ros::Publisher xian_dj_robotgo1_error_monitor_pub;
        std_msgs::String pub_msg;
           
        int xian_dj_car_chassis_error_code = 0;
        int xian_dj_retractable_platform_error_code = 0;
        int xian_dj_stewart_platform_error_code = 0;
        int xian_dj_robotgo1_error_code = 0;
        int xian_dj_tele_op_error_code = 0;

        int xian_dj_robotgo1_local_controller_heart_beat = 0;
        int xian_dj_robotgo1_state_display_heart_beat = 0;
        int xian_dj_robotgo1_switch_mode_heart_beat = 0;
        int xian_dj_robotgo1_control_heart_beat = 0;
        int xian_dj_robotgo1_lighter_displayer_heart_beat = 0;

        int xian_dj_robotgo1_local_controller_heart_beat_cur = 0;
        int xian_dj_robotgo1_local_controller_heart_beat_pre = 0;
        int xian_dj_robotgo1_back_end_heart_beat_cur = 0;
        int xian_dj_robotgo1_back_end_heart_beat_pre = 0;
        int xian_dj_robotgo1_state_display_heart_beat_cur = 0;
        int xian_dj_robotgo1_state_display_heart_beat_pre = 0;
        int xian_dj_robotgo1_switch_mode_heart_beat_cur = 0;
        int xian_dj_robotgo1_switch_mode_heart_beat_pre = 0;
        int xian_dj_robotgo1_control_heart_beat_cur = 0;
        int xian_dj_robotgo1_control_heart_beat_pre = 0;
        int xian_dj_robotgo1_lighter_displayer_heart_beat_cur = 0;
        int xian_dj_robotgo1_lighter_displayer_heart_beat_pre = 0;

        int xian_dj_robotgo1_local_controller_node_restart_flag = 0;
        int xian_dj_robotgo1_state_display_node_restart_flag = 0;
        int xian_dj_robotgo1_switch_mode_node_restart_flag = 0;
        int xian_dj_robotgo1_control_node_restart_flag = 0;
        int xian_dj_robotgo1_lighter_displayer_node_restart_flag = 0;

        int xian_dj_robotgo1_local_controller_error = 0;
        int xian_dj_robotgo1_state_display_error = 0;
        int xian_dj_robotgo1_switch_mode_error = 0;
        int xian_dj_robotgo1_control_error = 0;
        int xian_dj_robotgo1_lighter_displayer_error = 0;

        int time_out = 5;

        int error_sum = 0;
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_error_monitor");
    XianDjRobotgo1ErrorMonitor xian_dj_robotgo1_error_monitor;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_robotgo1_error_monitor.m_timer_control = nh_2.createWallTimer(ros::WallDuration(1), &XianDjRobotgo1ErrorMonitor::m_timer_control_func, &xian_dj_robotgo1_error_monitor);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}