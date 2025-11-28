
#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <std_msgs/UInt16.h>
#include "xian_dj_remote_operation_control_pkg/xian_dj_tele_op_controller_server.h"
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_local_controller.h"
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_switch_mode.h"

class XianDjRobotgo1SwitchMode
{
    public:
        XianDjRobotgo1SwitchMode()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            controller_server_sub = nh.subscribe<xian_dj_remote_operation_control_pkg::xian_dj_tele_op_controller_server>("xian_dj_tele_op_controller_server_msg", 10, &XianDjRobotgo1SwitchMode::remote_controller_callback, this);
            controller_local_sub = nh.subscribe<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_local_controller>("xian_dj_robotgo1_local_controller_msg", 10, &XianDjRobotgo1SwitchMode::local_controller_callback, this);
            xian_dj_robotgo1_switch_mode_state_pub = nh.advertise<std_msgs::UInt16>("xian_dj_robotgo1_switch_mode_state_msg", 1);
            xian_dj_robotgo1_switch_mode_pub = nh.advertise<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_switch_mode>("xian_dj_robotgo1_switch_mode_msg", 1); 
        }

        
        ros::WallTimer m_timer_control;

        void remote_controller_callback(const xian_dj_remote_operation_control_pkg::xian_dj_tele_op_controller_server::ConstPtr &data)
        {
            xian_dj_tele_op_r1_server_cmd = data->xian_dj_tele_op_r1_server_cmd;
            xian_dj_tele_op_r2_server_cmd = data->xian_dj_tele_op_r2_server_cmd;
            xian_dj_tele_op_l1_server_cmd = data->xian_dj_tele_op_l1_server_cmd;
            xian_dj_tele_op_l2_server_cmd = data->xian_dj_tele_op_l2_server_cmd;
        }

        void local_controller_callback(const xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_local_controller::ConstPtr &data)
        {
            xian_dj_local_controller_r1_cmd = data->xian_dj_local_controller_r1_cmd; 
            xian_dj_local_controller_r2_cmd = data->xian_dj_local_controller_r2_cmd; 
            xian_dj_local_controller_l1_cmd = data->xian_dj_local_controller_l1_cmd; 
            xian_dj_local_controller_l2_cmd = data->xian_dj_local_controller_l2_cmd;
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {   
            xian_dj_robotgo1_switch_mode_heart_beat = xian_dj_robotgo1_switch_mode_heart_beat > 1000 ? 0 : (xian_dj_robotgo1_switch_mode_heart_beat + 1);
            std::cout << "xian_dj_robotgo1_switch_mode_heart_beat: " << xian_dj_robotgo1_switch_mode_heart_beat << std::endl;
            heart_beat_msg.data = xian_dj_robotgo1_switch_mode_heart_beat;
            xian_dj_robotgo1_switch_mode_state_pub.publish(heart_beat_msg);

            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);

            // ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_r1_server_cmd", xian_dj_tele_op_r1_server_cmd); 
            // ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_r2_server_cmd", xian_dj_tele_op_r2_server_cmd); 
            // ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_l1_server_cmd", xian_dj_tele_op_l1_server_cmd); 
            // ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_l2_server_cmd", xian_dj_tele_op_l2_server_cmd); 

            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r1_cmd", xian_dj_local_controller_r1_cmd); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r2_cmd", xian_dj_local_controller_r2_cmd); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l1_cmd", xian_dj_local_controller_l1_cmd); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l2_cmd", xian_dj_local_controller_l2_cmd); 

            xian_dj_tele_op_r2_server_cmd_pre = xian_dj_tele_op_r2_server_cmd_cur;
            xian_dj_tele_op_r2_server_cmd_cur = xian_dj_tele_op_r2_server_cmd;
            xian_dj_local_controller_r2_cmd_pre = xian_dj_local_controller_r2_cmd_cur;
            xian_dj_local_controller_r2_cmd_cur = xian_dj_local_controller_r2_cmd;
            if ((xian_dj_tele_op_r2_server_cmd_pre ==0  && xian_dj_tele_op_r2_server_cmd_cur ==1) || (xian_dj_local_controller_r2_cmd_pre ==0 && xian_dj_local_controller_r2_cmd_cur==1))
            {
              
                xian_dj_robotgo1_tele_op_mode++;
                if(xian_dj_robotgo1_tele_op_mode > 1)
                {
                    xian_dj_robotgo1_tele_op_mode = 0;
                } 
                 
                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode);
                pub_msg.xian_dj_robotgo1_tele_op_mode = xian_dj_robotgo1_tele_op_mode;
            }
            
            // 远程遥控模式：
            if (xian_dj_robotgo1_tele_op_mode == 1)
            {

                xian_dj_tele_op_l2_server_cmd_pre = xian_dj_tele_op_l2_server_cmd_cur;
                xian_dj_tele_op_l2_server_cmd_cur = xian_dj_tele_op_l2_server_cmd;
                if (xian_dj_tele_op_l2_server_cmd_pre ==0  && xian_dj_tele_op_l2_server_cmd_cur ==1)
                {
                
                    xian_dj_robotgo1_auto_mode++;
                    if(xian_dj_robotgo1_auto_mode > 1)
                    {
                        xian_dj_robotgo1_auto_mode = 0;
                    } 
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode);
                    pub_msg.xian_dj_robotgo1_auto_mode = xian_dj_robotgo1_auto_mode;
                }
                
                // 主模式切换
                xian_dj_tele_op_l1_server_cmd_pre = xian_dj_tele_op_l1_server_cmd_cur;
                xian_dj_tele_op_l1_server_cmd_cur = xian_dj_tele_op_l1_server_cmd;
                if (xian_dj_tele_op_l1_server_cmd_pre ==0  && xian_dj_tele_op_l1_server_cmd_cur ==1)
                {
                    xian_dj_robotgo1_m_system_mode++;
                    if(xian_dj_robotgo1_m_system_mode > 2)
                    {
                        xian_dj_robotgo1_m_system_mode = 0;
                    } 
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode);
                    pub_msg.xian_dj_robotgo1_m_system_mode=xian_dj_robotgo1_m_system_mode;
                }
  
                xian_dj_tele_op_r1_server_cmd_pre = xian_dj_tele_op_r1_server_cmd_cur;
                xian_dj_tele_op_r1_server_cmd_cur = xian_dj_tele_op_r1_server_cmd;
                if (xian_dj_tele_op_r1_server_cmd_pre ==0  && xian_dj_tele_op_r1_server_cmd_cur ==1)
                {
                    if (xian_dj_robotgo1_m_system_mode == 2) 
                    {
                        xian_dj_robotgo1_s_system_mode++;
                        if(xian_dj_robotgo1_s_system_mode > 1)
                        {
                            xian_dj_robotgo1_s_system_mode = 0;
                        }
                    }  
                    else
                    {
                        xian_dj_robotgo1_s_system_mode = 0;
                    }
                             
                }
                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);
                pub_msg.xian_dj_robotgo1_s_system_mode = xian_dj_robotgo1_s_system_mode;
            }
            else
            {
                xian_dj_local_controller_l2_cmd_pre = xian_dj_local_controller_l2_cmd_cur;
                xian_dj_local_controller_l2_cmd_cur = xian_dj_local_controller_l2_cmd;
                if (xian_dj_local_controller_l2_cmd_pre ==0  && xian_dj_local_controller_l2_cmd_cur ==1)
                {
                
                    xian_dj_robotgo1_auto_mode++;
                    if(xian_dj_robotgo1_auto_mode > 1)
                    {
                        xian_dj_robotgo1_auto_mode = 0;
                    } 
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode);
                    pub_msg.xian_dj_robotgo1_auto_mode = xian_dj_robotgo1_auto_mode;
                }
                
                xian_dj_local_controller_l1_cmd_pre = xian_dj_local_controller_l1_cmd_cur;
                xian_dj_local_controller_l1_cmd_cur = xian_dj_local_controller_l1_cmd;
                if (xian_dj_local_controller_l1_cmd_pre ==0 && xian_dj_local_controller_l1_cmd_cur==1)
                {
                    xian_dj_robotgo1_m_system_mode++;
                    if(xian_dj_robotgo1_m_system_mode > 2)
                    {
                        xian_dj_robotgo1_m_system_mode = 0;
                    } 
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode);
                    pub_msg.xian_dj_robotgo1_m_system_mode = xian_dj_robotgo1_m_system_mode;
                }
  
                xian_dj_local_controller_r1_cmd_pre = xian_dj_local_controller_r1_cmd_cur;
                xian_dj_local_controller_r1_cmd_cur = xian_dj_local_controller_r1_cmd;
                if (xian_dj_local_controller_r1_cmd_pre ==0 && xian_dj_local_controller_r1_cmd_cur==1)
                {
                    if (xian_dj_robotgo1_m_system_mode == 2) 
                    {
                        xian_dj_robotgo1_s_system_mode++;
                        if(xian_dj_robotgo1_s_system_mode > 1)
                        {
                            xian_dj_robotgo1_s_system_mode = 0;
                        }
                    }  
                    else
                    {
                        xian_dj_robotgo1_s_system_mode = 0;
                    }
                             
                }
                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);
                pub_msg.xian_dj_robotgo1_s_system_mode = xian_dj_robotgo1_s_system_mode;
            }   
            // std::cout << "xian_dj_robotgo1_tele_op_mode: " << xian_dj_robotgo1_tele_op_mode << std::endl;
            // std::cout << "xian_dj_robotgo1_auto_mode: " << xian_dj_robotgo1_auto_mode << std::endl;
            // std::cout << "xian_dj_robotgo1_m_system_mode: " << xian_dj_robotgo1_m_system_mode << std::endl;
            // std::cout << "xian_dj_robotgo1_s_system_mode: " << xian_dj_robotgo1_s_system_mode << std::endl;
            xian_dj_robotgo1_switch_mode_pub.publish(pub_msg);
        }


    private:
        ros::Subscriber controller_server_sub;
        ros::Subscriber controller_local_sub;
        ros::Subscriber switch_mode_sub;
        
        ros::Publisher xian_dj_robotgo1_switch_mode_state_pub;
        ros::Publisher xian_dj_robotgo1_switch_mode_pub;
        xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_switch_mode pub_msg;
        std_msgs::UInt16 heart_beat_msg;

        int xian_dj_robotgo1_switch_mode_heart_beat = 0;
        // 模式变量
        int xian_dj_robotgo1_auto_mode = 0;
        int xian_dj_robotgo1_tele_op_mode = 0;
        int xian_dj_robotgo1_m_system_mode = 0;
        int xian_dj_robotgo1_s_system_mode = 0;

        int xian_dj_tele_op_r1_server_cmd = 0;
        int xian_dj_tele_op_r2_server_cmd = 0;
        int xian_dj_tele_op_l1_server_cmd = 0;
        int xian_dj_tele_op_l2_server_cmd = 0;

        int xian_dj_local_controller_r1_cmd = 0;
        int xian_dj_local_controller_r2_cmd = 0;
        int xian_dj_local_controller_l1_cmd = 0;
        int xian_dj_local_controller_l2_cmd = 0;

        // 中间变量
        int xian_dj_tele_op_r1_server_cmd_pre = 0;
        int xian_dj_tele_op_r2_server_cmd_pre = 0;
        int xian_dj_tele_op_l1_server_cmd_pre = 0;
        int xian_dj_tele_op_l2_server_cmd_pre = 0;

        int xian_dj_tele_op_r1_server_cmd_cur = 0;
        int xian_dj_tele_op_r2_server_cmd_cur = 0;
        int xian_dj_tele_op_l1_server_cmd_cur = 0;
        int xian_dj_tele_op_l2_server_cmd_cur = 0;

        int xian_dj_local_controller_r1_cmd_pre = 0;
        int xian_dj_local_controller_r2_cmd_pre = 0;
        int xian_dj_local_controller_l1_cmd_pre = 0;
        int xian_dj_local_controller_l2_cmd_pre = 0;

        int xian_dj_local_controller_r1_cmd_cur = 0;
        int xian_dj_local_controller_r2_cmd_cur = 0;
        int xian_dj_local_controller_l1_cmd_cur = 0;
        int xian_dj_local_controller_l2_cmd_cur = 0;
        
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_switch_mode");
    XianDjRobotgo1SwitchMode xian_dj_robotgo1_switch_mode;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // xian_dj_robotgo1_switch_mode.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1SwitchMode::m_timer_heart_beat_func, &xian_dj_robotgo1_switch_mode);
    xian_dj_robotgo1_switch_mode.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.1), &XianDjRobotgo1SwitchMode::m_timer_control_func, &xian_dj_robotgo1_switch_mode);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}
