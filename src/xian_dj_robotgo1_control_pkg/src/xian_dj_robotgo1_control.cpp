#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <std_msgs/UInt16.h>
#include "xian_dj_remote_operation_control_pkg/xian_dj_tele_op_controller_server.h"
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_local_controller.h"
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_switch_mode.h"
#include "xian_dj_car_chassis_control_pkg/xian_dj_car_chassis_diff_driver_control.h"
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_state_display.h"
#include"xian_dj_retractable_platform_control_pkg/xian_dj_retractable_platform_control.h"
#include"xian_dj_stewart_platform_control_pkg/xian_dj_stewart_platform_manual_controller.h"

class XianDjRobotgo1Control
{
    public:
        XianDjRobotgo1Control()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            controller_server_sub = nh.subscribe<xian_dj_remote_operation_control_pkg::xian_dj_tele_op_controller_server>("xian_dj_tele_op_controller_server_msg", 10, &XianDjRobotgo1Control::remote_controller_callback, this);
            controller_local_sub = nh.subscribe<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_local_controller>("xian_dj_robotgo1_local_controller_msg", 10, &XianDjRobotgo1Control::local_controller_callback, this);
            switch_mode_sub = nh.subscribe<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_switch_mode>("xian_dj_robotgo1_switch_mode_msg", 10, &XianDjRobotgo1Control::switch_mode_callback, this);
            state_pub = nh.advertise<std_msgs::UInt16>("xian_dj_robotgo1_control_state_msg", 1);
            state_display_pub = nh.advertise<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_state_display>("xian_dj_robotgo1_state_display_msg", 1);
            chassis_diff_drive_control_pub = nh.advertise<xian_dj_car_chassis_control_pkg::xian_dj_car_chassis_diff_driver_control>("xian_dj_car_chassis_diff_driver_control_msg", 1);
            retractable_platform_control_pub = nh.advertise<xian_dj_retractable_platform_control_pkg::xian_dj_retractable_platform_control>("xian_dj_retractable_platform_control_msg", 1);
            stewart_manual_pub = nh.advertise<xian_dj_stewart_platform_control_pkg::xian_dj_stewart_platform_manual_controller>("xian_dj_stewart_platform_manual_controller_msg", 1);
        }


        ros::WallTimer m_timer_control;

        void remote_controller_callback(const xian_dj_remote_operation_control_pkg::xian_dj_tele_op_controller_server::ConstPtr &data)
        {

            xian_dj_tele_op_left_rocker_y_server_cmd = data->xian_dj_tele_op_left_rocker_y_server_cmd;
            xian_dj_tele_op_right_rocker_x_server_cmd = data->xian_dj_tele_op_right_rocker_x_server_cmd;
            xian_dj_tele_op_left_server_cmd = data->xian_dj_tele_op_left_server_cmd;
            xian_dj_tele_op_right_server_cmd = data->xian_dj_tele_op_right_server_cmd;
            xian_dj_tele_op_up_server_cmd = data->xian_dj_tele_op_up_server_cmd;
            xian_dj_tele_op_down_server_cmd = data->xian_dj_tele_op_down_server_cmd;
            xian_dj_tele_op_x_server_cmd = data->xian_dj_tele_op_x_server_cmd;
            xian_dj_tele_op_b_server_cmd = data->xian_dj_tele_op_b_server_cmd;
            xian_dj_tele_op_y_server_cmd = data->xian_dj_tele_op_y_server_cmd;
            xian_dj_tele_op_a_server_cmd = data->xian_dj_tele_op_a_server_cmd;
        }

        void local_controller_callback(const xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_local_controller::ConstPtr &data)
        {
            xian_dj_local_controller_left_rocker_y_cmd = data->xian_dj_local_controller_left_rocker_y_cmd;
            xian_dj_local_controller_right_rocker_x_cmd = data->xian_dj_local_controller_right_rocker_x_cmd;
            xian_dj_local_controller_left_cmd = data->xian_dj_local_controller_left_cmd;
            xian_dj_local_controller_right_cmd = data->xian_dj_local_controller_right_cmd;
            xian_dj_local_controller_up_cmd = data->xian_dj_local_controller_up_cmd;
            xian_dj_local_controller_down_cmd = data->xian_dj_local_controller_down_cmd;
            xian_dj_local_controller_x_cmd = data->xian_dj_local_controller_x_cmd;
            xian_dj_local_controller_b_cmd = data->xian_dj_local_controller_b_cmd;
            xian_dj_local_controller_y_cmd = data->xian_dj_local_controller_y_cmd;
            xian_dj_local_controller_a_cmd = data->xian_dj_local_controller_a_cmd;
        }

        void switch_mode_callback(const xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_switch_mode::ConstPtr &data)
        {
            xian_dj_robotgo1_auto_mode = data->xian_dj_robotgo1_auto_mode;
            xian_dj_robotgo1_tele_op_mode = data->xian_dj_robotgo1_tele_op_mode;
            xian_dj_robotgo1_m_system_mode = data->xian_dj_robotgo1_m_system_mode;
            xian_dj_robotgo1_s_system_mode= data->xian_dj_robotgo1_s_system_mode;
        }


        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            
            xian_dj_robotgo1_control_heart_beat = xian_dj_robotgo1_control_heart_beat > 1000 ? 0 : (xian_dj_robotgo1_control_heart_beat + 1);
            std::cout << "xian_dj_robotgo1_control_heart_beat: " << xian_dj_robotgo1_control_heart_beat << std::endl;
            heart_beat_msg.data = xian_dj_robotgo1_control_heart_beat;
            state_pub.publish(heart_beat_msg);

            
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_error_code", xian_dj_robotgo1_error_code); 
            // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode);
            // ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_controller_client_error", xian_dj_tele_op_controller_client_error);

            if (xian_dj_robotgo1_error_code != 0)
            {
                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 1); 
                state_display_pub_msg.xian_dj_robotgo1_display_mode = 1;
                state_display_pub.publish(state_display_pub_msg);
                std::cout << "排除故障、故障复位"  << std::endl;
            }
            else
            {
                if (xian_dj_tele_op_controller_client_error == 0) //0表示远程控制在线
                {
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 6); 
                    state_display_pub_msg.xian_dj_robotgo1_display_mode = 6;
                    state_display_pub.publish(state_display_pub_msg);
                }
                else
                {
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 5); 
                    state_display_pub_msg.xian_dj_robotgo1_display_mode = 5;
                    state_display_pub.publish(state_display_pub_msg);
                }

                if (xian_dj_robotgo1_auto_mode !=0 ) //自动模式
                {
                    // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 2); 
                    state_display_pub_msg.xian_dj_robotgo1_display_mode = 2;
                    state_display_pub.publish(state_display_pub_msg);
                    return;
                }
                else
                {
                    // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode); 
                    // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);
                    // if (xian_dj_robotgo1_m_system_mode == 0)
                    // {
                    //     std::cout << "挂空，模式切换按钮作用，其余操作无效"  << std::endl;
                    //     return;
                    // }
                    
                    if (xian_dj_robotgo1_m_system_mode == 0) //控制车子运动
                    {
                        if (xian_dj_robotgo1_tele_op_mode ==0) //本地模式
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 
                            state_display_pub_msg.xian_dj_robotgo1_display_mode = 4;
                            state_display_pub.publish(state_display_pub_msg);
                    
                            // ros::param::set("/xian_dj_car_chassis_params_server/input_velocity_cmd", xian_dj_local_controller_left_rocker_y_cmd * 1000);
                            // ros::param::set("/xian_dj_car_chassis_params_server/input_theta_cmd", xian_dj_local_controller_right_rocker_x_cmd * 1000);
                            chassis_diff_drive_control_pub_msg.input_velocity_cmd = xian_dj_local_controller_left_rocker_y_cmd * 1000;
                            chassis_diff_drive_control_pub_msg.input_theta_cmd = xian_dj_local_controller_right_rocker_x_cmd * 1000;
                            chassis_diff_drive_control_pub.publish(chassis_diff_drive_control_pub_msg);

                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 
                            state_display_pub_msg.xian_dj_robotgo1_display_mode = 3;
                            state_display_pub.publish(state_display_pub_msg);
                        
                            // ros::param::set("/xian_dj_car_chassis_params_server/input_velocity_cmd", xian_dj_tele_op_left_rocker_y_server_cmd * 1000);
                            // ros::param::set("/xian_dj_car_chassis_params_server/input_theta_cmd", xian_dj_tele_op_right_rocker_x_server_cmd * 1000);
                            chassis_diff_drive_control_pub_msg.input_velocity_cmd = xian_dj_tele_op_left_rocker_y_server_cmd * 1000;
                            chassis_diff_drive_control_pub_msg.input_theta_cmd = xian_dj_tele_op_right_rocker_x_server_cmd * 1000;
                            chassis_diff_drive_control_pub.publish(chassis_diff_drive_control_pub_msg);
                        }
                        return;
                    }

                    if (xian_dj_robotgo1_m_system_mode == 1) //控制伸缩机构
                    {
                        if (xian_dj_robotgo1_tele_op_mode ==0) //本地模式
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 
                            state_display_pub_msg.xian_dj_robotgo1_display_mode = 4;
                            state_display_pub.publish(state_display_pub_msg);

                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_stand_cmd", xian_dj_local_controller_x_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_sit_cmd", xian_dj_local_controller_b_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_up_cmd", xian_dj_local_controller_up_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_down_cmd", xian_dj_local_controller_down_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_up_cmd", xian_dj_local_controller_y_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_down_cmd", xian_dj_local_controller_a_cmd);
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_stand_linear_actuator_stand_cmd = xian_dj_local_controller_x_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_stand_linear_actuator_sit_cmd = xian_dj_local_controller_b_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_first_linear_actuator_up_cmd = xian_dj_local_controller_up_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_first_linear_actuator_down_cmd = xian_dj_local_controller_down_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_second_linear_actuator_up_cmd = xian_dj_local_controller_y_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_second_linear_actuator_down_cmd = xian_dj_local_controller_a_cmd;
                            retractable_platform_control_pub.publish(retractable_platform_control_pub_msg);
                        }
                        else
                        {
                            // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 
                            state_display_pub_msg.xian_dj_robotgo1_display_mode = 3;
                            state_display_pub.publish(state_display_pub_msg);

                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_stand_cmd", xian_dj_tele_op_x_server_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_sit_cmd", xian_dj_tele_op_b_server_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_up_cmd", xian_dj_tele_op_up_server_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_down_cmd", xian_dj_tele_op_down_server_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_up_cmd", xian_dj_tele_op_y_server_cmd);
                            // ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_down_cmd", xian_dj_tele_op_a_server_cmd);
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_stand_linear_actuator_stand_cmd = xian_dj_tele_op_x_server_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_stand_linear_actuator_sit_cmd = xian_dj_tele_op_b_server_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_first_linear_actuator_up_cmd = xian_dj_tele_op_up_server_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_first_linear_actuator_down_cmd = xian_dj_tele_op_down_server_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_second_linear_actuator_up_cmd = xian_dj_tele_op_y_server_cmd;
                            retractable_platform_control_pub_msg.xian_dj_retractable_platform_second_linear_actuator_down_cmd = xian_dj_tele_op_a_server_cmd;
                            retractable_platform_control_pub.publish(retractable_platform_control_pub_msg);
                        }
                        return;
                    }
                    
                    if (xian_dj_robotgo1_m_system_mode == 2) //控制六自由度平台
                    {
                        if(xian_dj_robotgo1_s_system_mode == 0)//控制六自由度平台平移
                        {
                            if (xian_dj_robotgo1_tele_op_mode ==0)//本地模式
                            {
                                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 
                                state_display_pub_msg.xian_dj_robotgo1_display_mode = 4;
                                state_display_pub.publish(state_display_pub_msg);


                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_positive_cmd", xian_dj_local_controller_up_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_negative_cmd", xian_dj_local_controller_down_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_positive_cmd", xian_dj_local_controller_right_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_negative_cmd", xian_dj_local_controller_left_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_positive_cmd", xian_dj_local_controller_y_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_negative_cmd", xian_dj_local_controller_a_cmd);
                                
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_x_positive_cmd = xian_dj_local_controller_up_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_x_negative_cmd = xian_dj_local_controller_down_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_y_positive_cmd = xian_dj_local_controller_right_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_y_negative_cmd = xian_dj_local_controller_left_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_z_positive_cmd = xian_dj_local_controller_y_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_z_negative_cmd = xian_dj_local_controller_a_cmd;
                                stewart_manual_pub.publish(stewart_manual_pub_msg);
                            }
                            else 
                            {
                                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 
                                state_display_pub_msg.xian_dj_robotgo1_display_mode = 3;
                                state_display_pub.publish(state_display_pub_msg);

                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_positive_cmd", xian_dj_tele_op_up_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_negative_cmd", xian_dj_tele_op_down_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_positive_cmd", xian_dj_tele_op_right_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_negative_cmd", xian_dj_tele_op_left_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_positive_cmd", xian_dj_tele_op_y_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_negative_cmd", xian_dj_tele_op_a_server_cmd);
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_x_positive_cmd = xian_dj_tele_op_up_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_x_negative_cmd = xian_dj_tele_op_down_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_y_positive_cmd = xian_dj_tele_op_right_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_y_negative_cmd = xian_dj_tele_op_left_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_z_positive_cmd = xian_dj_tele_op_y_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_z_negative_cmd = xian_dj_tele_op_a_server_cmd;
                                stewart_manual_pub.publish(stewart_manual_pub_msg);
                            }
                        }
                        if(xian_dj_robotgo1_s_system_mode == 1)//控制六自由度平台旋转
                        {
                            if (xian_dj_robotgo1_tele_op_mode ==0) //本地模式
                            {
                                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 
                                state_display_pub_msg.xian_dj_robotgo1_display_mode = 4;
                                state_display_pub.publish(state_display_pub_msg);

                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_positive_cmd", xian_dj_local_controller_up_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_negative_cmd", xian_dj_local_controller_down_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_positive_cmd", xian_dj_local_controller_right_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_negative_cmd", xian_dj_local_controller_left_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_positive_cmd", xian_dj_local_controller_b_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_negative_cmd", xian_dj_local_controller_x_cmd);
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_alpha_positive_cmd = xian_dj_local_controller_up_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_alpha_negative_cmd = xian_dj_local_controller_down_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_beta_positive_cmd = xian_dj_local_controller_right_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_beta_negative_cmd = xian_dj_local_controller_left_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_gamma_positive_cmd = xian_dj_local_controller_b_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_gamma_negative_cmd = xian_dj_local_controller_x_cmd;
                                stewart_manual_pub.publish(stewart_manual_pub_msg);
                            }
                            else
                            {
                                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 
                                state_display_pub_msg.xian_dj_robotgo1_display_mode = 3;
                                state_display_pub.publish(state_display_pub_msg);

                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_positive_cmd", xian_dj_tele_op_up_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_negative_cmd", xian_dj_tele_op_down_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_positive_cmd", xian_dj_tele_op_right_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_negative_cmd", xian_dj_tele_op_left_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_positive_cmd", xian_dj_tele_op_b_server_cmd);
                                // ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_negative_cmd", xian_dj_tele_op_x_server_cmd);
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_alpha_positive_cmd = xian_dj_tele_op_up_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_alpha_negative_cmd = xian_dj_tele_op_down_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_beta_positive_cmd = xian_dj_tele_op_right_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_beta_negative_cmd = xian_dj_tele_op_left_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_gamma_positive_cmd = xian_dj_tele_op_b_server_cmd;
                                stewart_manual_pub_msg.xian_dj_stewart_platform_input_gamma_negative_cmd = xian_dj_tele_op_x_server_cmd;
                                stewart_manual_pub.publish(stewart_manual_pub_msg);
                            }
                        }
                        return;
                    }
                }
            }

        }
        
 
    private:
        ros::Subscriber controller_server_sub;
        ros::Subscriber controller_local_sub;
        ros::Subscriber switch_mode_sub;

        ros::Publisher state_pub;
        ros::Publisher retractable_platform_control_pub;
        ros::Publisher state_display_pub;
        ros::Publisher chassis_diff_drive_control_pub;
        ros::Publisher stewart_manual_pub;

        xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_state_display state_display_pub_msg;
        xian_dj_car_chassis_control_pkg::xian_dj_car_chassis_diff_driver_control chassis_diff_drive_control_pub_msg;
        xian_dj_retractable_platform_control_pkg::xian_dj_retractable_platform_control retractable_platform_control_pub_msg;
        std_msgs::UInt16 heart_beat_msg;
        xian_dj_stewart_platform_control_pkg::xian_dj_stewart_platform_manual_controller stewart_manual_pub_msg;

        int xian_dj_robotgo1_control_heart_beat = 0;

        int xian_dj_robotgo1_auto_mode = 0;
        int xian_dj_robotgo1_m_system_mode = 0;
        int xian_dj_robotgo1_s_system_mode = 0;
        int xian_dj_robotgo1_display_mode = 0;
        int xian_dj_robotgo1_error_code = 0;
        int xian_dj_robotgo1_tele_op_mode = 0;
        int xian_dj_tele_op_controller_client_error = 0;

        // 本地遥控器按键变量
        double xian_dj_local_controller_left_rocker_y_cmd = 0;
        double xian_dj_local_controller_right_rocker_x_cmd = 0;
        int xian_dj_local_controller_left_cmd = 0;
        int xian_dj_local_controller_right_cmd = 0;
        int xian_dj_local_controller_up_cmd = 0;
        int xian_dj_local_controller_down_cmd =0;
        int xian_dj_local_controller_x_cmd = 0;
        int xian_dj_local_controller_b_cmd = 0;
        int xian_dj_local_controller_y_cmd = 0;
        int xian_dj_local_controller_a_cmd = 0;

        // 远控遥控器变量
        double xian_dj_tele_op_left_rocker_y_server_cmd = 0;
        double xian_dj_tele_op_right_rocker_x_server_cmd = 0;
        int xian_dj_tele_op_left_server_cmd = 0;
        int xian_dj_tele_op_right_server_cmd = 0;
        int xian_dj_tele_op_up_server_cmd = 0;
        int xian_dj_tele_op_down_server_cmd =0;
        int xian_dj_tele_op_x_server_cmd = 0;
        int xian_dj_tele_op_b_server_cmd = 0;
        int xian_dj_tele_op_y_server_cmd = 0;
        int xian_dj_tele_op_a_server_cmd = 0;
};
    
int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_control");
    XianDjRobotgo1Control xian_dj_robotgo1_control;

    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", 0);
    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", 0); 
    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", 0);
    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 0);
    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // xian_dj_robotgo1_control.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1Control::m_timer_heart_beat_func, &xian_dj_robotgo1_control);
    xian_dj_robotgo1_control.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.1), &XianDjRobotgo1Control::m_timer_control_func, &xian_dj_robotgo1_control);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}