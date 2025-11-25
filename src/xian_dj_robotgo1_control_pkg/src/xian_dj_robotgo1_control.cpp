#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>

class XianDjRobotgo1Control
{
    public:
        XianDjRobotgo1Control()
        {
            // 创建一个ROS节点句柄
            // ros::NodeHandle nh;
        }

        // ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        // void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        // {
        //     ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", xian_dj_robotgo1_control_heart_beat); 
        //     std::cout << "xian_dj_robotgo1_control_heart_beat: " << xian_dj_robotgo1_control_heart_beat << std::endl;
        //     counter = counter > 1000 ? 0 : (counter + 1);
        //     ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", counter);  // 自行替换
        // }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", xian_dj_robotgo1_control_heart_beat); 
            std::cout << "xian_dj_robotgo1_control_heart_beat: " << xian_dj_robotgo1_control_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", counter);  // 自行替换

            
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_error_code", xian_dj_robotgo1_error_code); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode);
            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_controller_client_error", xian_dj_tele_op_controller_client_error);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);
            std::cout << "xian_dj_robotgo1_tele_op_mode: " << xian_dj_robotgo1_tele_op_mode << std::endl;
            std::cout << "xian_dj_robotgo1_auto_mode: " << xian_dj_robotgo1_auto_mode << std::endl;
            std::cout << "xian_dj_robotgo1_m_system_mode: " << xian_dj_robotgo1_m_system_mode << std::endl;
            std::cout << "xian_dj_robotgo1_s_system_mode: " << xian_dj_robotgo1_s_system_mode << std::endl;

            if (xian_dj_robotgo1_error_code != 0)
            {
                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 1); 
                std::cout << "排除故障、故障复位"  << std::endl;
            }
            else
            {
                if (xian_dj_tele_op_controller_client_error == 0) //0表示远程控制在线
                {
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 6); 
                }
                else
                {
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 5); 
                }
                // ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 0); 
                // ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode);
                if (xian_dj_robotgo1_auto_mode !=0 ) //自动模式
                {
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 2); 
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
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_rocker_y_cmd", xian_dj_local_controller_left_rocker_y_cmd);
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_rocker_x_cmd", xian_dj_local_controller_right_rocker_x_cmd);
                            ros::param::set("/xian_dj_car_chassis_params_server/input_velocity_cmd", xian_dj_local_controller_left_rocker_y_cmd * 1000);
                            ros::param::set("/xian_dj_car_chassis_params_server/input_theta_cmd", xian_dj_local_controller_right_rocker_x_cmd * 1000);
                        }
                        else
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_left_rocker_y_server_cmd", xian_dj_tele_op_left_rocker_y_server_cmd);
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_right_rocker_x_server_cmd", xian_dj_tele_op_right_rocker_x_server_cmd);
                            ros::param::set("/xian_dj_car_chassis_params_server/input_velocity_cmd", xian_dj_tele_op_left_rocker_y_server_cmd * 1000);
                            ros::param::set("/xian_dj_car_chassis_params_server/input_theta_cmd", xian_dj_tele_op_right_rocker_x_server_cmd * 1000);
                        }
                        return;
                    }

                    if (xian_dj_robotgo1_m_system_mode == 1) //控制伸缩机构
                    {
                        if (xian_dj_robotgo1_tele_op_mode ==0) //本地模式
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 

                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_x_cmd", xian_dj_local_controller_x_cmd);
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_b_cmd", xian_dj_local_controller_b_cmd);
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_up_cmd", xian_dj_local_controller_up_cmd);
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_down_cmd", xian_dj_local_controller_down_cmd);
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_y_cmd", xian_dj_local_controller_y_cmd);
                            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_a_cmd", xian_dj_local_controller_a_cmd);

                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_stand_cmd", xian_dj_local_controller_x_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_sit_cmd", xian_dj_local_controller_b_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_up_cmd", xian_dj_local_controller_up_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_down_cmd", xian_dj_local_controller_down_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_up_cmd", xian_dj_local_controller_y_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_down_cmd", xian_dj_local_controller_a_cmd);
                        }
                        else
                        {
                            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 

                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_x_server_cmd", xian_dj_tele_op_x_server_cmd);
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_b_server_cmd", xian_dj_tele_op_b_server_cmd);
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_up_server_cmd", xian_dj_tele_op_up_server_cmd);
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_down_server_cmd", xian_dj_tele_op_down_server_cmd);
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_y_server_cmd", xian_dj_tele_op_y_server_cmd);
                            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_a_server_cmd", xian_dj_tele_op_a_server_cmd);

                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_stand_cmd", xian_dj_tele_op_x_server_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_sit_cmd", xian_dj_tele_op_b_server_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_up_cmd", xian_dj_tele_op_up_server_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_down_cmd", xian_dj_tele_op_down_server_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_up_cmd", xian_dj_tele_op_y_server_cmd);
                            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_down_cmd", xian_dj_tele_op_a_server_cmd);
                        }
                        return;
                    }
                    
                    if (xian_dj_robotgo1_m_system_mode == 2) //控制六自由度平台
                    {
                        if(xian_dj_robotgo1_s_system_mode == 0)//控制六自由度平台平移
                        {
                            if (xian_dj_robotgo1_tele_op_mode ==0)//本地模式
                            {
                                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 

                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_up_cmd", xian_dj_local_controller_up_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_down_cmd", xian_dj_local_controller_down_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_cmd", xian_dj_local_controller_left_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_cmd", xian_dj_local_controller_right_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_y_cmd", xian_dj_local_controller_y_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_a_cmd", xian_dj_local_controller_a_cmd);

                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_positive_cmd", xian_dj_local_controller_up_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_negative_cmd", xian_dj_local_controller_down_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_positive_cmd", xian_dj_local_controller_right_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_negative_cmd", xian_dj_local_controller_left_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_positive_cmd", xian_dj_local_controller_y_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_negative_cmd", xian_dj_local_controller_a_cmd);
                            }
                            else 
                            {
                                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 

                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_up_server_cmd", xian_dj_tele_op_up_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_down_server_cmd", xian_dj_tele_op_down_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_left_server_cmd", xian_dj_tele_op_left_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_right_server_cmd", xian_dj_tele_op_right_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_y_server_cmd", xian_dj_tele_op_y_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_a_server_cmd", xian_dj_tele_op_a_server_cmd);

                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_positive_cmd", xian_dj_tele_op_up_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_negative_cmd", xian_dj_tele_op_down_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_positive_cmd", xian_dj_tele_op_right_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_negative_cmd", xian_dj_tele_op_left_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_positive_cmd", xian_dj_tele_op_y_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_negative_cmd", xian_dj_tele_op_a_server_cmd);
                            }
                        }
                        if(xian_dj_robotgo1_s_system_mode == 1)//控制六自由度平台旋转
                        {
                            if (xian_dj_robotgo1_tele_op_mode ==0) //本地模式
                            {
                                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 4); 

                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_up_cmd", xian_dj_local_controller_up_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_down_cmd", xian_dj_local_controller_down_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_cmd", xian_dj_local_controller_left_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_cmd", xian_dj_local_controller_right_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_b_cmd", xian_dj_local_controller_b_cmd);
                                ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_x_cmd", xian_dj_local_controller_x_cmd);

                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_positive_cmd", xian_dj_local_controller_up_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_negative_cmd", xian_dj_local_controller_down_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_positive_cmd", xian_dj_local_controller_right_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_negative_cmd", xian_dj_local_controller_left_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_positive_cmd", xian_dj_local_controller_b_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_negative_cmd", xian_dj_local_controller_x_cmd);
                            }
                            else
                            {
                                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", 3); 

                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_up_server_cmd", xian_dj_tele_op_up_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_down_server_cmd", xian_dj_tele_op_down_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_left_server_cmd", xian_dj_tele_op_left_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_right_server_cmd", xian_dj_tele_op_right_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_b_server_cmd", xian_dj_tele_op_b_server_cmd);
                                ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_x_server_cmd", xian_dj_tele_op_x_server_cmd);

                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_positive_cmd", xian_dj_tele_op_up_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_negative_cmd", xian_dj_tele_op_down_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_positive_cmd", xian_dj_tele_op_right_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_negative_cmd", xian_dj_tele_op_left_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_positive_cmd", xian_dj_tele_op_b_server_cmd);
                                ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_negative_cmd", xian_dj_tele_op_x_server_cmd);
                            }
                        }
                        return;
                    }
                }
            }

        }
        
 
    private:
        int counter = 0;
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
    xian_dj_robotgo1_control.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.3), &XianDjRobotgo1Control::m_timer_control_func, &xian_dj_robotgo1_control);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}