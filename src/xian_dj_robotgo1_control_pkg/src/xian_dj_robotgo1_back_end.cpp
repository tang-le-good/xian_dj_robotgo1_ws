#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include "xian_dj_robotgo1_control_pkg/xian_dj_robotgo1_back_end.h"

class XianDjRobotgo1BackEnd
{
    public:
        XianDjRobotgo1BackEnd()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            pub = nh.advertise<xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_back_end>("xian_dj_robotgo1_back_end_msg", 1);
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_back_end_heart_beat", xian_dj_robotgo1_back_end_heart_beat); 
            std::cout << "xian_dj_robotgo1_back_end_heart_beat: " << xian_dj_robotgo1_back_end_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_back_end_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_params_server_heart_beat", xian_dj_robotgo1_params_server_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_error_monitor_heart_beat", xian_dj_robotgo1_error_monitor_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_back_end_heart_beat", xian_dj_robotgo1_back_end_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_state_display_heart_beat", xian_dj_robotgo1_state_display_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_switch_mode_heart_beat", xian_dj_robotgo1_switch_mode_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", xian_dj_robotgo1_control_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat", xian_dj_robotgo1_lighter_displayer_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_heart_beat", xian_dj_robotgo1_tele_op_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_local_controller_heart_beat", xian_dj_robotgo1_local_controller_heart_beat);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level_heart_beat", xian_dj_robotgo1_battery_level_heart_beat);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_battery_level", xian_dj_robotgo1_battery_level);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_display_mode", xian_dj_robotgo1_display_mode);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd", xian_dj_robotgo1_red_ligher_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd", xian_dj_robotgo1_green_ligher_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd", xian_dj_robotgo1_yellow_ligher_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd", xian_dj_robotgo1_displayer_cmd);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_cmd", xian_dj_local_controller_left_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_cmd", xian_dj_local_controller_right_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_up_cmd", xian_dj_local_controller_up_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_down_cmd", xian_dj_local_controller_down_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_x_cmd", xian_dj_local_controller_x_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_b_cmd", xian_dj_local_controller_b_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_y_cmd", xian_dj_local_controller_y_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_a_cmd", xian_dj_local_controller_a_cmd);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_rocker_x_cmd", xian_dj_local_controller_left_rocker_x_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_rocker_y_cmd", xian_dj_local_controller_left_rocker_y_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_rocker_x_cmd", xian_dj_local_controller_right_rocker_x_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_rocker_y_cmd", xian_dj_local_controller_right_rocker_y_cmd);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r1_cmd", xian_dj_local_controller_r1_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r2_cmd", xian_dj_local_controller_r2_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l1_cmd", xian_dj_local_controller_l1_cmd);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l2_cmd", xian_dj_local_controller_l2_cmd);

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_params_server_error", xian_dj_robotgo1_params_server_error);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_state_display_error", xian_dj_robotgo1_state_display_error);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_switch_mode_error", xian_dj_robotgo1_switch_mode_error);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_back_end_error", xian_dj_robotgo1_back_end_error);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_error", xian_dj_robotgo1_control_error);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_error", xian_dj_robotgo1_lighter_displayer_error);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_error_code", xian_dj_robotgo1_error_code);
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_heat_beat_error_code", xian_dj_robotgo1_heat_beat_error_code);

            web_show.xian_dj_robotgo1_params_server_heart_beat = xian_dj_robotgo1_params_server_heart_beat;
            web_show.xian_dj_robotgo1_error_monitor_heart_beat = xian_dj_robotgo1_error_monitor_heart_beat;
            web_show.xian_dj_robotgo1_back_end_heart_beat = xian_dj_robotgo1_back_end_heart_beat;
            web_show.xian_dj_robotgo1_state_display_heart_beat = xian_dj_robotgo1_state_display_heart_beat;
            web_show.xian_dj_robotgo1_switch_mode_heart_beat = xian_dj_robotgo1_switch_mode_heart_beat;
            web_show.xian_dj_robotgo1_control_heart_beat = xian_dj_robotgo1_control_heart_beat;
            web_show.xian_dj_robotgo1_lighter_displayer_heart_beat = xian_dj_robotgo1_lighter_displayer_heart_beat;
            web_show.xian_dj_robotgo1_tele_op_heart_beat = xian_dj_robotgo1_tele_op_heart_beat;
            web_show.xian_dj_robotgo1_local_controller_heart_beat = xian_dj_robotgo1_local_controller_heart_beat;
            web_show.xian_dj_robotgo1_battery_level_heart_beat = xian_dj_robotgo1_battery_level_heart_beat;

            web_show.xian_dj_robotgo1_auto_mode = xian_dj_robotgo1_auto_mode;
            web_show.xian_dj_robotgo1_tele_op_mode = xian_dj_robotgo1_tele_op_mode;
            web_show.xian_dj_robotgo1_m_system_mode = xian_dj_robotgo1_m_system_mode;
            web_show.xian_dj_robotgo1_s_system_mode = xian_dj_robotgo1_s_system_mode;

            web_show.xian_dj_robotgo1_battery_level = xian_dj_robotgo1_battery_level;
            web_show.xian_dj_robotgo1_display_mode = xian_dj_robotgo1_display_mode;

            web_show.xian_dj_robotgo1_red_ligher_cmd = xian_dj_robotgo1_red_ligher_cmd;
            web_show.xian_dj_robotgo1_green_ligher_cmd = xian_dj_robotgo1_green_ligher_cmd;
            web_show.xian_dj_robotgo1_yellow_ligher_cmd = xian_dj_robotgo1_yellow_ligher_cmd;
            web_show.xian_dj_robotgo1_displayer_cmd = xian_dj_robotgo1_displayer_cmd;

            web_show.xian_dj_local_controller_left_cmd = xian_dj_local_controller_left_cmd;
            web_show.xian_dj_local_controller_right_cmd = xian_dj_local_controller_right_cmd;
            web_show.xian_dj_local_controller_up_cmd = xian_dj_local_controller_up_cmd;
            web_show.xian_dj_local_controller_down_cmd = xian_dj_local_controller_down_cmd;
            web_show.xian_dj_local_controller_x_cmd = xian_dj_local_controller_x_cmd;
            web_show.xian_dj_local_controller_b_cmd = xian_dj_local_controller_b_cmd;
            web_show.xian_dj_local_controller_y_cmd = xian_dj_local_controller_y_cmd;
            web_show.xian_dj_local_controller_a_cmd = xian_dj_local_controller_a_cmd;

            web_show.xian_dj_local_controller_left_rocker_x_cmd = xian_dj_local_controller_left_rocker_x_cmd;
            web_show.xian_dj_local_controller_left_rocker_y_cmd = xian_dj_local_controller_left_rocker_y_cmd;
            web_show.xian_dj_local_controller_right_rocker_x_cmd = xian_dj_local_controller_right_rocker_x_cmd;
            web_show.xian_dj_local_controller_right_rocker_y_cmd = xian_dj_local_controller_right_rocker_y_cmd;

            web_show.xian_dj_local_controller_r1_cmd = xian_dj_local_controller_r1_cmd;
            web_show.xian_dj_local_controller_r2_cmd = xian_dj_local_controller_r2_cmd;
            web_show.xian_dj_local_controller_l1_cmd = xian_dj_local_controller_l1_cmd;
            web_show.xian_dj_local_controller_l2_cmd = xian_dj_local_controller_l2_cmd;

            web_show.xian_dj_robotgo1_params_server_error = xian_dj_robotgo1_params_server_error;
            web_show.xian_dj_robotgo1_state_display_error = xian_dj_robotgo1_state_display_error;
            web_show.xian_dj_robotgo1_switch_mode_error = xian_dj_robotgo1_switch_mode_error;
            web_show.xian_dj_robotgo1_back_end_error = xian_dj_robotgo1_back_end_error;
            web_show.xian_dj_robotgo1_control_error = xian_dj_robotgo1_control_error;
            web_show.xian_dj_robotgo1_lighter_displayer_error = xian_dj_robotgo1_lighter_displayer_error;
            web_show.xian_dj_robotgo1_error_code = xian_dj_robotgo1_error_code;
            web_show.xian_dj_robotgo1_heat_beat_error_code = xian_dj_robotgo1_heat_beat_error_code;

            pub.publish(web_show);
             
        }

    private:
        int counter = 0;
        int xian_dj_robotgo1_back_end_heart_beat = 0;
        ros::Publisher pub; // 声明发布者
        xian_dj_robotgo1_control_pkg::xian_dj_robotgo1_back_end web_show; // 声明要发布的消息类型

        int xian_dj_robotgo1_params_server_heart_beat ;
        int xian_dj_robotgo1_error_monitor_heart_beat ;
        int xian_dj_robotgo1_state_display_heart_beat ;
        int xian_dj_robotgo1_switch_mode_heart_beat ;
        int xian_dj_robotgo1_control_heart_beat ;
        int xian_dj_robotgo1_lighter_displayer_heart_beat ;
        int xian_dj_robotgo1_tele_op_heart_beat ;
        int xian_dj_robotgo1_local_controller_heart_beat ;
        int xian_dj_robotgo1_battery_level_heart_beat ;
        int xian_dj_robotgo1_auto_mode ;
        int xian_dj_robotgo1_tele_op_mode ;
        int xian_dj_robotgo1_m_system_mode ;
        int xian_dj_robotgo1_s_system_mode ;
        std::string xian_dj_robotgo1_battery_level ;
        int xian_dj_robotgo1_display_mode ;
        int xian_dj_robotgo1_red_ligher_cmd ;
        int xian_dj_robotgo1_green_ligher_cmd ;
        int xian_dj_robotgo1_yellow_ligher_cmd ;
        std::string xian_dj_robotgo1_displayer_cmd ;
        int xian_dj_local_controller_left_cmd ;
        int xian_dj_local_controller_right_cmd ;
        int xian_dj_local_controller_up_cmd ;
        int xian_dj_local_controller_down_cmd;
        int xian_dj_local_controller_x_cmd ;
        int xian_dj_local_controller_b_cmd ;
        int xian_dj_local_controller_y_cmd ;
        int xian_dj_local_controller_a_cmd ;

        double xian_dj_local_controller_left_rocker_x_cmd ;
        double xian_dj_local_controller_left_rocker_y_cmd ;
        double xian_dj_local_controller_right_rocker_x_cmd ;
        double xian_dj_local_controller_right_rocker_y_cmd ;

        int xian_dj_local_controller_r1_cmd ;
        int xian_dj_local_controller_r2_cmd ;
        int xian_dj_local_controller_l1_cmd ;
        int xian_dj_local_controller_l2_cmd ;
        int xian_dj_robotgo1_params_server_error ;
        int xian_dj_robotgo1_state_display_error ;
        int xian_dj_robotgo1_switch_mode_error ;
        int xian_dj_robotgo1_back_end_error ;
        int xian_dj_robotgo1_control_error ;
        int xian_dj_robotgo1_lighter_displayer_error ;
        int xian_dj_robotgo1_error_code ;
        int xian_dj_robotgo1_heat_beat_error_code ;

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_back_end");
    XianDjRobotgo1BackEnd xian_dj_robotgo1_back_end;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_robotgo1_back_end.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1BackEnd::m_timer_heart_beat_func, &xian_dj_robotgo1_back_end);
    xian_dj_robotgo1_back_end.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.1), &XianDjRobotgo1BackEnd::m_timer_control_func, &xian_dj_robotgo1_back_end);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}