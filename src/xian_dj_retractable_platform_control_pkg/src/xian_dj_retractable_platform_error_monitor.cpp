#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>

class XianDjRetractablePlatformErrorMonitor
{
    public:
        XianDjRetractablePlatformErrorMonitor()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_error_monitor_heart_beat", xian_dj_retractable_platform_error_monitor_heart_beat); 
            std::cout << "xian_dj_retractable_platform_error_monitor_heart_beat: " << xian_dj_retractable_platform_error_monitor_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_error_monitor_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_params_server_heart_beat", xian_dj_retractable_platform_params_server_heart_beat); 
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_control_heart_beat", xian_dj_retractable_platform_control_heart_beat); 
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_base_electric_heart_beat", xian_dj_retractable_platform_base_electric_heart_beat); 
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_back_end_heart_beat", xian_dj_retractable_platform_back_end_heart_beat); 
            
            // 监控 xian_dj_retractable_platform_params_server_heart_beat
            xian_dj_retractable_platform_params_server_heart_beat_pre = xian_dj_retractable_platform_params_server_heart_beat_cur;
            xian_dj_retractable_platform_params_server_heart_beat_cur = xian_dj_retractable_platform_params_server_heart_beat;
            if(xian_dj_retractable_platform_params_server_heart_beat_pre == xian_dj_retractable_platform_params_server_heart_beat_cur)
            {
                xian_dj_retractable_platform_params_server_node_restart_flag ++;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_params_server_error", 1); 
            }
            else
            {
                xian_dj_retractable_platform_params_server_node_restart_flag = 0;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_params_server_error", 0);
            }
            if(xian_dj_retractable_platform_params_server_node_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                // logline = timeStr + "   xian_dj_retractable_platform_params_server_node error";
                // logFile.Write(logline);
                // std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_dj_retractable_platform_params_server | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_dj_retractable_platform_param_pkg xian_dj_retractable_platform_params_server &";
                system(command_restart_current_node.c_str());
                xian_dj_retractable_platform_params_server_node_restart_flag = 0; // 重启后多等20s
            }

            // 监控 xian_dj_retractable_platform_control_heart_beat
            xian_dj_retractable_platform_control_heart_beat_pre = xian_dj_retractable_platform_control_heart_beat_cur;
            xian_dj_retractable_platform_control_heart_beat_cur = xian_dj_retractable_platform_control_heart_beat;
            if(xian_dj_retractable_platform_control_heart_beat_pre == xian_dj_retractable_platform_control_heart_beat_cur)
            {
                xian_dj_retractable_platform_control_node_restart_flag ++;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_control_error", 1); 
            }
            else
            {
                xian_dj_retractable_platform_control_node_restart_flag = 0;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_control_error", 0);
            }
            if(xian_dj_retractable_platform_control_node_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                // logline = timeStr + "   xian_dj_retractable_platform_control_node error";
                // logFile.Write(logline);
                // std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_dj_retractable_platform_control | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_dj_retractable_platform_control_pkg xian_dj_retractable_platform_control &";
                system(command_restart_current_node.c_str());
                xian_dj_retractable_platform_control_node_restart_flag = 0; // 重启后多等20s
            }

            // 监控 xian_dj_retractable_platform_base_electric_heart_beat
            xian_dj_retractable_platform_base_electric_heart_beat_pre = xian_dj_retractable_platform_base_electric_heart_beat_cur;
            xian_dj_retractable_platform_base_electric_heart_beat_cur = xian_dj_retractable_platform_base_electric_heart_beat;
            if(xian_dj_retractable_platform_base_electric_heart_beat_pre == xian_dj_retractable_platform_base_electric_heart_beat_cur)
            {
                xian_dj_retractable_platform_base_electric_node_restart_flag ++;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_base_electric_error", 1); 
            }
            else
            {
                xian_dj_retractable_platform_base_electric_node_restart_flag = 0;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_base_electric_error", 0);
            }
            if(xian_dj_retractable_platform_base_electric_node_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                // logline = timeStr + "   xian_dj_retractable_platform_base_electric_node error";
                // logFile.Write(logline);
                // std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_dj_retractable_platform_base_electric | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_dj_retractable_platform_control_pkg xian_dj_retractable_platform_base_electric &";
                system(command_restart_current_node.c_str());
                xian_dj_retractable_platform_base_electric_node_restart_flag = 0; // 重启后多等20s
            }

            // 监控 xian_dj_retractable_platform_back_end_heart_beat
            xian_dj_retractable_platform_back_end_heart_beat_pre = xian_dj_retractable_platform_back_end_heart_beat_cur;
            xian_dj_retractable_platform_back_end_heart_beat_cur = xian_dj_retractable_platform_back_end_heart_beat;
            if(xian_dj_retractable_platform_back_end_heart_beat_pre == xian_dj_retractable_platform_back_end_heart_beat_cur)
            {
                xian_dj_retractable_platform_back_end_node_restart_flag ++;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_back_end_error", 1); 
            }
            else
            {
                xian_dj_retractable_platform_back_end_node_restart_flag = 0;
                ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_back_end_error", 0);
            }
            if(xian_dj_retractable_platform_back_end_node_restart_flag > v_global_restart_max_count)
            {
                // 写入故障日志
                // logline = timeStr + "   xian_dj_retractable_platform_back_end_node error";
                // logFile.Write(logline);
                // std::cout << logline << std::endl;
                // 重启节点
                command_kill_current_node = "ps -x | grep xian_dj_retractable_platform_back_end | grep -v 'grep' | kill `awk '{ print $1}'`";
                system(command_kill_current_node.c_str());
                usleep(v_restart_sleep_time);
                command_restart_current_node = "rosrun xian_dj_retractable_platform_control_pkg xian_dj_retractable_platform_back_end &";
                system(command_restart_current_node.c_str());
                xian_dj_retractable_platform_back_end_node_restart_flag = 0; // 重启后多等20s
            }
            error_sum = xian_dj_retractable_platform_params_server_error + xian_dj_retractable_platform_control_error + xian_dj_retractable_platform_base_electric_error + xian_dj_retractable_platform_back_end_error ;
            xian_dj_retractable_platform_error_code = error_sum + 200;
            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_error_code", xian_dj_retractable_platform_error_code);

        }
    private:

        std::string command_kill_current_node = "";
        std::string command_restart_current_node = "";

        int counter = 0;
        int xian_dj_retractable_platform_error_monitor_heart_beat = 0;

        int xian_dj_retractable_platform_params_server_heart_beat = 0;
        int xian_dj_retractable_platform_control_heart_beat = 0;
        int xian_dj_retractable_platform_base_electric_heart_beat = 0;
        int xian_dj_retractable_platform_back_end_heart_beat = 0;

        int xian_dj_retractable_platform_params_server_heart_beat_cur = 0;
        int xian_dj_retractable_platform_params_server_heart_beat_pre = 0;
        int xian_dj_retractable_platform_control_heart_beat_cur = 0;
        int xian_dj_retractable_platform_control_heart_beat_pre = 0;
        int xian_dj_retractable_platform_base_electric_heart_beat_cur = 0;
        int xian_dj_retractable_platform_base_electric_heart_beat_pre = 0;
        int xian_dj_retractable_platform_back_end_heart_beat_cur = 0;
        int xian_dj_retractable_platform_back_end_heart_beat_pre = 0;

        int xian_dj_retractable_platform_params_server_node_restart_flag = 0;
        int xian_dj_retractable_platform_control_node_restart_flag = 0;
        int xian_dj_retractable_platform_base_electric_node_restart_flag = 0;
        int xian_dj_retractable_platform_back_end_node_restart_flag = 0;

        int xian_dj_retractable_platform_params_server_error = 0;
        int xian_dj_retractable_platform_control_error = 0;
        int xian_dj_retractable_platform_base_electric_error = 0;
        int xian_dj_retractable_platform_back_end_error = 0;

        int v_global_restart_max_count = 500;
        useconds_t v_restart_sleep_time = 1000 * 1000;

        int error_sum = 0;
        int xian_dj_retractable_platform_error_code = 0;

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_retractable_platform_error_monitor");
    XianDjRetractablePlatformErrorMonitor xian_dj_retractable_platform_error_monitor;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_retractable_platform_error_monitor.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRetractablePlatformErrorMonitor::m_timer_heart_beat_func, &xian_dj_retractable_platform_error_monitor);
    xian_dj_retractable_platform_error_monitor.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjRetractablePlatformErrorMonitor::m_timer_control_func, &xian_dj_retractable_platform_error_monitor);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}