#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>

class XianDjRetractablePlatformControl
{
    public:
        XianDjRetractablePlatformControl()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_control_heart_beat", xian_dj_retractable_platform_control_heart_beat); 
            std::cout << "xian_dj_retractable_platform_control_heart_beat: " << xian_dj_retractable_platform_control_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_control_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_base_electric_error", xian_dj_retractable_platform_base_electric_error); 
            if(xian_dj_retractable_platform_base_electric_error != 0)
            {
                // 什么也不用做，等待
            }
            else
            {
                ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_stand_cmd", xian_dj_retractable_platform_stand_linear_actuator_stand_cmd);
                ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_sit_cmd", xian_dj_retractable_platform_stand_linear_actuator_sit_cmd);
                ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_up_cmd", xian_dj_retractable_platform_first_linear_actuator_up_cmd);
                ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_down_cmd", xian_dj_retractable_platform_first_linear_actuator_down_cmd);
                ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_up_cmd", xian_dj_retractable_platform_second_linear_actuator_up_cmd);
                ros::param::get("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_down_cmd", xian_dj_retractable_platform_second_linear_actuator_down_cmd);

                // 控制倒伏伸缩杆运动状态
                if (xian_dj_retractable_platform_stand_linear_actuator_stand_cmd==1 || xian_dj_retractable_platform_stand_linear_actuator_sit_cmd==0)
                {
                    ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_enble", 1);
                    ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_move", 1);
                }
                else
                {
                    if (xian_dj_retractable_platform_stand_linear_actuator_stand_cmd==0 || xian_dj_retractable_platform_stand_linear_actuator_sit_cmd==1)
                    {
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_enble", 1);
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_move", 0);
                    }
                    else
                    {
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_enble", 0);
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_stand_linear_actuator_move", 0);
                    }
                }

                // 控制一级伸缩杆运动状态
                if (xian_dj_retractable_platform_first_linear_actuator_up_cmd==1 || xian_dj_retractable_platform_first_linear_actuator_down_cmd==0)
                {
                    ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_enble", 1);
                    ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_move", 1);
                }
                else
                {
                    if (xian_dj_retractable_platform_first_linear_actuator_up_cmd==0 || xian_dj_retractable_platform_first_linear_actuator_down_cmd==1)
                    {
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_enble", 1);
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_move", 0);
                    }
                    else
                    {
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_enble", 0);
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_first_linear_actuator_move", 0);
                    }
                }

                // 控制二级伸缩杆运动状态
                if (xian_dj_retractable_platform_second_linear_actuator_up_cmd==1 || xian_dj_retractable_platform_second_linear_actuator_down_cmd==0)
                {
                    ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_enble", 1);
                    ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_move", 1);
                }
                else
                {
                    if (xian_dj_retractable_platform_second_linear_actuator_up_cmd==0 || xian_dj_retractable_platform_second_linear_actuator_down_cmd==1)
                    {
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_enble", 1);
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_move", 0);
                    }
                    else
                    {
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_enble", 0);
                        ros::param::set("/xian_dj_retractable_platform_params_server/xian_dj_retractable_platform_second_linear_actuator_move", 0);
                    }
                }
            }
        }

    private:
        int counter = 0;
        int xian_dj_retractable_platform_control_heart_beat = 0;
        
        int xian_dj_retractable_platform_stand_linear_actuator_enble = 0;
        int xian_dj_retractable_platform_stand_linear_actuator_move = 0;
        int xian_dj_retractable_platform_first_linear_actuator_enble = 0;
        int xian_dj_retractable_platform_first_linear_actuator_move = 0;
        int xian_dj_retractable_platform_second_linear_actuator_enble = 0;
        int xian_dj_retractable_platform_second_linear_actuator_move = 0;
        int xian_dj_retractable_platform_stand_linear_actuator_stand_cmd = 0;
        int xian_dj_retractable_platform_stand_linear_actuator_sit_cmd = 0;
        int xian_dj_retractable_platform_first_linear_actuator_up_cmd = 0;
        int xian_dj_retractable_platform_first_linear_actuator_down_cmd = 0;
        int xian_dj_retractable_platform_second_linear_actuator_up_cmd = 0;
        int xian_dj_retractable_platform_second_linear_actuator_down_cmd = 0;

        int xian_dj_retractable_platform_base_electric_error = 0;

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_retractable_platform_control");
    XianDjRetractablePlatformControl xian_dj_retractable_platform_control;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_retractable_platform_control.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRetractablePlatformControl::m_timer_heart_beat_func, &xian_dj_retractable_platform_control);
    xian_dj_retractable_platform_control.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjRetractablePlatformControl::m_timer_control_func, &xian_dj_retractable_platform_control);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}