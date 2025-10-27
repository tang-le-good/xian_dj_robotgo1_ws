
#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <sensor_msgs/Joy.h>

class XianDjRobotgo1SwitchMode
{
    public:
        XianDjRobotgo1SwitchMode()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            controller_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XianDjRobotgo1SwitchMode::controller_callback, this);
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", xian_dj_robotgo1_control_heart_beat); 
            std::cout << "xian_dj_robotgo1_control_heart_beat: " << xian_dj_robotgo1_control_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_control_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {   
            
            std::cout << "xian_dj_robotgo1_tele_op_mode: " << xian_dj_robotgo1_tele_op_mode << std::endl;
            std::cout << "xian_dj_robotgo1_auto_mode: " << xian_dj_robotgo1_auto_mode << std::endl;
            std::cout << "xian_dj_robotgo1_m_system_mode: " << xian_dj_robotgo1_m_system_mode << std::endl;
            std::cout << "xian_dj_robotgo1_s_system_mode: " << xian_dj_robotgo1_s_system_mode << std::endl;
        }

        void controller_callback(const sensor_msgs::Joy::ConstPtr &Joy)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);

            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_r1_client_cmd", xian_dj_tele_op_r1_client_cmd); 
            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_r2_client_cmd", xian_dj_tele_op_r2_client_cmd); 
            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_l1_client_cmd", xian_dj_tele_op_l1_client_cmd); 
            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_l2_client_cmd", xian_dj_tele_op_l2_client_cmd); 

            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r1_cmd", xian_dj_local_controller_r1_cmd); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r2_cmd", xian_dj_local_controller_r2_cmd); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l1_cmd", xian_dj_local_controller_l1_cmd); 
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l2_cmd", xian_dj_local_controller_l2_cmd); 

            xian_dj_tele_op_r2_client_cmd_pre = xian_dj_tele_op_r2_client_cmd_cur;
            xian_dj_tele_op_r2_client_cmd_cur = xian_dj_tele_op_r2_client_cmd;
            xian_dj_local_controller_r2_cmd_pre = xian_dj_local_controller_r2_cmd_cur;
            xian_dj_local_controller_r2_cmd_cur = xian_dj_local_controller_r2_cmd;
            if ((xian_dj_tele_op_r2_client_cmd_pre ==0  && xian_dj_tele_op_r2_client_cmd_cur ==1) || (xian_dj_local_controller_r2_cmd_pre ==0 && xian_dj_local_controller_r2_cmd_cur==1))
            {
              
                xian_dj_robotgo1_tele_op_mode++;
                if(xian_dj_robotgo1_tele_op_mode > 1)
                {
                    xian_dj_robotgo1_tele_op_mode = 0;
                } 
                 
                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_tele_op_mode", xian_dj_robotgo1_tele_op_mode);
            }
            
            // 远程遥控模式：
            if (xian_dj_robotgo1_tele_op_mode == 1)
            {

                xian_dj_tele_op_l2_client_cmd_pre = xian_dj_tele_op_l2_client_cmd_cur;
                xian_dj_tele_op_l2_client_cmd_cur = xian_dj_tele_op_l2_client_cmd;
                xian_dj_local_controller_l2_cmd_pre = xian_dj_local_controller_l2_cmd_cur;
                xian_dj_local_controller_l2_cmd_cur = xian_dj_local_controller_l2_cmd;
                if ((xian_dj_tele_op_l2_client_cmd_pre ==0  && xian_dj_tele_op_l2_client_cmd_cur ==1) || (xian_dj_local_controller_l2_cmd_pre ==0 && xian_dj_local_controller_l2_cmd_cur==1))
                {
                
                    xian_dj_robotgo1_auto_mode++;
                    if(xian_dj_robotgo1_auto_mode > 1)
                    {
                        xian_dj_robotgo1_auto_mode = 0;
                    } 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_auto_mode", xian_dj_robotgo1_auto_mode);
                }
                
                xian_dj_tele_op_l1_client_cmd_pre = xian_dj_tele_op_l1_client_cmd_cur;
                xian_dj_tele_op_l1_client_cmd_cur = xian_dj_tele_op_l1_client_cmd;
                xian_dj_local_controller_l1_cmd_pre = xian_dj_local_controller_l1_cmd_cur;
                xian_dj_local_controller_l1_cmd_cur = xian_dj_local_controller_l1_cmd;
                if ((xian_dj_tele_op_l1_client_cmd_pre ==0  && xian_dj_tele_op_l1_client_cmd_cur ==1) || (xian_dj_local_controller_l1_cmd_pre ==0 && xian_dj_local_controller_l1_cmd_cur==1))
                {
                    xian_dj_robotgo1_m_system_mode++;
                    if(xian_dj_robotgo1_m_system_mode > 3)
                    {
                        xian_dj_robotgo1_m_system_mode = 0;
                    } 
                    ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_m_system_mode", xian_dj_robotgo1_m_system_mode);
                }
  
                xian_dj_tele_op_r1_client_cmd_pre = xian_dj_tele_op_r1_client_cmd_cur;
                xian_dj_tele_op_r1_client_cmd_cur = xian_dj_tele_op_r1_client_cmd;
                xian_dj_local_controller_r1_cmd_pre = xian_dj_local_controller_r1_cmd_cur;
                xian_dj_local_controller_r1_cmd_cur = xian_dj_local_controller_r1_cmd;
                if ((xian_dj_tele_op_r1_client_cmd_pre ==0  && xian_dj_tele_op_r1_client_cmd_cur ==1) || (xian_dj_local_controller_r1_cmd_pre ==0 && xian_dj_local_controller_r1_cmd_cur==1))
                {
                    if (xian_dj_robotgo1_m_system_mode == 3) 
                    {
                        xian_dj_robotgo1_s_system_mode++;
                        if(xian_dj_robotgo1_s_system_mode > 3)
                        {
                            xian_dj_robotgo1_s_system_mode = 0;
                        }
                    }  
                    else
                    {
                        xian_dj_robotgo1_s_system_mode = 0;
                    }
                             
                }
                ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_s_system_mode", xian_dj_robotgo1_s_system_mode);
                
            }   
        }

    private:
        ros::Subscriber controller_sub;
        int counter = 0;
        int xian_dj_robotgo1_control_heart_beat = 0;
        // 模式变量
        int xian_dj_robotgo1_auto_mode = 0;
        int xian_dj_robotgo1_tele_op_mode = 0;
        int xian_dj_robotgo1_m_system_mode = 0;
        int xian_dj_robotgo1_s_system_mode = 0;

        int xian_dj_tele_op_r1_client_cmd = 0;
        int xian_dj_tele_op_r2_client_cmd = 0;
        int xian_dj_tele_op_l1_client_cmd = 0;
        int xian_dj_tele_op_l2_client_cmd = 0;

        int xian_dj_local_controller_r1_cmd = 0;
        int xian_dj_local_controller_r2_cmd = 0;
        int xian_dj_local_controller_l1_cmd = 0;
        int xian_dj_local_controller_l2_cmd = 0;

        // 中间变量
        int xian_dj_tele_op_r1_client_cmd_pre = 0;
        int xian_dj_tele_op_r2_client_cmd_pre = 0;
        int xian_dj_tele_op_l1_client_cmd_pre = 0;
        int xian_dj_tele_op_l2_client_cmd_pre = 0;

        int xian_dj_tele_op_r1_client_cmd_cur = 0;
        int xian_dj_tele_op_r2_client_cmd_cur = 0;
        int xian_dj_tele_op_l1_client_cmd_cur = 0;
        int xian_dj_tele_op_l2_client_cmd_cur = 0;

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

    xian_dj_robotgo1_switch_mode.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1SwitchMode::m_timer_heart_beat_func, &xian_dj_robotgo1_switch_mode);
    xian_dj_robotgo1_switch_mode.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjRobotgo1SwitchMode::m_timer_control_func, &xian_dj_robotgo1_switch_mode);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}
