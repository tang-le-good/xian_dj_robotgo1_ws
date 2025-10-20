#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <sensor_msgs/Joy.h>

class XianDjRobotgo1LocalController
{
    public:
        XianDjRobotgo1LocalController()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            controller_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XianDjRobotgo1LocalController::controller_callback, this);
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_local_controller_heart_beat", xian_dj_robotgo1_local_controller_heart_beat); 
            std::cout << "xian_dj_robotgo1_local_controller_heart_beat: " << xian_dj_robotgo1_local_controller_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_local_controller_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_cmd", xian_dj_local_controller_left_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_cmd", xian_dj_local_controller_right_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_up_cmd", xian_dj_local_controller_up_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_down_cmd", xian_dj_local_controller_down_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_x_cmd", xian_dj_local_controller_x_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_b_cmd", xian_dj_local_controller_b_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_y_cmd", xian_dj_local_controller_y_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_a_cmd", xian_dj_local_controller_a_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_rocker_x_cmd", xian_dj_local_controller_left_rocker_x_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_left_rocker_y_cmd", xian_dj_local_controller_left_rocker_y_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_rocker_x_cmd", xian_dj_local_controller_right_rocker_x_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_right_rocker_y_cmd", xian_dj_local_controller_right_rocker_y_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r1_cmd", xian_dj_local_controller_r1_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_r2_cmd", xian_dj_local_controller_r2_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l1_cmd", xian_dj_local_controller_l1_cmd);
            ros::param::set("/xian_dj_robotgo1_params_server/xian_dj_local_controller_l2_cmd", xian_dj_local_controller_l2_cmd);

        }
        void controller_callback(const sensor_msgs::Joy::ConstPtr &Joy)
        {
            axes_6_common = Joy->axes[6];
            switch (axes_6_common)
            {
                case 0:
                    xian_dj_local_controller_left_cmd = 0;
                    xian_dj_local_controller_right_cmd = 0;
                    break;
                case 1:
                    xian_dj_local_controller_left_cmd = 1;
                    xian_dj_local_controller_right_cmd = 0;
                    break;  
                case -1:
                    xian_dj_local_controller_left_cmd = 0;
                    xian_dj_local_controller_right_cmd = 1;
                    break;
            }

            axes_7_common = Joy->axes[7];
            switch (axes_7_common)
            {
                case 0:
                    xian_dj_local_controller_up_cmd = 0;
                    xian_dj_local_controller_down_cmd = 0;
                    break;
                case 1:
                    xian_dj_local_controller_up_cmd = 1;
                    xian_dj_local_controller_down_cmd = 0;
                    break;  
                case -1:
                    xian_dj_local_controller_up_cmd = 0;
                    xian_dj_local_controller_down_cmd = 1;
                    break;
            }
            // xian_dj_local_controller_left_cmd = Joy->axes[6];
            // xian_dj_local_controller_right_cmd = Joy->axes[6];
            // xian_dj_local_controller_up_cmd = Joy->axes[7];
            // xian_dj_local_controller_down_cmd = Joy->axes[7];
            xian_dj_local_controller_x_cmd = Joy->buttons[3];
            xian_dj_local_controller_b_cmd = Joy->buttons[1];
            xian_dj_local_controller_y_cmd = Joy->buttons[4];
            xian_dj_local_controller_a_cmd = Joy->buttons[0];
            xian_dj_local_controller_left_rocker_x_cmd = Joy->axes[0];
            xian_dj_local_controller_left_rocker_y_cmd = Joy->axes[1];
            xian_dj_local_controller_right_rocker_x_cmd = Joy->axes[2];
            xian_dj_local_controller_right_rocker_y_cmd = Joy->axes[3];
            xian_dj_local_controller_r1_cmd = Joy->buttons[7];
            xian_dj_local_controller_r2_cmd = Joy->buttons[9];
            xian_dj_local_controller_l1_cmd = Joy->buttons[6];
            xian_dj_local_controller_l2_cmd = Joy->buttons[8];
            
        }

    private:
        ros::Subscriber controller_sub;
        int counter = 0;
        int xian_dj_robotgo1_local_controller_heart_beat = 0;

        int xian_dj_local_controller_left_cmd = 0;
        int xian_dj_local_controller_right_cmd = 0;
        int xian_dj_local_controller_up_cmd = 0;
        int xian_dj_local_controller_down_cmd =0;
        int xian_dj_local_controller_x_cmd = 0;
        int xian_dj_local_controller_b_cmd = 0;
        int xian_dj_local_controller_y_cmd = 0;
        int xian_dj_local_controller_a_cmd = 0;

        double xian_dj_local_controller_left_rocker_x_cmd = 0;
        double xian_dj_local_controller_left_rocker_y_cmd = 0;
        double xian_dj_local_controller_right_rocker_x_cmd = 0;
        double xian_dj_local_controller_right_rocker_y_cmd = 0;

        int xian_dj_local_controller_r1_cmd = 0;
        int xian_dj_local_controller_r2_cmd = 0;
        int xian_dj_local_controller_l1_cmd = 0;
        int xian_dj_local_controller_l2_cmd = 0;

        int axes_6_common = 0;
        int axes_7_common = 0;
};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_local_controller");
    XianDjRobotgo1LocalController xian_dj_robotgo1_local_controller;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_robotgo1_local_controller.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjRobotgo1LocalController::m_timer_heart_beat_func, &xian_dj_robotgo1_local_controller);
    xian_dj_robotgo1_local_controller.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjRobotgo1LocalController::m_timer_control_func, &xian_dj_robotgo1_local_controller);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}