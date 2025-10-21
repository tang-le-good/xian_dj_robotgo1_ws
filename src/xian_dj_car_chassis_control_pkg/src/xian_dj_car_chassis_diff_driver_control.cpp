#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>

class XianDjCarChassisDiffDriverControl
{
    public:
        XianDjCarChassisDiffDriverControl()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_car_chassis_param_pkg/xian_dj_car_chassis_diff_driver_control_heart_beat", xian_dj_car_chassis_diff_driver_control_heart_beat); 
            std::cout << "xian_dj_car_chassis_diff_driver_control_heart_beat: " << xian_dj_car_chassis_diff_driver_control_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_car_chassis_param_pkg/xian_dj_car_chassis_diff_driver_control_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_car_chassis_param_pkg/xian_dj_car_chassis_diff_driver_control_error", xian_dj_car_chassis_diff_driver_control_error); 
            if(xian_dj_car_chassis_diff_driver_control_error != 0)
            {
                // 什么也不用做，等待
            }
            else
            {
                ros::param::get("/xian_dj_car_chassis_param_pkg/input_velocity_cmd", input_velocity_cmd); 
                ros::param::get("/xian_dj_car_chassis_param_pkg/input_theta_cmd", input_theta_cmd); 
                input_velocity = input_velocity_cmd;
                input_theta = input_theta_cmd;
                delta_v1c = input_velocity;
                delta_v2c = input_velocity;
                delta_theta_1 = delta_v1c - input_theta;
                delta_theta_2 = delta_v2c + input_theta;
                xian_dj_car_chassis_delta_theta1c = delta_theta_1;
                xian_dj_car_chassis_delta_theta2c = delta_theta_2;

                ros::param::set("/xian_dj_car_chassis_param_pkg/xian_dj_car_chassis_delta_theta1c", xian_dj_car_chassis_delta_theta1c); 
                ros::param::set("/xian_dj_car_chassis_param_pkg/xian_dj_car_chassis_delta_theta2c", xian_dj_car_chassis_delta_theta2c); 
            }
        }

    private:
        // 心跳
        int counter = 0;
        int xian_dj_car_chassis_diff_driver_control_heart_beat = 0;
        // 输入量
        double input_velocity_cmd = 0;
        double input_theta_cmd = 0;
        // 中间变量
        double input_velocity = 0;
        double input_theta = 0;
        double delta_v1c = 0;
        double delta_v2c = 0;
        double delta_theta_1 = 0;
        double delta_theta_2 = 0;
        //输出量
        double xian_dj_car_chassis_delta_theta1c = 0;
        double xian_dj_car_chassis_delta_theta2c = 0;
        
        int xian_dj_car_chassis_diff_driver_control_error = 0;

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_car_chassis_diff_driver_control");
    XianDjCarChassisDiffDriverControl xian_dj_car_chassis_diff_driver_control;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_car_chassis_diff_driver_control.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjCarChassisDiffDriverControl::m_timer_heart_beat_func, &xian_dj_car_chassis_diff_driver_control);
    xian_dj_car_chassis_diff_driver_control.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjCarChassisDiffDriverControl::m_timer_control_func, &xian_dj_car_chassis_diff_driver_control);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}