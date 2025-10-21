#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>
#include <sensor_msgs/Joy.h>

class XianDjStewartPlatformManualControl
{
    public:
        XianDjStewartPlatformManualControl()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;
            joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10,&XianDjStewartPlatformManualControl::joyCallback, this);
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;
        // 节点心跳
        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_manual_controller_heart_beat", xian_dj_stewart_platform_manual_controller_heart_beat); 
            std::cout << "xian_dj_stewart_platform_manual_controller_heart_beat: " << xian_dj_stewart_platform_manual_controller_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_manual_controller_heart_beat", counter);  // 自行替换
        }
 
        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            // 获取参数服务器中的这12个变量
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_positive_cmd", xian_dj_stewart_platform_input_alpha_positive_cmd); 
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_negative_cmd", xian_dj_stewart_platform_input_alpha_negative_cmd);
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_positive_cmd", xian_dj_stewart_platform_input_beta_positive_cmd); 
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_negative_cmd", xian_dj_stewart_platform_input_beta_negative_cmd);  
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_positive_cmd", xian_dj_stewart_platform_input_gamma_positive_cmd); 
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_negative_cmd", xian_dj_stewart_platform_input_gamma_negative_cmd);
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_positive_cmd", xian_dj_stewart_platform_input_x_positive_cmd); 
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_negative_cmd", xian_dj_stewart_platform_input_x_negative_cmd);
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_positive_cmd", xian_dj_stewart_platform_input_y_positive_cmd); 
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_negative_cmd", xian_dj_stewart_platform_input_y_negative_cmd);  
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_positive_cmd", xian_dj_stewart_platform_input_z_positive_cmd); 
            ros::param::get("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_negative_cmd", xian_dj_stewart_platform_input_z_negative_cmd);   
            if(xian_dj_stewart_platform_input_x_positive_cmd == 1)
            {
                input_x += step_length;
            }
            if(xian_dj_stewart_platform_input_x_negative_cmd == -1)
            {
                input_x -= step_length;
            }
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_x_cmd", input_x);

            if(xian_dj_stewart_platform_input_y_positive_cmd == 1)
            {
                input_y += step_length;
            }
            if(xian_dj_stewart_platform_input_y_negative_cmd == -1)
            {
                input_y -= step_length;
            }
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_y_cmd", input_y);

            if(xian_dj_stewart_platform_input_z_positive_cmd == 1)
            {
                input_z += step_length;
            }
            if(xian_dj_stewart_platform_input_z_negative_cmd == 1)
            {
                input_z -= step_length;
            }
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_z_cmd", input_z);

            printf("input_x: %0.2f, input_y: %0.2f, input_z: %0.2f \n", input_x, input_y, input_z);


            if(xian_dj_stewart_platform_input_alpha_positive_cmd == 1)
            {
                input_alpha += step_length;
            }
            if(xian_dj_stewart_platform_input_alpha_negative_cmd == -1)
            {
                input_alpha -= step_length;
            }
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_alpha_cmd", input_alpha);

            if(xian_dj_stewart_platform_input_beta_positive_cmd == 1)
            {
                input_beta += step_length;
            }
            if(xian_dj_stewart_platform_input_beta_negative_cmd == -1)
            {
                input_beta -= step_length;
            }
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_beta_cmd", input_beta);

            if(xian_dj_stewart_platform_input_gamma_positive_cmd == 1)
            {
                input_gamma += step_length;
            }
            if(xian_dj_stewart_platform_input_gamma_negative_cmd == 1)
            {
                input_gamma -= step_length;
            }
            ros::param::set("/xian_dj_stewart_platform_params_server/xian_dj_stewart_platform_input_gamma_cmd", input_gamma);

            printf("input_alpha: %0.2f, input_beta: %0.2f, input_gamma: %0.2f \n", input_alpha, input_beta, input_gamma);
            
        }
        void joySetup();
        void joyCallback(const sensor_msgs::JoyConstPtr& msg)
        {
            Joy = msg;
        }

    private:
        sensor_msgs::JoyConstPtr Joy;
        ros::Subscriber joy_sub;

        int counter = 0;
        int xian_dj_stewart_platform_manual_controller_heart_beat = 0;

        double input_alpha = 0;
        double input_beta = 0;
        double input_gamma = 0;
        double input_x = 0;
        double input_y = 0;
        double input_z = 168;  //杆长为160时，动平台水平对应的Z高度
        double step_length = 0.25 ;

        double xian_dj_stewart_platform_input_alpha_cmd = 0;
        double xian_dj_stewart_platform_input_beta_cmd = 0;
        double xian_dj_stewart_platform_input_gamma_cmd = 0;
        double xian_dj_stewart_platform_input_x_cmd = 0;
        double xian_dj_stewart_platform_input_y_cmd = 0;
        double xian_dj_stewart_platform_input_z_cmd = 0;

        int xian_dj_stewart_platform_input_alpha_positive_cmd = 0;
        int xian_dj_stewart_platform_input_alpha_negative_cmd = 0;
        int xian_dj_stewart_platform_input_beta_positive_cmd = 0;
        int xian_dj_stewart_platform_input_beta_negative_cmd = 0;
        int xian_dj_stewart_platform_input_gamma_positive_cmd = 0;
        int xian_dj_stewart_platform_input_gamma_negative_cmd = 0;
        int xian_dj_stewart_platform_input_x_positive_cmd = 0;
        int xian_dj_stewart_platform_input_x_negative_cmd = 0;
        int xian_dj_stewart_platform_input_y_positive_cmd = 0;
        int xian_dj_stewart_platform_input_y_negative_cmd = 0;
        int xian_dj_stewart_platform_input_z_positive_cmd = 0;
        int xian_dj_stewart_platform_input_z_negative_cmd = 0;

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_stewart_platform_manual_controller");
    XianDjStewartPlatformManualControl xian_dj_stewart_platform_manual_controller;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_stewart_platform_manual_controller.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjStewartPlatformManualControl::m_timer_heart_beat_func, &xian_dj_stewart_platform_manual_controller);
    xian_dj_stewart_platform_manual_controller.m_timer_control = nh_2.createWallTimer(ros::WallDuration(0.02), &XianDjStewartPlatformManualControl::m_timer_control_func, &xian_dj_stewart_platform_manual_controller);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}