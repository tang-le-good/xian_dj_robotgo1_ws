#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "xian_dj_visual_platform_param_pkg/xian_dj_visual_platform_paramsConfig.h"
//define call back function
void paramCallback(xian_dj_visual_platform_param_pkg::xian_dj_visual_platform_paramsConfig& config,uint32_t level)
{
    ROS_INFO("Request");
}
int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_visual_platform_params_server");
    //create node handle
    dynamic_reconfigure::Server<xian_dj_visual_platform_param_pkg::xian_dj_visual_platform_paramsConfig> server;
    dynamic_reconfigure::Server<xian_dj_visual_platform_param_pkg::xian_dj_visual_platform_paramsConfig>::CallbackType f;
    f = boost::bind(&paramCallback,_1,_2);
    server.setCallback(f);

    int counter = 0;
    int xian_dj_visual_platform_params_server_heart_beat = 0;
    while(ros::ok())
    {
        usleep(1000 * 1000); // 1000 ms
        ros::param::set("/xian_dj_visual_platform_params_server/xian_dj_visual_platform_params_server_heart_beat", counter); // 原始检测结果
        ros::param::get("/xian_dj_visual_platform_params_server/xian_dj_visual_platform_params_server_heart_beat", xian_dj_visual_platform_params_server_heart_beat);
        std::cout << "xian_dj_visual_platform_params_server_heart_beat:" << xian_dj_visual_platform_params_server_heart_beat << std::endl;
        if(counter > 1000)
        {
            counter = 0;
        }

        counter ++;
    }

    ros::spin();
    return 0;
}