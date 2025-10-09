#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "xian_dj_robotgo1_param_pkg/xian_dj_robotgo1_paramsConfig.h"
//define call back function
void paramCallback(xian_dj_robotgo1_param_pkg::xian_dj_robotgo1_paramsConfig& config,uint32_t level)
{
    ROS_INFO("Request");
}
int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_robotgo1_params_server");
    //create node handle
    dynamic_reconfigure::Server<xian_dj_robotgo1_param_pkg::xian_dj_robotgo1_paramsConfig> server;
    dynamic_reconfigure::Server<xian_dj_robotgo1_param_pkg::xian_dj_robotgo1_paramsConfig>::CallbackType f;
    f = boost::bind(&paramCallback,_1,_2);
    server.setCallback(f);
    ros::spin();
    return 0;
}