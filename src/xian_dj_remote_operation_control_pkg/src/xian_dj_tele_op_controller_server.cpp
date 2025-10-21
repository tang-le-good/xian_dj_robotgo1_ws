#include<ros/ros.h>
#include<stdio.h>
#include<sys/types.h>

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 3241
#define BACKLOG 128

// 接收client的数据
struct client2server 
{
    int xian_dj_tele_op_controller_client_tcp_heart_beat; 
    int xian_dj_tele_op_left_client_cmd;
    int xian_dj_tele_op_right_client_cmd;
    int xian_dj_tele_op_up_client_cmd;
    int xian_dj_tele_op_down_client_cmd;
    int xian_dj_tele_op_x_client_cmd;
    int xian_dj_tele_op_b_client_cmd;
    int xian_dj_tele_op_y_client_cmd;
    int xian_dj_tele_op_a_client_cmd;
    double xian_dj_tele_op_left_rocker_x_client_cmd;
    double xian_dj_tele_op_left_rocker_y_client_cmd;
    double xian_dj_tele_op_right_rocker_x_client_cmd;
    double xian_dj_tele_op_right_rocker_y_client_cmd;
    int xian_dj_tele_op_r1_client_cmd;
    int xian_dj_tele_op_r2_client_cmd;
    int xian_dj_tele_op_l1_client_cmd;
    int xian_dj_tele_op_l2_client_cmd;
};

// server发送的数据
struct server2client
{
    int xian_dj_tele_op_controller_server_tcp_heart_beat; 
    // int xian_dj_tele_op_left_server_cmd;
    // int xian_dj_tele_op_right_server_cmd;
    // int xian_dj_tele_op_up_server_cmd;
    // int xian_dj_tele_op_down_server_cmd;
    // int xian_dj_tele_op_x_server_cmd;
    // int xian_dj_tele_op_b_server_cmd;
    // int xian_dj_tele_op_y_server_cmd;
    // int xian_dj_tele_op_a_server_cmd;
    // double xian_dj_tele_op_left_rocker_x_server_cmd;
    // double xian_dj_tele_op_left_rocker_y_server_cmd;
    // double xian_dj_tele_op_right_rocker_x_server_cmd;
    // double xian_dj_tele_op_right_rocker_y_server_cmd;
    // int xian_dj_tele_op_r1_server_cmd;
    // int xian_dj_tele_op_r2_server_cmd;
    // int xian_dj_tele_op_l1_server_cmd;
    // int xian_dj_tele_op_l2_server_cmd;
};

class TCPServer 
{
    private:
        int server_fd;              // 服务器套接字描述符
        int client_fd;              // 客户端套接字描述符
        struct sockaddr_in server_addr;  // 服务器地址结构
        struct sockaddr_in client_addr;  // 客户端地址结构
        bool is_running;            // 服务器运行状态标志
        
    public:
        // 构造函数
        TCPServer() : server_fd(-1), client_fd(-1), is_running(false) 
        {
            memset(&server_addr, 0, sizeof(server_addr));
            memset(&client_addr, 0, sizeof(client_addr));
        }
        
        // 析构函数
        ~TCPServer() 
        {
            stop();
        }
        
        // 初始化服务器
        bool init() 
        {
            // 1. 创建套接字
            if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
            {
                std::cerr << "创建套接字失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            // 2. 设置SO_REUSEADDR选项，解决TIME_WAIT状态导致的绑定失败问题[1,6](@ref)
            int opt = 1;
            if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) 
            {
                std::cerr << "设置SO_REUSEADDR失败: " << strerror(errno) << std::endl;
                close(server_fd);
                return false;
            }
            
            // 3. 配置服务器地址
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = INADDR_ANY;
            server_addr.sin_port = htons(PORT);
            
            return true;
        }
        
        // 启动服务器
        bool start() {
            if (server_fd == -1 && !init()) 
            {
                return false;
            }
            
            // 4. 绑定套接字到端口
            if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) 
            {
                std::cerr << "绑定失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            // 5. 开始监听，增加backlog参数提高并发能力[4](@ref)
            if (listen(server_fd, BACKLOG) < 0) {
                std::cerr << "监听失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            is_running = true;
            std::cout << "服务器启动成功，监听端口 " << PORT << "..." << std::endl;
            
            return true;
        }
        
        // 停止服务器
        void stop() 
        {
            is_running = false;
            
            if (client_fd != -1) 
            {
                close(client_fd);
                client_fd = -1;
            }
            
            if (server_fd != -1) 
            {
                close(server_fd);
                server_fd = -1;
            }
            
            std::cout << "服务器已停止" << std::endl;
        }
        
        // 接受客户端连接
        bool acceptClient() 
        {
            socklen_t client_len = sizeof(client_addr);
            if ((client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len)) < 0) 
            {
                std::cerr << "接受连接失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
            std::cout << "客户端连接成功：" << client_ip << ":" << ntohs(client_addr.sin_port) << std::endl;
            
            return true;
        }
        
        // 接收客户端数据
        int receiveData(client2server& client_data) 
        {
            if (client_fd == -1) 
            {
                return -1;
            }
            
            // memset(&client_data, 0, sizeof(client_data));
            int recv_len = recv(client_fd, &client_data, sizeof(client_data), 0);
            
            if (recv_len <= 0) 
            {
                if (recv_len == 0) {
                    std::cout << "客户端正常断开连接" << std::endl;
                } else {
                    std::cerr << "接收错误: " << strerror(errno) << std::endl;
                }
                return -1;
            }
            
            return recv_len;
        }
        
        // 发送数据到客户端
        bool sendData(server2client& server_data) 
        {
            if (client_fd == -1) 
            {
                return false;
            }
            
            if (send(client_fd, &server_data, sizeof(server_data), 0) < 0) 
            {
                std::cerr << "发送失败: " << strerror(errno) << std::endl;
                return false;
            }
            
            return true;
        }
        
        // 获取客户端文件描述符
        int getClientFD() const 
        {
            return client_fd;
        }
        
        // 检查服务器是否运行
        bool isRunning() const 
        {
            return is_running;
        }
        
        // 关闭客户端连接
        void closeClient() 
        {
            if (client_fd != -1) 
            {
                close(client_fd);
                client_fd = -1;
            }
        }
};


class XianDjTeleOpControllerServer
{
    public:
        XianDjTeleOpControllerServer()
        {
            // 创建一个ROS节点句柄
            ros::NodeHandle nh;

            // 启动服务器
            if (!server.start()) 
            {
                return ;
            }
        }

        ros::WallTimer m_timer_heart_beat;
        ros::WallTimer m_timer_control;

        void m_timer_heart_beat_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_controller_server_heart_beat", xian_dj_tele_op_controller_server_heart_beat); 
            std::cout << "xian_dj_tele_op_controller_server_heart_beat: " << xian_dj_tele_op_controller_server_heart_beat << std::endl;
            counter = counter > 1000 ? 0 : (counter + 1);
            ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_controller_server_heart_beat", counter);  // 自行替换
        }

        void m_timer_control_func(const ros::WallTimerEvent& event)
        {
            ros::param::get("/xian_dj_tele_op_params_server/xian_dj_tele_op_controller_client_tcp_error", xian_dj_tele_op_controller_client_tcp_error); 
            if(xian_dj_tele_op_controller_client_tcp_error != 0)
            {
                // 什么也不用做，等待
            }
            else
            {
                this->command_callback();
            }
            
        }

    private:
        int counter = 0;
        int xian_dj_tele_op_controller_server_heart_beat = 0;
        int xian_dj_tele_op_controller_client_tcp_heart_beat = 0;
        int xian_dj_tele_op_controller_client_tcp_error = 0;

         // 声明鲁班猫接收到的变量
        int xian_dj_tele_op_left_server_cmd;
        int xian_dj_tele_op_right_server_cmd;
        int xian_dj_tele_op_up_server_cmd;
        int xian_dj_tele_op_down_server_cmd;
        int xian_dj_tele_op_x_server_cmd;
        int xian_dj_tele_op_b_server_cmd;
        int xian_dj_tele_op_y_server_cmd;
        int xian_dj_tele_op_a_server_cmd;
        double xian_dj_tele_op_left_rocker_x_server_cmd;
        double xian_dj_tele_op_left_rocker_y_server_cmd;
        double xian_dj_tele_op_right_rocker_x_server_cmd;
        double xian_dj_tele_op_right_rocker_y_server_cmd;
        int xian_dj_tele_op_r1_server_cmd;
        int xian_dj_tele_op_r2_server_cmd;
        int xian_dj_tele_op_l1_server_cmd;
        int xian_dj_tele_op_l2_server_cmd;


        TCPServer server;
        client2server client_data;
        server2client server_data;
        
        int command_callback()
        {
            // 接受客户端连接
            if (!server.acceptClient()) 
            {
                return -1;
            }
            
            // 数据交互循环
            while (true) 
            {
                usleep(20 * 1000); // 20 ms
                // 接收数据
                int recv_len = server.receiveData(client_data);
                if (recv_len <= 0) 
                {
                    break;
                }
                printf("收到数据：xian_dj_tele_op_controller_client_tcp_heart_beat= %d \n", client_data.xian_dj_tele_op_controller_client_tcp_heart_beat);
                printf("收到数据：xian_dj_tele_op_left_client_cmd = %d \n", client_data.xian_dj_tele_op_left_client_cmd);
                printf("收到数据：xian_dj_tele_op_right_client_cmd = %d \n", client_data.xian_dj_tele_op_right_client_cmd);
                printf("收到数据：xian_dj_tele_op_up_client_cmd = %d \n", client_data.xian_dj_tele_op_up_client_cmd);
                printf("收到数据：xian_dj_tele_op_down_client_cmd= %d \n", client_data.xian_dj_tele_op_down_client_cmd);
                printf("收到数据：xian_dj_tele_op_x_client_cmd= %d \n", client_data.xian_dj_tele_op_x_client_cmd);
                printf("收到数据：xian_dj_tele_op_b_client_cmd= %d \n", client_data.xian_dj_tele_op_b_client_cmd);
                printf("收到数据：xian_dj_tele_op_y_client_cmd= %d \n", client_data.xian_dj_tele_op_y_client_cmd);
                printf("收到数据：xian_dj_tele_op_a_client_cmd= %d \n", client_data.xian_dj_tele_op_a_client_cmd);
                printf("收到数据：xian_dj_tele_op_left_rocker_x_client_cmd= %f \n", client_data.xian_dj_tele_op_left_rocker_x_client_cmd);
                printf("收到数据：xian_dj_tele_op_left_rocker_y_client_cmd= %f \n", client_data.xian_dj_tele_op_left_rocker_y_client_cmd);
                printf("收到数据：xian_dj_tele_op_right_rocker_x_client_cmd= %f \n", client_data.xian_dj_tele_op_right_rocker_x_client_cmd);
                printf("收到数据：xian_dj_tele_op_right_rocker_y_client_cmd= %f \n", client_data.xian_dj_tele_op_right_rocker_y_client_cmd);
                printf("收到数据：xian_dj_tele_op_r1_client_cmd= %d \n", client_data.xian_dj_tele_op_r1_client_cmd);
                printf("收到数据：xian_dj_tele_op_r2_client_cmd= %d \n", client_data.xian_dj_tele_op_r2_client_cmd);
                printf("收到数据：xian_dj_tele_op_l1_client_cmd= %d \n", client_data.xian_dj_tele_op_l1_client_cmd);
                printf("收到数据：xian_dj_tele_op_l2_client_cmd= %d \n", client_data.xian_dj_tele_op_l2_client_cmd);

                xian_dj_tele_op_controller_client_tcp_heart_beat = client_data.xian_dj_tele_op_controller_client_tcp_heart_beat;
                xian_dj_tele_op_left_server_cmd = client_data.xian_dj_tele_op_left_client_cmd;
                xian_dj_tele_op_right_server_cmd = client_data.xian_dj_tele_op_right_client_cmd;
                xian_dj_tele_op_up_server_cmd = client_data.xian_dj_tele_op_up_client_cmd;
                xian_dj_tele_op_down_server_cmd = client_data.xian_dj_tele_op_down_client_cmd;
                xian_dj_tele_op_x_server_cmd = client_data.xian_dj_tele_op_x_client_cmd;
                xian_dj_tele_op_b_server_cmd = client_data.xian_dj_tele_op_b_client_cmd;
                xian_dj_tele_op_y_server_cmd = client_data.xian_dj_tele_op_y_client_cmd;
                xian_dj_tele_op_a_server_cmd = client_data.xian_dj_tele_op_a_client_cmd;
                xian_dj_tele_op_left_rocker_x_server_cmd = client_data.xian_dj_tele_op_left_rocker_x_client_cmd;
                xian_dj_tele_op_left_rocker_y_server_cmd = client_data.xian_dj_tele_op_left_rocker_y_client_cmd;
                xian_dj_tele_op_right_rocker_x_server_cmd = client_data.xian_dj_tele_op_right_rocker_x_client_cmd;
                xian_dj_tele_op_right_rocker_y_server_cmd = client_data.xian_dj_tele_op_right_rocker_y_client_cmd;
                xian_dj_tele_op_r1_server_cmd = client_data.xian_dj_tele_op_r1_client_cmd;
                xian_dj_tele_op_r2_server_cmd = client_data.xian_dj_tele_op_r2_client_cmd;
                xian_dj_tele_op_l1_server_cmd = client_data.xian_dj_tele_op_l1_client_cmd;
                xian_dj_tele_op_l2_server_cmd = client_data.xian_dj_tele_op_l2_client_cmd;
                // set param server
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_left_server_cmd", xian_dj_tele_op_left_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_right_server_cmd", xian_dj_tele_op_right_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_up_server_cmd", xian_dj_tele_op_up_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_down_server_cmd", xian_dj_tele_op_down_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_x_server_cmd", xian_dj_tele_op_x_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_b_server_cmd", xian_dj_tele_op_b_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_y_server_cmd", xian_dj_tele_op_y_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_a_server_cmd", xian_dj_tele_op_a_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_left_rocker_x_server_cmd", xian_dj_tele_op_left_rocker_x_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_left_rocker_y_server_cmd", xian_dj_tele_op_left_rocker_y_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_right_rocker_x_server_cmd", xian_dj_tele_op_right_rocker_x_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_right_rocker_y_server_cmd", xian_dj_tele_op_right_rocker_y_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_r1_server_cmd", xian_dj_tele_op_r1_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_r2_server_cmd", xian_dj_tele_op_r2_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_l1_server_cmd", xian_dj_tele_op_l1_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_l2_server_cmd", xian_dj_tele_op_l2_server_cmd); 
                ros::param::set("/xian_dj_tele_op_params_server/xian_dj_tele_op_controller_client_tcp_heart_beat", xian_dj_tele_op_controller_client_tcp_heart_beat); 

                // 构造并发送响应
                server_data.xian_dj_tele_op_controller_server_tcp_heart_beat = xian_dj_tele_op_controller_server_heart_beat;
                printf("发送client的数据：xian_dj_tele_op_controller_server_tcp_heart_beat= %d \n", server_data.xian_dj_tele_op_controller_server_tcp_heart_beat);
                if (!server.sendData(server_data)) 
                {
                    break;
                }
            }
            
            // 关闭当前客户端连接
            server.closeClient();
            return 0;
        }

};

int main(int argc, char** argv)
{
    //initial and name node
    ros::init(argc,argv,"xian_dj_tele_op_controller_server");
    XianDjTeleOpControllerServer xian_dj_tele_op_controller_server;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    xian_dj_tele_op_controller_server.m_timer_heart_beat = nh_2.createWallTimer(ros::WallDuration(1.0), &XianDjTeleOpControllerServer::m_timer_heart_beat_func, &xian_dj_tele_op_controller_server);
    xian_dj_tele_op_controller_server.m_timer_control = nh_2.createWallTimer(ros::WallDuration(2.0), &XianDjTeleOpControllerServer::m_timer_control_func, &xian_dj_tele_op_controller_server);
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}