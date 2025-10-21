#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

class OdometryCalculator {
public:
    OdometryCalculator() {
        ros::NodeHandle nh;
        // ros::NodeHandle private_nh("~");

        // // 参数初始化：车轮直径、轮距
        // private_nh.param<double>("wheel_diameter_left", d1, 0.1);
        // private_nh.param<double>("wheel_diameter_right", d2, 0.1);
        // private_nh.param<double>("wheel_separation", l, 0.5);

        // 初始化位姿和速度
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;

        // 上一次接收到的轮速值和时间
        last_left_speed = 0.0;
        last_right_speed = 0.0;
        last_time = ros::Time::now();

        // // 订阅轮速话题（这里假设左右轮速是分开的两个话题）
        // left_wheel_sub = nh.subscribe("left_wheel_speed", 10, &OdometryCalculator::leftWheelCallback, this);
        // right_wheel_sub = nh.subscribe("right_wheel_speed", 10, &OdometryCalculator::rightWheelCallback, this);

        // 发布里程计话题和tf变换
        odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom", 50);
        odom_broadcaster = new tf::TransformBroadcaster();
    }

    // void leftWheelCallback(const std_msgs::Float32::ConstPtr& msg) {
    //     left_speed = msg->data;
    // }

    // void rightWheelCallback(const std_msgs::Float32::ConstPtr& msg) {
    //     right_speed = msg->data;
    // }

    void updateOdometry() {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        ros::param::get("/xian_magnet_agv_dynamic_params/v1", left_speed);
        ros::param::get("/xian_magnet_agv_dynamic_params/v2", right_speed);

        left_speed = left_speed / 25.0;
        right_speed = right_speed / 25.0 * -1;

        if (dt > 0.0) {
            // 计算左右轮的实际线速度（v = ω * r，这里假设输入的轮速是转速rad/s）
            double left_v = (left_speed * d1) / 2.0;
            double right_v = (right_speed * d2) / 2.0;

            // 计算机器人的线速度和角速度（差速模型）
            vx = (left_v + right_v) / 2.0;
            vth = (right_v - left_v) / l;

            // 计算位姿变化量
            double delta_x = vx * cos(theta) * dt;
            double delta_y = vx * sin(theta) * dt;
            double delta_th = vth * dt;

            // 更新位姿
            x += delta_x;
            y += delta_y;
            theta += delta_th;

            printf("vx: %0.3f, vth:%0.3f, x: %0.3f, y: %0.3f, theta: %0.3f \n", vx, vth, x, y, theta);

            // 发布tf变换：从odom坐标系到base_link坐标系
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
            odom_broadcaster->sendTransform(odom_trans);

            // 发布Odometry消息
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;
            odom_pub.publish(odom);

            last_time = current_time;
        }
    }

private:
    ros::Subscriber left_wheel_sub, right_wheel_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster* odom_broadcaster;

    double d1 = 0.1;
    double d2 = 0.1;
    double l=0.394;  // 车轮参数
    double x, y, theta; // 位姿
    double vx, vy, vth; // 速度
    double left_speed, right_speed; // 当前轮速
    double last_left_speed, last_right_speed; // 上一次的轮速
    ros::Time last_time;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "diff_drive_odometry");
    OdometryCalculator odometry_calculator;
    ros::Rate loop_rate(50); // 10Hz

    while (ros::ok()) {
        odometry_calculator.updateOdometry();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}