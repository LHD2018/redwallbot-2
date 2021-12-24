
#include "tcpclient.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>

// 控制机器人的参数
struct ControlParams{
    bool type_flag;     // 客户端标识符（true为控制器发送的ControlParams. false>为机器人发送的StateParams）
    int camera_tag;     // 相机流标识符（-1为后置相机，0为不传输相机流，1为前置>相机）
    int robot_model;    // 机器人控制模式（0为正常模式，3为单独轮转）
        float x_speed;      // x反向速度
        float y_speed;      // y方向速度
        float w_speed;      // w速度

};


// 机器人的当前状态参数
struct StatusParams{
    bool type_flag;     // 客户端标识符（true为控制器发送的ControlParams. false>为机器人发送的StateParams）
    int robot_state;    // 机器人状态（0为离线，1为本地控制，2为远程控制）
        struct {
        double lon;
        double lat;
    } gps_data;         // GPS信息（lon:经度， lat：纬度）
};

const char *server_ip = "81.70.196.142";
const int server_port = 10127;

struct ControlParams c_params;

struct StatusParams s_params;

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    
   s_params.gps_data.lon = msg->longitude;
   s_params.gps_data.lat = msg->latitude; 
}


int main(int argc, char **argv){

    geometry_msgs::Twist c_msg;

    memset(&c_params, 0, sizeof(struct ControlParams));
    memset(&s_params, 0, sizeof(struct StatusParams));
    memset(&c_msg, 0, sizeof(geometry_msgs::Twist));

    TcpClient tcp_client;
    
    //ros
    ros::init(argc, argv, "control_socket");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("/robot_gps", 1, callback);
    ros::Rate loop_rate(100);

    while(ros::ok()){
        if(tcp_client.connectToServer(server_ip, server_port) ) break;
        sleep(5);
    }

     int robot_state = -1;
    while(ros::ok()){
        if(!ros::param::get("robot_state", robot_state)){
            ros::param::set("robot_state",  2);
            robot_state = 2;
        }

        s_params.robot_state = robot_state;
        s_params.type_flag = false;
        if(!tcp_client.tcpSend(tcp_client.m_sockfd, (char*)&s_params, sizeof(struct StatusParams))){
            ROS_ERROR("socket send dada faild");
        }
        if(robot_state != 1){
            char recv_buffer[512] = {0};
            if(!tcp_client.tcpRecv(tcp_client.m_sockfd, recv_buffer, NULL, 10)){
                ROS_ERROR("socket recv data faild!");
            }
            memcpy(&c_params, recv_buffer, sizeof(struct ControlParams));
            c_msg.linear.x = c_params.x_speed;
            c_msg.linear.y = c_params.y_speed;
            c_msg.angular.z = c_params.w_speed;
            ros::param::set("camera_tag", c_params.camera_tag);
            ros::param::set("robot_model", c_params.robot_model);
            pub.publish(c_msg);

        }
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
