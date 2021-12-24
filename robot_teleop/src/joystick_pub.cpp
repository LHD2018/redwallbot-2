#include "myjoystick.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

bool joy_flag;

double speed[3];

void* joyThread(void *);

pthread_mutex_t s_mutex;

int main(int argc, char **argv){

    geometry_msgs::Twist msg;
    
    joy_flag = true;

    memset(speed, 0, sizeof(speed));

    pthread_t pth_joy;
    pthread_create(&pth_joy,  NULL, joyThread, NULL);
    pthread_detach(pth_joy);

    ros::init(argc, argv, "joystick_pub");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",  10);
    ros::Rate loop_rate(100);

    ros::param::set("robot_state",  1);

    while(ros::ok()){
        pthread_mutex_lock(&s_mutex);
        msg.linear.x = speed[0];
        msg.linear.y = speed[1];
        msg.angular.z = speed[2];
        pthread_mutex_unlock(&s_mutex);
        if(msg.linear.x >=0){
            ros::param::set("camera_tag", 1);
        }else{
            ros::param::set("camera_tag", -1);
        }
        
        // ROS_INFO(" linear.x  linear.y angular.z : %f  %f   %f  ",msg.linear.x, msg.linear.y, msg.angular.z);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::param::set("robot_state",  0);
    joy_flag = false;
    usleep(500);
    pthread_mutex_destroy(&s_mutex);
    return 0;
}


void* joyThread(void *){

    MyJoyStick joy;
    if(!joy.initJostick()){
        cout << "init joystick faild" << endl;
        return 0;
    }

    while(joy_flag){
         joy.listenJs();
         pthread_mutex_lock(&s_mutex);
        joy.getSpeed(speed);
        pthread_mutex_unlock(&s_mutex);
    }
    return 0;
}

