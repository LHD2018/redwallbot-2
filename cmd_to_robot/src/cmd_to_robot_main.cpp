
#include "cmd_to_robot/whellparams.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <Eigen/Dense>

ros::Publisher pub;                     // 话题发布者

cmd_to_robot::whellparams w_params;     // 轮子参数

Eigen::Matrix<double,8,3> coff_mat;     // 机器人模型参数矩阵
Eigen::Matrix<double,3,8> coff_mat_inv; // 参数矩阵逆矩阵

//机器人长的一半
const double half_len = 0.268;
//机器人宽的一半
const double half_wid = 0.159;

Eigen::Matrix<double,8,1> p_xyw;        // 模型参数矩阵与Vx， Vy， W向量的乘积


void callback(const geometry_msgs::Twist::ConstPtr& msg){
    memset(&w_params, 0, sizeof(w_params));

    int robot_model ;
    ros::param::get("robot_model", robot_model);

    if(robot_model == 3){
        
             // 处理单轮模式
            w_params.lf_p = msg->angular.z;
            w_params.lb_p= msg->angular.z;
            w_params.rb_p = msg->angular.z;
            w_params.rf_p = msg->angular.z;
            w_params.lf_v = msg->linear.x;
            w_params.lb_v = msg->linear.x;
            w_params.rb_v = msg->linear.x;
            w_params.rf_v = msg->linear.x;

    }else{
        // 处理正常模式
            Eigen::Matrix<double,3,1> xyw;  // 机器人Vx， Vy， W向量
            xyw << msg->linear.x, msg->linear.y, msg->angular.z;

            p_xyw = coff_mat * xyw;

            double _A = pow(p_xyw(0,0),2);
            double _B = pow(p_xyw(1,0),2);
            double _C = pow(p_xyw(4,0),2);
            double _D = pow(p_xyw(3,0),2);

            if(p_xyw(0,0) >= 0) w_params.lf_v = sqrt(_A + _B);
            else w_params.lf_v = -1.0 * sqrt(_A + _B);

            if(w_params.lf_v != 0)
                w_params.lf_p = asin(p_xyw(1,0) / w_params.lf_v);


            if(p_xyw(2,0) >= 0) w_params.lb_v = sqrt(_A + _D);
            else w_params.lb_v = -1.0 * sqrt(_A + _D);

            if(w_params.lb_v != 0)
                w_params.lb_p = asin(p_xyw(3,0) / w_params.lb_v);

            if(p_xyw(4,0) >= 0) w_params.rb_v = sqrt(_C + _D);
            else w_params.rb_v = -1.0 * sqrt(_C + _D);
                
            if(w_params.rb_v != 0)
                w_params.rb_p = asin(p_xyw(3,0) / w_params.rb_v);

            if(p_xyw(6,0) >= 0) w_params.rf_v = sqrt(_C + _B);
            else w_params.rf_v = -1.0 * sqrt(_C + _B);

            if(w_params.rf_v != 0)
                w_params.rf_p = asin(p_xyw(1,0) / w_params.rf_v);
       
    }

   

    // ROS_INFO("w_params.lf_v  w_params.lf_p w_params.rf_v w_params_rf_p : %f  %f   %f  %f", w_params.lf_v, w_params.lf_p, w_params.rf_v, w_params.rf_p);
         
    pub.publish(w_params);
}


int main(int argc, char **argv){

    
    coff_mat << 1,0,-half_wid,
                0,1, half_len,
                1,0,-half_wid,
                0,1,-half_len,
                1,0, half_wid,
                0,1,-half_len,
                1,0, half_wid,
                0,1, half_len;    
    //ros
    ros::init(argc, argv, "cmd_to_robot_main");
    ros::NodeHandle nh;
    pub = nh.advertise<cmd_to_robot::whellparams>("/whell_params", 10);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, callback);
    ros::spin();

    return 0;
}
