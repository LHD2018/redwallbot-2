#include "motor.h"
#include "cmd_to_robot/whellparams.h"
#include <ros/ros.h>
#include <fstream>
#include <sys/time.h>

Motor *motor;  

ofstream out_file;

int msg_flag;

float last_p;

struct timeval now_time;

void callback(const cmd_to_robot::whellparams::ConstPtr& msg){

    float vp  = 0.0;
    float p = 0.0;
    switch (msg_flag){
    case 1:
        vp = msg->lf_v;
        p = msg->lf_p - last_p;
        last_p = msg->lf_p;
        break;
    case 2:
        vp = msg->lf_p;
        break;
    case 3:
        vp = msg->lb_v;
        p = msg->lb_p - last_p;
        last_p = msg->lb_p;
        break;
    case 4:
        vp = msg->lb_p;
        break;
    case 5:
        vp = msg->rb_v;
        p = msg->rb_p - last_p;
        last_p = msg->rb_p;
        break;
    case 6:
        vp = msg->rb_p;
        break;
    case 7:
        vp = msg->rf_v;
        p = msg->rf_p  - last_p;
        last_p = msg->rf_p;
        break;
    case 8:
        vp = msg->rf_p;
        break;
    
    default:
        break;
    }


   motor->writeVorP(vp, p);

    gettimeofday(&now_time, NULL);

    motor->readCoder();
    
   usleep(100000);

    out_file << now_time.tv_sec << "\t" << motor->getVX() << "\t" << motor->getPX() << endl;
 

}


int main(int argc, char **argv){

    string  arg = argv[1];

    if(arg == "lfd"){
        motor = new Motor("/dev/usb_lfd", DRIVE_MOTOR );
        out_file.open("/home/redwall/Documents/lfd.txt");
        msg_flag = 1;
    }else if(arg == "lfr"){
        motor = new Motor("/dev/usb_lfr", ROTATE_MOTOR );
        msg_flag = 2;
    }else if(arg == "lbd"){
        motor = new Motor("/dev/usb_lbd", DRIVE_MOTOR );
        out_file.open("/home/redwall/Documents/lbd.txt");
        msg_flag = 3;
    }else if(arg == "lbr"){
        motor = new Motor("/dev/usb_lbr", ROTATE_MOTOR );
        msg_flag = 4;
    }else if(arg == "rbd"){
        motor = new Motor("/dev/usb_rbd", DRIVE_MOTOR );
        out_file.open("/home/redwall/Documents/rbd.txt");
        msg_flag = 5;
    }else if(arg == "rbr"){
        motor = new Motor("/dev/usb_rbr", ROTATE_MOTOR );
        msg_flag = 6;
    }else if(arg == "rfd"){
        motor = new Motor("/dev/usb_rfd", DRIVE_MOTOR );
        out_file.open("/home/redwall/Documents/rfd.txt");
        msg_flag = 7;
    }else if(arg == "rfr"){
        motor = new Motor("/dev/usb_rfr", ROTATE_MOTOR );
        msg_flag = 8;
    }else{
        return -1;
    }

    if(!motor->initMotor()){
        ROS_ERROR("inti motor faild!");
        return -1;
    }
   
    //ros
    ros::init(argc, argv, "cmd_to_robot_motor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("whell_params",  1,  callback);
    ros::spin();

    out_file.close();

    return 0;
}
