#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include<serial/serial.h>

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <tf/tf.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <cstdlib>
#include <sstream>
using namespace std;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"robot_gps");
    ros::NodeHandle nh;
    ros::Publisher gps_pub= nh.advertise<sensor_msgs::NavSatFix>("/robot_gps",5);
    ros::Rate loop_rate(50);

            
    serial::Serial ser;
    ser.setPort("/dev/ttyUSB2");
    ser.setBaudrate(115200);
    serial::Timeout delay=serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(delay);
    try{   
        ser.open();
    }catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to the port(Authority)");
        return -1;
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        ROS_ERROR_STREAM("Failure to open port(Authority)");
        return -1;
    }
    string log_str="log loglist\r\n";
    string result;

    cout<<"关闭自动输出"<<endl;
    string auto_str="unlog gpgga\r\n";
    ser.write(auto_str);
    result=ser.readline();
    cout<<"反馈结果："<<endl;
    cout<<result<<endl;


    //uint8_t code[13]={0x6c,0x6f,0x67,0x20,0x6c,0x6f,0x67,0x6c,0x69,0x73,0x74,0x0D,0x0a};
    ser.write(log_str);
    
    result=ser.readline();
    //result=ser.read(100);
    cout<<"反馈结果：";
    cout<<result<<endl;

    auto_str="log gpgga ontime 0.5\r\n";
    ser.write(auto_str);
    //result=ser.read(100);
    result=ser.readline();
    cout<<"反馈结果：";
    cout<<result<<endl;

    for(int i=0;i<10;i++)
    {
        result=ser.readline();
        cout<<"反馈结果：";
        cout<<result<<"  前处理"<<endl; 
    }


    double latitude,longitude, altitude;
    vector<string> raw_s;
    cout.precision(12);

    while(true){

        latitude=0.0;
        longitude=0.0;
        altitude=0.0;
        raw_s.clear();
        
        result=ser.readline();
        cout<<"反馈结果：";
        cout<<result<<result.size()<<endl;
        if(result.size()<85){           
            cout<<"无效数据"<<endl;
            latitude=0.0;
            longitude=0.0;
            altitude=0.0;
        }
        else{
            cout<<"有效数据，进行处理"<<endl;
            //string result="$GPGGA,013531.00,3150.93478694,N,11717.60132940,E,1,12,3.3,-15.3488,M,-4.4011,M,,*41";
            boost::split(raw_s,result,boost::is_any_of(","));
            //cout<<"分割得到latitude:"<<raw_s[2]<<" longitude:"<<raw_s[4]<<"  altitude:"<<raw_s[9]<<endl;

            vector<string> change_s;
            boost::split(change_s,raw_s[2],boost::is_any_of("."));
            latitude=atoi(change_s[0].c_str())/100+(atoi(change_s[0].c_str())%100+atoi(change_s[1].c_str())/100000000.0)/60.0;
            

            boost::split(change_s,raw_s[4],boost::is_any_of("."));
            longitude=atoi(change_s[0].c_str())/100+(atoi(change_s[0].c_str())%100+atoi(change_s[1].c_str())/100000000.0)/60.0;
            
            altitude=atof(raw_s[9].c_str());
        }
        
        sensor_msgs::NavSatFix gps_data;
        gps_data.header.stamp=ros::Time::now();
        gps_data.header.frame_id="base_link";
        gps_data.latitude=latitude;
        gps_data.longitude=longitude;
        gps_data.altitude=altitude;
        
        gps_pub.publish(gps_data);

        cout<<"latitude"<<latitude<<endl;
        cout<<"longitude"<<longitude<<endl;
        cout<<"altitude"<<altitude<<endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
