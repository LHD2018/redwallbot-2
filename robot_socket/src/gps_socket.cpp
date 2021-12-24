#include "udpserver.h"

#include <cmath>
#include <iomanip>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>


const int server_port = 8001;
sensor_msgs::NavSatFix gps_msg;

double str2Num(string s);

int main(int argc, char **argv){

    UdpServer udp_server;

    memset(&gps_msg, 0, sizeof(sensor_msgs::NavSatFix));

    ros::init(argc, argv, "GPS_socket");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::NavSatFix>("/robot_gps", 10);
    ros::Rate loop_rate(100);

    while (ros::ok()){
        if(udp_server.initServer(server_port)) break;
        sleep(5);
    }
    

    while(ros::ok()){
        char buff[64] = {0};
        udp_server.udpRecv(buff, sizeof(buff));
        
        //cout << buff << endl;

        vector<string> strs;
        boost::split( strs, buff, boost::is_any_of( "," ), boost::token_compress_on );
        string lat_s, lon_s;
        for(vector<string> :: iterator it = strs.begin(); it != strs.end();it++){
            
            if((*it) == "N"){
                lat_s = *(it - 1);
            }
            if((*it) == "E"){
                lon_s = *(it - 1);
                break;
            }	
        }
        if((!lat_s .empty()) && (!lon_s.empty())){
            gps_msg.latitude = str2Num(lat_s);
            gps_msg.longitude = str2Num(lon_s);
            //cout << "lat_s:" << gps_msg.latitude << "lon_s:" << gps_msg.longitude<< endl;

            pub.publish(gps_msg);
        }
       
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

double str2Num(string s){
    vector<string> tmp;
	boost::split( tmp, s, boost::is_any_of( "." ), boost::token_compress_on );

	return atoi(tmp[0].c_str()) + atoi(tmp[1].c_str()) * pow(0.1, tmp[1].size());
	 
}
