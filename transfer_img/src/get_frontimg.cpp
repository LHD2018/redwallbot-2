
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


ros::Publisher pub;                     // 话题发布者

void callback(const sensor_msgs::Image::ConstPtr& msg){
    int camera_tag;
    ros::param::get("camera_tag", camera_tag);
    
    if(camera_tag == 1){
        pub.publish(msg);
    }
    
}


int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "get_frontimg");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::Image>("/camear_img", 10);
    ros::Subscriber sub = nh.subscribe("/zedm/zed_node/rgb/image_rect_color", 1, callback);
    ros::spin();

    return 0;
}