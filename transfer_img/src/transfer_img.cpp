
#include "myrtmp.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

int width;
int height;

cv::Mat image;

const double robot_width = 0.54 / 0.05;     // 0.05 是设置的雷达分辨率
const double robot_length = 0.74 / 0.05;

cv_bridge::CvImagePtr cv_ptr;

MyRTMP *rtmp;

int img_tag;

void callback(const sensor_msgs::Image::ConstPtr& msg){
    
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_brige exception : %s", e.what() );
        return;
    }
    if(img_tag == 1){
        cv::Rect rect((cv_ptr->image.cols - robot_width) / 2, (cv_ptr->image.rows - robot_length) / 2, robot_width, robot_length);
        cv::rectangle(cv_ptr->image, rect, cv::Scalar(255, 0, 0), -1);
    }
    cv::resize(cv_ptr->image, image, cv::Size(width, height));

    
    //cv::imshow("test", image);
    //cv::waitKey(30);
    rtmp->startRTMP(image.data, image.elemSize());
}


int main(int argc, char **argv){

    string  arg = argv[1];
    img_tag = -1;

    string node_name;
    string  img_topic;
    string rtmp_addr;
    if(arg == "camera"){
        img_tag = 0;
        width = 1280;
        height = 720;
        node_name = "transfer_camera";
        img_topic = "/camear_img";
        rtmp_addr = "rtmp://81.70.196.142:8090/live";
    }else if(arg == "lidar"){
        img_tag = 1;
        width = 400;
        height = 400;
        node_name = "transfer_lidar";
        img_topic = "/lidar_img";
        rtmp_addr = "rtmp://81.70.196.142:8091/live";
    }else{
        return -1;
    }
    
    //ros
    ros::init(argc, argv, node_name);

    while(ros::ok()){
        rtmp = new MyRTMP(width, height, 3, 24);
        if(rtmp->initRTMP(rtmp_addr.c_str())) break;

        delete rtmp;
        rtmp = NULL;
        sleep(5);
    }

    ros::NodeHandle nh;
     image_transport::ImageTransport image_transport(nh);
     image_transport::Subscriber sub = image_transport.subscribe(img_topic, 1, &callback);
    ros::spin();
    
    delete rtmp;
    rtmp = NULL;
    return 0;
}