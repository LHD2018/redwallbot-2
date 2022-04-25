#ifndef MOTOR_H
#define MOTOR_H

#include "myserial.h"
#include <string>
#include <cmath>
#include <vector>

// 轮子回零偏差
#define RIGHT_FRONT_BIAS 2100           //右前轮
#define LEFT_FRONT_BIAS 0            // 左前轮
#define LEFT_BACK_BIAS 0              // 左后轮
#define RIGHT_BACK_BIAS -1000         //右后轮


// 轮子类型
#define DRIVE_MOTOR 0       // 驱动轮
#define ROTATE_MOTOR 1      // 旋转轮

// 编码器线数
#define MAIN_ENCODER_NUM 2000
#define AUX_ENCODER_NUM 2500

// 轮子直径
#define WHELL_DIA 0.155
// 减速比
#define REDUCER_RATIO 50


// 编码器信息
struct  Coder
{
    int vx;             // 主反馈速度
    int px;             // 主反馈位置

    int vy;             // 副编码器速度
    int py;             // 副编码器位置

    int home_px;    // 零点时的px
};


class Motor{

private:

    MySerial m_serial;      // 串口
    string m_serial_dev;    // 串口名
    int m_motor_type;       // 轮子类型  
    Coder m_coder;          // 编码器

    int m_pv;               // 发送的jv或pa

public:
    Motor(string serial_dev, int motor_type);

    // 初始化电机
    bool initMotor();
    // 回零点
    bool homing(int bias=0);
    // 写入速度或角度
    bool writeVorP(float vp, float p=0);
    // 读取编码器
    bool readCoder();
    // 停止电机
    bool stopMotor();

    int getVX();
    int getPX();

    int getVY();
    int getPY();

    int getHomePX();

    int getMotorType();
    
    ~Motor();

};


#endif