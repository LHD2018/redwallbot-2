#ifndef MYSERIAL_H
#define MYSERIAL_H

#include <stdio.h>    
#include <stdlib.h>
#include <unistd.h>   
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <iostream>

using namespace std;

class MySerial{

private:
    int m_serial_fd;        // 串口fd
    struct termios option;  // 串口配置参数
    int writen_len;         // 返回写入的数据大小

    int m_ret;

public:
    MySerial();

    // 初始化串口
    bool initSerial(const char *dev);
    // 向串口写数据
    bool writeSerial(const char *buff, int buff_len);
    // 从串口读数据
    bool readSerial(char *buff, const size_t n, int &read_len);
    // 关闭串口
    void closeSerial();

    ~MySerial();
};

#endif