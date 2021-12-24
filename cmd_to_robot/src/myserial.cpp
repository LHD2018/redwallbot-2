#include "myserial.h"

MySerial::MySerial(){
    writen_len = 0;
    m_serial_fd = -1;
    m_ret = 0;
    bzero(&option, sizeof(struct termios));
}

// 初始化串口
bool MySerial::initSerial(const char *dev){
    closeSerial();
    if((m_serial_fd = open(dev, O_RDWR | O_NOCTTY  | O_NONBLOCK))<0){
        perror("ERR: can't open serial port!\n");
        return false;
    }
     tcgetattr(m_serial_fd, &option); // 配置UART
    // 19200波特率 | 8位 | 使能接收器 |  no modem control lines
    option.c_cflag = B19200 | CS8 | CREAD | CLOCAL;
    // ignore partity errors, CR -> newline
    option.c_iflag = IGNPAR | ICRNL;
    //turn off software stream control
    option.c_iflag &= ~(IXON | IXOFF | IXANY);
    //关闭回显功能,关闭经典输入 改用原始输入
    option.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 
    tcflush(m_serial_fd,TCIOFLUSH);        // 清理输入输出缓冲区
    tcsetattr(m_serial_fd, TCSANOW, &option); // changes occur immmediately
    return true;

}
// 向串口写数据
bool MySerial::writeSerial(const char *buff, int buff_len){
    tcflush(m_serial_fd, TCOFLUSH);
    if(m_serial_fd < 0){
        perror("ERR: serial not open");
        return false;
    }
    fd_set wset;
    struct timeval time_out;
    FD_ZERO(&wset);
    time_out.tv_sec = 3;
    time_out.tv_usec = 0;
    FD_SET(m_serial_fd, &wset);

    if((m_ret = select(m_serial_fd + 1, 0, &wset, 0,  &time_out)) <= 0){
        perror("ERR:write serial timeout");
        return false;
    }
    if((writen_len = write(m_serial_fd, buff, buff_len)) < 0){
        perror("ERR:write serial faild");
        return false;
    }
    //cout << "writen_len:" << writen_len << endl;
    return true;
}
// 从串口读数据
bool MySerial::readSerial(char *buff, const size_t n, int &read_len){
    
    if(m_serial_fd < 0){
        perror("ERR: serial not open");
        return false;
    }
    fd_set rset;
    struct timeval time_out;
    FD_ZERO(&rset);
    time_out.tv_sec = 3;
    time_out.tv_usec = 0;
    FD_SET(m_serial_fd, &rset);

    if((m_ret = select(m_serial_fd + 1, &rset, 0, 0, &time_out))  <= 0){
        perror("ERR:read serial timeout");
        return true;
    }
    if((read_len = read(m_serial_fd, buff, n)) < 0){
        perror("ERR:read serial faild");
        return false;
    }
    tcflush(m_serial_fd, TCIFLUSH);
    return true;

}
// 关闭串口
void MySerial::closeSerial(){
    if(m_serial_fd > 0){
        close(m_serial_fd);
        m_serial_fd = -1;
    }
}

MySerial::~MySerial(){
    closeSerial();
}