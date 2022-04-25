#include "motor.h"

Motor::Motor(string serial_dev, int motor_type){
    m_pv = 0;
    m_serial_dev = serial_dev;
    m_motor_type = motor_type;
    memset(&m_coder, 0, sizeof(struct Coder));
}

bool Motor::initMotor(){
    if(!m_serial.initSerial(m_serial_dev.data())) return false;

    string init_str;
    if(m_motor_type == DRIVE_MOTOR){
        init_str = "ya[4]=2;um=2;mo=1;rm=0;vx=0;px=0;vy=0;py=0;ac=120000;dc=120000;";
    }else{
        
    init_str = "ya[4]=2;um=2;mo=1;rm=0;vx=0;px=0;vy=0;py=0;ac=150000;dc=150000;";        
    }
        return m_serial.writeSerial(init_str.data(), init_str.size());

}

bool Motor::homing(int bias){
    int last_py = 0;

    string homing_str = "jv=10000;px;py;bg;hy[1]=1;hy[2]=0;hy[3]=3;hy[5]=0;hy[8];";

    while (m_coder.home_px > -10 && m_coder.home_px < 10){
        if(m_serial.writeSerial(homing_str.data(), homing_str.size()) == false) return false;
        usleep(100000);
        readCoder();
        if(m_coder.home_px == 0 && m_coder.py - last_py > 10) {
            m_coder.home_px = m_coder.px;
            break;
        }
        last_py = m_coder.py;
    }
    m_coder.home_px += bias;
    homing_str = "sf=80;pa=" + to_string(m_coder.home_px);
    homing_str += "bg;";

    return m_serial.writeSerial(homing_str.data(), homing_str.size());
    
     
}

bool Motor::writeVorP(float vp, float p){

    string v_str;
    if(m_motor_type == DRIVE_MOTOR){

        double jv = MAIN_ENCODER_NUM * vp / (M_PI * WHELL_DIA);
        m_pv = (int)(jv * REDUCER_RATIO);
        if( p!= 0){

            int pr = (int)(MAIN_ENCODER_NUM * p / (2 * M_PI) * REDUCER_RATIO);

            v_str = "sf=80;sp=20000;pr=" + to_string(pr);

            if(fabs(p) > 0.5) {
                int wait_s = p / M_PI_2 * 1000;
                //v_str += ";bg;wait 1000;sf=80;jv=";
                v_str += ";bg;wait ";
                v_str += to_string(wait_s);
                v_str += ";sf=80;jv=";
            }else {
                v_str += ";bg;jv=";
            }
        
            v_str += to_string(m_pv);

        }else{
            v_str = "sf=80;jv=" +to_string(m_pv);
        }
        
          //v_str = "pr=0;bg;jv=" + to_string(m_pv);
    }else{
        double pa = MAIN_ENCODER_NUM * vp / (2 * M_PI);
        m_pv = (int)(pa * REDUCER_RATIO);
        m_pv += m_coder.home_px;
        v_str = "sf=80;sp=10000;pa=" + to_string(m_pv);
       
    }
    
    v_str += ";vx;px;vy;py;bg;";              // 这里读取主反馈数据，如想读取辅助编码器，则使用vy;py;，并修改readCoder函数
    // v_str += ";vx;px;vy;py;bg;hy[1]=1;hy[2]=0;hy[3]=3;hy[5]=0;hy[8];";   // 用于回零测试          
    
    //cout << v_str << endl;
    return m_serial.writeSerial(v_str.data(), v_str.size());
}

bool Motor::readCoder(){
    int read_len = 0;
    char recv_buff[512] = {0};
    
    if(!m_serial.readSerial(recv_buff, sizeof(recv_buff), read_len)) return false;

     //cout << recv_buff << endl;
    
    vector<string> read_vs;
    char tmp_buff[16] = {0};
    int index = 0;
    for(int i = 0;i < read_len;i++){
        if(recv_buff[i] == ';'){
            read_vs.push_back(tmp_buff);
            memset(tmp_buff, 0, sizeof(tmp_buff));
            index = 0;
        }else{
            tmp_buff[index] = recv_buff[i];
            index++;
        }
    }
    for(vector<string> :: iterator it = read_vs.begin(); it != read_vs.end();it++){
        
        if((*it) == "vx"){
            m_coder.vx = atoi((*(++it)).c_str());
        }else if((*it) == "px"){
            m_coder.px = atoi((*(++it)).c_str());
        }else if((*it) == "vy"){
            m_coder.vy = atoi((*(++it)).c_str());
        }else if((*it) == "py"){
            m_coder.py = atoi((*(++it)).c_str());
        }else if((*it) == "hy[8]" && m_coder.home_px >-10 && m_coder.home_px < 10){
            m_coder.home_px = atoi((*(++it)).c_str());
        }
    }
    //cout << "vx:" << m_coder.vx << "\tpx:" << m_coder.px  << endl;
    return true;

}

bool Motor::stopMotor(){
    string stop_str;
    if(m_motor_type == DRIVE_MOTOR){
            stop_str = "jv=0;bg;";
    }else{
            stop_str = "pa=0;bg;";
    }

    return m_serial.writeSerial(stop_str.data(), stop_str.size());
}

int Motor::getVX(){
    return m_coder.vx;
}

int Motor::getPX(){
    return m_coder.px;
}

 int Motor::getVY(){
     return m_coder.vy;
 }
int Motor::getPY(){
    return m_coder.py;
}
int Motor::getHomePX(){
    return m_coder.home_px;
}
int Motor::getMotorType(){
    return m_motor_type;
}

Motor::~Motor(){
    stopMotor();
    sleep(2);
}