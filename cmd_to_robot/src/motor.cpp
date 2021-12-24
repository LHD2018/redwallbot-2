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
        init_str = "um=2;mo=1;rm=0;vx=0;px=0;vy=0;py=0;ac=120000;dc=120000;";
    }else{
        init_str = "um=2;mo=1;rm=0;vx=0;px=0;vy=0;py=0;ac=150000;dc=150000;";
    }
    return m_serial.writeSerial(init_str.data(), init_str.size());
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
        v_str = "sf=80;sp=20000;pa=" + to_string(m_pv);
       
    }
    
    v_str += ";vx;px;bg;";              // 这里读取主反馈数据，如想读取辅助编码器，则使用vy;py;，并修改readCoder函数
    
    //cout << v_str << endl;
    return m_serial.writeSerial(v_str.data(), v_str.size());
}

bool Motor::readCoder(){
    int read_len = 0;
    char recv_buff[512] = {0};
    
    if(!m_serial.readSerial(recv_buff, sizeof(recv_buff), read_len)) return false;
    
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
        }
    }
    // cout << "vx:" << m_coder.vx << "\tpx:" << m_coder.px  << endl;
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

Motor::~Motor(){
    stopMotor();
    sleep(2);
}