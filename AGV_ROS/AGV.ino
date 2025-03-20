/*
 * Serial_node.py /dev/ttyACM0
 * AGV + Stabil dual rosserial test1
 * DC모터 구동과 yaw방향제어 및 ebimu센서값(roll, pitch)와 theta Stabil노드로 pub 
 */
/////////////////////////// 받은 속도값으로 모터 제어+엔코더 값을 읽어 퍼블리시하는 node
#include <ros.h>
#include<std_msgs/Int16.h>
#include <geometry_msgs/PointStamped.h>
#include<ros/time.h>
#include <Wire.h> 

const float pi = 3.141592;              // 파이
const float g = 9.81;                   // 중력가속도
// 프로파일 계산 함수 전역변수 선언
float  p_time; float  p_velocity;float  p_length;float  p_alpha;float  p_beta;float  p_accel;    

float a;                                // accel 현재 가속도
float theta;                            // 보정각도 프로파일 세타

unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임

char  CMD;

#define SBUF_SIZE 64

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

// 역기구학 통과한 모터1,2,3 각도
float theta_x , theta_y, theta_z;   

float ang_x;float ang_y;float ang_z;float accel_x;float accel_y;float accel_z;

//pitch, roll, yaw
float angle_x;float angle_y;float angle_z;                         

float the_x, the_y, the_z;                     // 보정각도 프로파일 세타

unsigned long loop_dt;                       // 1 루프 타임
unsigned long sttime;

float Gimbal_Roll;float Gimbal_Pitch;

float RPM;
float av_dt = 0;

float distance = 0.365/2;
float RPM_yaw;
float target_yaw;

const float ratio =    17;

ros::NodeHandle  nh;

float roll,pitch;
// cmd_vel 콜백 함수(Key_profile.py노드에서 받은 data로 agv dc모터 제어)
void AGVcontrol_cmd (const std_msgs :: Int16& cmd_vel) {
  //publish_imu_accel();
  target_yaw = angle_z;
  ///////////////////////////                                                           
    loop_start_time = millis();
    ///////////////////////////
    sttime          = millis(); 
  theta=0;
  //ros 명령 받으면 1ms속도 시나리오 
  int key=cmd_vel.data;//명령 받은 키
  if (key ==1 )
  {
    Agv_accel(10,1.2);
    //Agv(2);
    Agv_decel(10,1.2);
  }
  else if ( key==3) //turn 90도 좌회전 
  {
    theta=0;
    Serial3.println("co1=1");Serial3.println("co2=1");
    Serial3.println("mvc=-50,-50");
    delay(1900);
    Serial3.println("co1=0");Serial3.println("co2=0");
  }
  else if ( key==6) //turn 180도 유턴 
  {
    theta=0;
    Serial3.println("co1=1");Serial3.println("co2=1");
    Serial3.println("mvc=70,70");
    delay(1920);
    Serial3.println("co1=0");Serial3.println("co2=0");
  }
  else if ( key==5) //turn 90도 우회전 
  {
    theta=0;
    Serial3.println("co1=1");Serial3.println("co2=1");
    Serial3.println("mvc=50,50");
    delay(1900);
    Serial3.println("co1=0");Serial3.println("co2=0");
  }
  else if ( key==2) //a
  {
    Agv_accel(3,0.2);
    Agv_decel(3,0.2);
  }
  else if ( key==7)
  {
    Agv_turn_right(1,0.1);//우회전 
  }
  else if ( key==8)
  {
    Agv_turn_left(1,0.1);//좌회전 
  }
    
  else{
    Serial3.println("co1=0");Serial3.println("co2=0");
  }
    
  timecycle(); // -주기 계산 함수
}

//---------------------------------------------------------------------------------------------
// subscriber ( Key_profile.py 노드에서 key명령받음)
ros::Subscriber<std_msgs::Int16> cmd_vel("cmd_vel", AGVcontrol_cmd);

//publisher (roll, pitch값을 /imu토픽으로 pub)
geometry_msgs::PointStamped imu;
ros::Publisher imu_pub("imu", &imu);

void setup() {
  Serial.begin(57600);
  Serial2.begin(57600);
  Serial3.begin(57600);
  Serial3.println("co1=0");Serial3.println("co2=0");
  Wire.begin();
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(imu_pub);
}

void loop() {
  nh.spinOnce();//rosserial을 통해 계속해서 키보드 값을 받는다.
  publish_imu_accel();
  ///imu_pub.publish(&imu);
}


//----------------------------------------------------------------------------------------------

//ebimu roll, pitch 값을 serial_node2로 publish
void publish_imu_accel(){
  float euler[3];
  
  if(EBimuAsciiParser(euler, 3))
  {
     //imu.point.x=euler[0];//Roll
     //imu.point.y=-euler[1];//Pitch
     //imu.point.y=0;//Pitch
     angle_z=euler[2];//Yaw
     imu.point.y=angle_z;
     //imu.point.z=euler[4]*9.81;//ay
     imu.point.z=theta; //Stabil 로 pub할 theta값
     imu.header.stamp=nh.now();
     imu_pub.publish(&imu);
     
  }
  
}


int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial2.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial2.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;
    }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}

void Agv_turn_left(float highest_theta, float Length){
  
  float Vel;
  float Accel;
   float cmd_rpm_1;                   //모터에 부여할 RPM
  float cmd_rpm_2;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어

  Serial3.println("co1=1");Serial3.println("co2=1");
 
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec=0; sec < p_time;  sec += 0.02){
    cycle_start_time = millis();
    theta=0;
     //RPM_yaw   = (target_yaw - angle_z) * distance * ratio /PI/0.095;
        
    
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  0;    
    RPM     =  Vel*60/(3.14*0.095);
    
    cmd_rpm_1 =  RPM;
    cmd_rpm_2 =  RPM;
    
    cmd_str =  "mvc="+String(-cmd_rpm_1)+","+String(-cmd_rpm_2);
    Serial3.println(cmd_str);
    Calculate_delay( p_time*1000,  p_time*50 );
  }
    for(float sec= p_time ; sec > 0;  sec -= 0.02){
      cycle_start_time = millis();
 
     
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    RPM     =  Vel*60/(3.14*0.095);
    
    cmd_rpm_1 =  RPM;
    cmd_rpm_2 =  RPM;
    
    cmd_str =  "mvc="+String(-cmd_rpm_1)+","+String(-cmd_rpm_2);
    Serial3.println(cmd_str);
                                       //모터에명령통신

      Calculate_delay( p_time*1000,  p_time*50 );
    }
    Serial3.println("mvc=0,0");
}
void Agv_turn_right(float highest_theta, float Length){
  
  float Vel;
  float Accel;
  float cmd_rpm_1;                   //모터에 부여할 RPM
  float cmd_rpm_2;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어

  Serial3.println("co1=1");Serial3.println("co2=1");
 
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec=0; sec < p_time;  sec += 0.02){
    cycle_start_time = millis();
       theta=0;
       //publish_imu_accel();
       
     RPM_yaw   = (target_yaw - angle_z) * distance * ratio /PI/0.095;
        
    theta   =  0;  
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도  
    RPM     =  Vel*60/(3.14*0.095);
    cmd_rpm_1 =  RPM;
    cmd_rpm_2 =  RPM;
    
    cmd_str =  "mvc="+String(cmd_rpm_1)+","+String(cmd_rpm_2);
    Serial3.println(cmd_str);                                     //모터에명령통신
    
          
    
    Calculate_delay( p_time*1000,  p_time*50 );
  }
    for(float sec= p_time ; sec > 0;  sec -= 0.02){
      cycle_start_time = millis();
      
    theta   =  0;  
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도  
    RPM     =  Vel*60/(3.14*0.095);
    cmd_rpm_1 =  RPM;
    cmd_rpm_2 =  RPM;
    
    cmd_str =  "mvc="+String(cmd_rpm_1)+","+String(cmd_rpm_2);
    Serial3.println(cmd_str);                                     //모터에명령통신
    
        
      Calculate_delay( p_time*1000,  p_time*50 );
  }
  Serial3.println("mvc=0,0");
}


void Agv_accel( float highest_theta, float Length){

  float Vel;
  float Accel;
   float cmd_rpm_1;                   //모터에 부여할 RPM
  float cmd_rpm_2;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어

  Serial3.println("co1=1");Serial3.println("co2=1");
 
  //theta=0;
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  for(float sec=0; sec < p_time;  sec += 0.02){
    cycle_start_time = millis();

    //roll,pitch,theta값 pub
    publish_imu_accel();
    RPM_yaw   = (target_yaw - angle_z) * distance * ratio /PI/0.095;
        
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    
    RPM     =  Vel*60/(3.14*0.095);
    
    cmd_rpm_1 =  RPM - (RPM_yaw/5);
    cmd_rpm_2 =  RPM + (RPM_yaw/5);
    cmd_str =  "mvc="+String(cmd_rpm_1)+","+String(-cmd_rpm_2);
    Serial3.println(cmd_str);                                     //모터에명령통신

    //theta값 pub
    
    Calculate_delay( p_time*1000,  p_time*50 );
  }
}

void Agv(float Time){
  
   float cmd_rpm_1;                   //모터에 부여할 RPM
  float cmd_rpm_2;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  //theta=0;
  av_dt = Time;
  for(float sec=0; sec < Time;  sec += 0.02){
    cycle_start_time = millis();
theta=0;
    //roll,pitch,theta값 pub
    publish_imu_accel();
    RPM_yaw   = (target_yaw - angle_z) * distance * ratio /PI/0.095;
    
    cmd_rpm_1 =  RPM - (RPM_yaw/5);
    cmd_rpm_2 =  RPM + (RPM_yaw/5);
    cmd_str =  "mvc="+String(cmd_rpm_1)+","+String(-cmd_rpm_2);
    Serial3.println(cmd_str);                                     //모터에명령통신
                                 //모터에명령통신
   
    
    Calculate_delay( p_time*1000,  p_time*50 );
  } 
}

void Agv_decel( float highest_theta, float Length){

  float Vel;
  float Accel;
  float RPM;
   float cmd_rpm_1;                   //모터에 부여할 RPM
  float cmd_rpm_2;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec= p_time ; sec > 0;  sec -= 0.02){
    cycle_start_time = millis();

    //roll,pitch,theta값 pub
    publish_imu_accel();
    RPM_yaw   = (target_yaw - angle_z) * distance * ratio /PI/0.095;
    
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  -atan((Accel/g))*180/3.14  ;    
    RPM     =  Vel*60/(3.14*0.095);
    cmd_rpm_1 =  RPM - (RPM_yaw/5);
    cmd_rpm_2 =  RPM + (RPM_yaw/5);
    cmd_str =  "mvc="+String(cmd_rpm_1)+","+String(-cmd_rpm_2);
    Serial3.println(cmd_str);                                     //모터에명령통신

    
    Calculate_delay( p_time*1000,  p_time*50 );
    
  }
  Serial3.println("mvc=0,0"); 
  av_dt = 0 ;
  theta=0;
}


void profile_maker(float highest_theta, float Length){
  //플레이트 각과 이동거리를 입력하면 최대 가속도, 최대 속도, 걸리는 시간이 나온다
  
  float profile_Time;       //a(t)= 0 이 되는 시각 t
  float t;                  //임시 지역 변수
  float V_length;           //적분된 velocity     = L(t) - L(0) = V(t) - V(0) 
  float A_velocity;         //적분된 acceleration = v(t) - v(0) = A(t) - A(0)
  float highest_accel;      //최대 가속도
  float alpha, beta;        //계수 알파 베타
  float g = 9.81;           //중력 가속도
  float pi = 3.1415;
   
  highest_accel = g*tan((highest_theta*pi)/180);
  
  // (3m/s,   2초) alpha = 0.75 , beta =  2.25
  // (2m/s,   1초) alpha = 4   ,  beta =     6
  // (1m/s, 0.5초) alpha = 16  ,  beta =    12 

  //  V         = -pow(t,4)* (alpha/12)    + pow(t,3)* (beta/6);    // 거리 
  //  A         = -pow(t,3)* (alpha/3)     + pow(t,2)* (beta/2);    // 속도 
  //  a         = -pow(t,2)*  alpha        + pow(t,1)*  beta;       // 가속도
  
  profile_Time  =  sqrt((3*Length)/(highest_accel));
  t             =  profile_Time;
  alpha         =  (4*highest_accel/pow(t,2));
  beta          =  alpha*t;
  V_length      = -pow(t,4)* (alpha/12)  + pow(t,3)* (beta/6);      // 거리 
  A_velocity    = -pow(t,3)* (alpha/ 3)  + pow(t,2)* (beta/2);      // 속도 

  p_accel       =  highest_accel;
  p_time        =  profile_Time;
  p_velocity    =  A_velocity;
  p_length      =  V_length;
  p_alpha       =  alpha;
  p_beta        =  beta;
  
}


void timecycle(){//ㅡㅡㅡㅡㅡㅡㅡㅡ
  float t;                            
  t = millis() - loop_start_time;
  dt = t;
  
  //Serial.print("loop time : ");
  //Serial.print(dt);
  //Serial.println();
                                       
}//timecycle

void Calculate_delay( float target_time, float number ){//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
                                                            //target_time (ms)
                                                            //number (횟수)  
  float cycle_end_time = millis();                            //사이클 끝나는 시각
  float cycle_time;                                           //측정된 1 사이클 시간
  float target_delay;                                         //원하는 딜레이 시간
  
  cycle_time = cycle_end_time - cycle_start_time;
  target_delay = (target_time/number)- cycle_time;   

  
  delay(target_delay);                                          
}//Calculate_delay//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ