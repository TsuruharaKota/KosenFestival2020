#include<mbed.h>
#include"amt_encoder.h"
#include"motor_driver.h"
#include"serial.h"
class PID{
  public:
    PID(const float Kp_, const float Ki_, const float Kd_, const float FREQ_) : _Kp(Kp_), _Ki(Ki_), _Kd(Kd_), _FREQ(FREQ_){}
    void PidUpdate(float goal_, float now_, float prev_){
    	_goal_value = goal_;
    	_now_value = now_;
    	_prev_value = prev_;
    	Defferential();
    	Integral();
    	Calcurate();
    }
    float Get(){
      return _answer_value;
    }
    const float _Kp;
    const float _Ki;
    const float _Kd;
    const float _FREQ;
  private:
    void Defferential(){
    	_defferential_value = (_now_value - _prev_value) / (1 / _FREQ);
    }
    void Integral(){
    	_integral_value += _now_value * (1 / _FREQ);
    }
    void Calcurate(){
    //	cout << goal_value << ":" << now_value << endl;
    	_answer_value = _Kp * (_goal_value - _now_value);
                 		- _Kd * _defferential_value;
    }

		float _goal_value = 0;
		float _now_value = 0;
		float _prev_value = 0;
		float _answer_value = 0;
		float _integral_value = 0;
		float _defferential_value = 0;
};
int pwmConv(float speed){
  return static_cast<int>(speed * 200);
}
Serial pc(USBTX, USBRX, 9600); // tx, rx

int main() {
  //float send_data[9] = {};
  float receive_data[4]{};
  int counter{};
  RotaryInc encoder[4]{
    RotaryInc(PC_11, PC_10, 2 * 50.8 * M_PI, 200),
    RotaryInc(PA_13, PC_4, 2 * 50.8 * M_PI, 200),
    RotaryInc(PA_15, PA_14, 2 * 50.8 * M_PI, 200),
    RotaryInc(PC_3, PC_2, 2 * 50.8 * M_PI, 200),
  };
  MotorDriver motor[4]{
    MotorDriver(PB_1, PA_11),
    MotorDriver(PB_13, PB_14),
    MotorDriver(PB_4, PB_5),
    MotorDriver(PC_8, PC_9)
  };
  PID motor_pid[4]{
    PID(1, 1, 1, 100),
    PID(1, 1, 1, 100),
    PID(1, 1, 1, 100),
    PID(1, 1, 1, 100)
  };
  float speed_now[4]{};
  float speed_prev[4]{};
  float speed_output[4]{};
  while(1) {
    //serialReceive(receive_data, pc);
    receive_data[0] = 100;
    receive_data[1] = 100;
    receive_data[2] = 100;
    receive_data[3] = 100;
    for(int i = 0; i < 4; ++i){
      /*speed_now[i] = encoder[i].getSpeed();
      motor_pid[i].PidUpdate(receive_data[i], speed_now[i], speed_prev[i]);
      speed_output[i] = motor_pid[i].Get();
      motor[i].setPwm(pwmConv(speed_output[i]));
      speed_prev[i] = speed_now[i];*/
      motor[i].setPwm(100);
    }
  }
}
