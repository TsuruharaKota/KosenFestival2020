#include<mbed.h>
#include"amt_encoder.h"
#include"motor_driver.h"
#include"my_serial.h"

int main(){
  Enc motor_enc[4] = {Enc(D14, D15), Enc(D13, D12), Enc(D11, D10), Enc(D18, D19)};
  Enc  odom_enc[4] = {Enc(D14, D15), Enc(D13, D12), Enc(D11, D10), Enc(D18, D19)};
  MD motor[4] = {MD(), MD(), MD(), MD()};
  MySerial pc(PB_8, PB_9);
  
  float ref_speed[4];

  while(1){
    pc.receive(ref_speed);
    for(int i = 0; i < motor.size(); ++i){
      motor.output(ref_speed[i], motor_enc[i].getSpeed());
    }
  }
}