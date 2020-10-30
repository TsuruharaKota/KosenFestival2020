#include"motor_driver.h"
void MotorDriver::setPwm(int pwm){
    if(pwm > 0){
        _pin_pwm_forward = (float)pwm/255;
        _pin_pwm_reverse = 0;
    }else{
        _pin_pwm_forward = 0;
        _pin_pwm_reverse = (float)pwm/-255;
    }
};
