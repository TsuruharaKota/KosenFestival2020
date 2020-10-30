#ifndef MD
#define MD
#include"mbed.h"

typedef struct{
    PinName pin_A;
    PinName pin_B;
    PinName pin_PWM;
    PinName pin_CS;
    PinName pin_EN;
}MotorDriverPin;

class MotorDriver{
    public:
        MotorDriver(PinName pin_pwm_forward_, PinName pin_pwm_reverse_) : _pin_pwm_forward(pin_pwm_forward_), _pin_pwm_reverse(pin_pwm_reverse_){
        }
        void setPwm(int pwm);
    private:
        DigitalOut _pin_pwm_forward;
        DigitalOut _pin_pwm_reverse;
};
#endif
