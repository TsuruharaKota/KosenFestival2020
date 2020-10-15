#pragma once

template<typename T>
class PID{
  public:
    PID(T _p_gain, T _i_gain, T _d_gain, T _freq):p_gain_(_p_gain), i_gain_(_i_gain), d_gain_(_d_gain), freq_(_freq){}
    ~PID(){}
    T operator()(T _ref_val, T _now_val, T _prev_val){
      //PDのみ
      return p_gain_ * (_ref_val - _now_val) - d_gain_ * (_now_val - _prev_val) * (1 / freq_);
    }
  private:
    const T p_gain_;
    const T i_gain_;
    const T d_gain_;
    const T freq_;
};
