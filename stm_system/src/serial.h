#ifndef SERIAL_H_
#define SERIAL_H_

#include"mbed.h"
#define HEAD_BYTE 0xFF
#define STX 0x02
DigitalOut led(LED1);
void serialSend(float send_data, Serial &obj){ 
  uint8_t checksum_send = 0;
  unsigned char data_h_h{};
  unsigned char data_h_l{};
  unsigned char data_l_h{};
  unsigned char data_l_l{};
  uint32_t byte_divide{};
  memcpy(&byte_divide, &send_data, 4);
  data_h_h = (byte_divide >> 24) & 0xFF;
  data_h_l = (byte_divide >> 16) & 0xFF;
  data_l_h = (byte_divide >> 8) & 0xFF;
  data_l_l = (byte_divide >> 0) & 0xFF;
  unsigned char sendFormat[5] = {0, data_h_h, data_h_l, data_l_h, data_l_l};
  //send head byte
  obj.putc(HEAD_BYTE);
  checksum_send += HEAD_BYTE;
  obj.putc(STX);
  checksum_send += STX;
  for(int k = 0; k < 5; ++k){
    obj.putc(sendFormat[k]);
    checksum_send += sendFormat[k];
  }
  //send checksum
  obj.putc(checksum_send);
}
void serialReceive(float *receive_result, Serial &obj){
  led = 1;
  uint8_t got_data{};
  uint8_t checksum_receive{};
  uint8_t receive_data[5];
  unsigned char receiveFormat[4][5] = {
    {0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0},
    {2, 0, 0, 0, 0},
    {3, 0, 0, 0, 0}
  };
  got_data = static_cast<uint8_t>(obj.getc());
  if(got_data == HEAD_BYTE){
    got_data = static_cast<uint8_t>(obj.getc());
    if(got_data == STX){
      checksum_receive += HEAD_BYTE;
      checksum_receive += STX;
      for(int k = 0; k < 4; ++k){
        for(int i = 0; i < 5; ++i){
          receive_data[i] = static_cast<uint8_t>(obj.getc());
          checksum_receive += receive_data[i];
          receiveFormat[receive_data[0]][i] = receive_data[i];
        }
      }
      got_data = static_cast<uint8_t>(obj.getc());
      if(got_data == checksum_receive){
        int32_t result[4];
        for(int i = 0; i < 4; ++i){
          //receiveFormat[i][0]はidである
          result[i] = static_cast<int32_t>((receiveFormat[i][1] << 24 & 0xFF000000)
                                           | (receiveFormat[i][2] << 16 & 0x00FF0000)
                                           | (receiveFormat[i][3] <<  8 & 0x0000FF00)
                                           | (receiveFormat[i][4] <<  0 & 0x000000FF)
          );
          memcpy(&receive_result[i], &result[i], 4);
        }
      }
    }
  }
}

#endif