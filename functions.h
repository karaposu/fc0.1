#ifndef FUNCTIONS_H_ 
#define FUNCTIONS_H_ 

#include "main.h" 
char mpu_initialise();
s8	hal_i2c_write( u8 dev_addr,u8  reg_addr,u8  *data,   u8  length);
void process_gyro();
s8 hal_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len) ;
void hal_i2c_delay(u32 a);
uint16_t TF_RX_Proc(uint8_t *buf, uint32_t len);

#endif
							 

