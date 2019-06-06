#include "functions.h"

extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2 ; 
 
 
extern int16_t euler_h,euler_p,euler_r;


#define TFMINI_DATA_Len             9
#define TFMINT_DATA_HEAD            0x59


//#define BNO055_I2C_ADDR1                (0x28)
//#define BNO055_I2C_ADDR2                (0x29)
//#define BNO055_CALIB_STAT_ADDR        (0X35)
//#define BNO055_OPR_MODE_ADDR        (0X3D)  //111101
//#define BNO055_OPERATION_MODE_NDOF        (0X0C)

//#define BNO055_I2C_ADDR3                (0x50)
//#define BNO055_I2C_ADDR4                (0x52)
//char device_active=0;

extern uint8_t temp;
extern uint8_t range;
//uint8_t	 RecBuf[8];
	//uint8_t	 is_working[3];
		uint8_t	 is_working_pg[3];
		
		
		 uint16_t packet_arrived=0;
  uint16_t packet_arrived_previous=0;
extern uint8_t counter ,buffrec[5],cout_doku;
extern  uint8_t Rx_indx, Rx_data[2], TBuffer[150],Transfer_cplt ,Rx_Buffer[30] ;
extern  uint8_t un_decoded_data[13];
		
		




uint16_t TF_RX_Proc(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;
    uint8_t chk_cal = 0;
    uint16_t cordist = 0;

    if(TFMINI_DATA_Len == len)
    {
        if((TFMINT_DATA_HEAD == buf[0])&&(TFMINT_DATA_HEAD == buf[1]))
        {
            for(i = 0; i < (TFMINI_DATA_Len - 1); i++)
            {
                chk_cal += buf[i];
            }

            if(chk_cal == buf[TFMINI_DATA_Len - 1])
            {
                cordist = buf[2] | (buf[3] << 8);
                      return   cordist;
                /*cordist > TFMINI_ACTION_DIST cm, PA8 set Low;
                  cordist <= TFMINI_ACTION_DIST cm, PA8 set High.*/
                
            }
        }
    }
}




		
//				void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
//{

//    cout_doku++;
//	
//    if (huart->Instance == USART2)  //current UART
//        {
//					
//				int a=89;
//				   
//       if (Rx_indx==0) {for (int i=0;i<9;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data 

//        if (Rx_data[0]!=a) //if received data different from 89
//            {
//							
//							
//							
//							Transfer_cplt=0;
//						 Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
//            }
//        else            //if received data = 0
//            {
//           
//								 		 		for( int i=0; i < 13; i++ ){
//												un_decoded_data[i]=	 *( Rx_Buffer + i ) ;  }
//												packet_arrived++;
//												
//												if (packet_arrived==1) //if received data different from 0
//            {
//												//	HAL_TIM_Base_Start_IT(&htim2);
//												
//						}
//												
//							Rx_indx=0;
//							 Transfer_cplt=1;
//							 
//					 
//							 
//            }
//		HAL_UART_Receive_DMA(&huart2, Rx_data, 1);
//          }
//				}

		
				void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
{
uint8_t  frame_capture=0;
	uint8_t  number_of_reads=1;
    cout_doku++;int a=89;
	
    if (huart->Instance == USART2)  //current UART
        {
					
				
				   
       if (Rx_indx==0) {for (int i=0;i<9;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data 

        if (Rx_data[0]==a) //if received data different from 89
            {
							frame_capture=1; 
								HAL_UART_Receive_DMA(&huart2, Rx_data, number_of_reads);
							
							       if (Rx_data[0]==a) {  
											 number_of_reads=7;
						}
							
						
            }
								HAL_UART_Receive_DMA(&huart2, Rx_Buffer, number_of_reads);
						
						
//        else            //if received data = 89
//            {     
//						     	frame_capture=1;
//           
//								 		 		for( int i=0; i < 13; i++ ){
//												un_decoded_data[i]=	 *( Rx_Buffer + i ) ;  }
//												packet_arrived++;
//												
//												if (packet_arrived==1) //if received data different from 0
//            {
//												//	HAL_TIM_Base_Start_IT(&htim2);
//												
//						}
//												
//							Rx_indx=0;
//							 Transfer_cplt=1;
//							 
//					 
//							 
//            }
	
          }
				}
		
		
		
		
		
		void process_gyro()

{
	
	
//	 bno055_read_euler_hrp( bno055_euler_t *euler)
    bno055_read_euler_h( &euler_h );

    bno055_read_euler_p( &euler_p );

    bno055_read_euler_r( &euler_r );

}

s8	hal_i2c_write( u8 dev_addr,u8  reg_addr,u8  *data,   u8  length) {
	

	u8 buffer[8];
	u8 i =0;
	
	buffer[0] =reg_addr;  
	for( i =0;i<length;i++)  // 0 için buffer_global(1) e data_gl[0]  i atiyor.    // 1 için 
	 {
	   buffer[i+1]=  (*data+i)  ;
	 }
												
	//HAL_I2C_Master_Transmit(&hi2c1,dev_addr,buffer,length+1,100);
	
		if  (HAL_I2C_Master_Transmit(&hi2c1,dev_addr,buffer,length+1,100)==HAL_OK  ){return 0;}
   else {return -1;}
}





s8 hal_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len) {
	
u8 i =0;  u8 array[8] = {0};   array[0] = reg_addr;
	
	HAL_I2C_Master_Transmit(&hi2c1,dev_addr,&reg_addr,1,100);
	if  (	HAL_I2C_Master_Receive(	&hi2c1,dev_addr, array, r_len, 100)==HAL_OK  ){
			
				for (i = 0; i < r_len; i++)
	    	*(reg_data + i) = array[i];return 0;
		}
   else {return -1; }

		
}
void hal_i2c_delay(u32 a) {
	
	HAL_Delay(a);
  
}
		
//				
//void process_gyro()

//{
//	//uint8_t	 is_working_pg[3];
//	 is_working_pg[0] =2;
////	 bno055_read_euler_hrp( bno055_euler_t *euler);
// is_working_pg[0] =bno055_read_euler_h( &euler_h );

//   // bno055_read_euler_p( &euler_p );

//   // bno055_read_euler_r( &euler_r );

//}
//		
//		


	char mpu_initialise(struct bno055_t *bno055 ) {
	    uint8_t	 is_working[3];
				if  (HAL_I2C_IsDeviceReady(&hi2c1,0x50, 3, 1000)==HAL_OK  )
			{ is_working[0]=1;
			HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
			}
				HAL_Delay(10);
			
	
    bno055->dev_addr = 0x50;
    bno055->bus_read = hal_i2c_read;
    bno055->bus_write = hal_i2c_write;
    bno055->delay_msec = hal_i2c_delay;
	
			
			 bno055_init( bno055 );//	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_14 ,GPIO_PIN_SET);
      bno055_set_operation_mode( BNO055_OPERATION_MODE_NDOF );  HAL_Delay(100);

    // bno055_get_operation_mode(&RecBuf[3] );  HAL_Delay(15);
	  bno055_get_operation_mode(&is_working[1] );   HAL_Delay(15);
			
			if  (is_working[1]  ==BNO055_OPERATION_MODE_NDOF  ){  is_working[1]=1;   }
			else {   is_working[1]=0; }
			
			 // 	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
			
bno055_set_accel_range( 0x01 );
bno055_get_accel_range( &range );
bno055_get_accel_unit( &temp );
bno055_set_gyro_range( 0x04 );
bno055_get_gyro_range( &range );
   
			
			
		    if  (is_working[0]  ==1  && is_working[1]  ==1     ){  
   
         
			return 1;

		}	
			
			
			
			
			
			
			
}



//	char mpu_initialise() {
//	    uint8_t	 is_working[3];
//				if  (HAL_I2C_IsDeviceReady(&hi2c1,0x50, 3, 1000)==HAL_OK  )
//			{ is_working[0]=1;
//			HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
//			}
//				HAL_Delay(10);
//			
//		extern  struct bno055_t bno055;
//    bno055.dev_addr = 0x50;
//    bno055.bus_read = hal_i2c_read;
//    bno055.bus_write = hal_i2c_write;
//    bno055.delay_msec = hal_i2c_delay;
//	
//			
//			 bno055_init( &bno055 );//	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_14 ,GPIO_PIN_SET);
//			
//}

//	char mpu_initialise() {
//	    uint8_t	 is_working[3];
//				if  (HAL_I2C_IsDeviceReady(&hi2c1,BNO055_I2C_ADDR3, 3, 1000)==HAL_OK  )
//			{device_active=1;  is_working[0]=1;
//			
//			}
//				HAL_Delay(10);
//			
//			 struct bno055_t bno055;
//    bno055.dev_addr = BNO055_I2C_ADDR3;
//    bno055.bus_read = hal_i2c_read;
//    bno055.bus_write = hal_i2c_write;
//    bno055.delay_msec = hal_i2c_delay;
//		
//    bno055_init( &bno055 );	//HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_15 ,GPIO_PIN_SET);
//			
//		bno055_set_operation_mode( BNO055_OPERATION_MODE_NDOF );  HAL_Delay(100);

//    // bno055_get_operation_mode(&RecBuf[3] );  HAL_Delay(15);
//	  bno055_get_operation_mode(&is_working[1] );   HAL_Delay(15);
//			
//			if  (is_working[1]  ==BNO055_OPERATION_MODE_NDOF  ){  is_working[1]=1;   }
//			else {   is_working[1]=0; }
//			
//			 // 	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
//			
//bno055_set_accel_range( 0x01 );
//bno055_get_accel_range( &range );
//bno055_get_accel_unit( &temp );
//bno055_set_gyro_range( 0x04 );
//bno055_get_gyro_range( &range );
//   
//			
//			
//		    if  (is_working[0]  ==1  && is_working[1]  ==1     ){  
//   
//         
//			return 1;

//		}	
//			
//			
//			
//}
//							 



	