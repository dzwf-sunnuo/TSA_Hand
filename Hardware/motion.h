//#include "main.h"
#ifndef __MOTION_H__
#define __MOTION_H__

//#define COUNT_NUMBERS               1
//#define CONTINUOUS_BENDING          2
enum Motion{COUNT_NUMBERS			 = 1,
						CONTINUOUS_BENDING = 2,
						OTHER_MOTION			 = 3,
						CONTINUOUS_FISTING = 4



};
	


extern volatile uint8_t rcvFlag;//接收标志位需要在main.c内使用
void finger_motion(uint8_t num,uint16_t speed);
void finger_single_action(uint8_t num);




#endif


