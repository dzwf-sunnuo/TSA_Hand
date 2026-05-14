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
	


void finger_motion(uint8_t num, uint16_t speed);
void finger_single_action(uint16_t num);




#endif


