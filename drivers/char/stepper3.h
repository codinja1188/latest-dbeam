#ifndef _STEPPER3_H_
#define _STEPPER3_H_

#define STEPPER_5_6_ROTATE_45      _IO('c', 1)
#define STEPPER_5_6_ROTATE_90      _IO('c', 2)
#define STEPPER_5_6_ROTATE_180     _IO('c', 3)
#define STEPPER_5_6_ROTATE_270     _IO('c', 4)
#define STEPPER_5_6_ROTATE_360     _IO('c', 5)
#define STEPPER_5_6_ROTATE_OFF 	   _IO('c', 6)

int stepper3_chopper_rotate(int rcount);
#endif
