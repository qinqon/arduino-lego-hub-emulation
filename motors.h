#ifndef __MOTORS_H__
#define __MOTORS_H__

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

void mcpwm_init(void);
void moveCar(uint8_t inputValue,uint8_t speed);
#endif // __MOTORS_H__
