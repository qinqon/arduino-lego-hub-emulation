#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"

#include "motors.h"

#define M1_EN 12
#define M1_PN 13
#define M2_EN 14
#define M2_PN 21
#define M3_EN 9
#define M3_PN 10
#define M4_EN 47
#define M4_PN 11

#define FORWARD 1
#define BACKWARD -1

uint8_t Speed = 50;

void advance(uint8_t motorNumber,uint8_t speed)
{
  switch(motorNumber)
  {
    case 1:
      mcpwm_set_signal_high(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_B);//Give PH a consistently high level
      mcpwm_set_duty_type(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;

    case 2:
      mcpwm_set_signal_high(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_B);//Give PH a consistently high level
      mcpwm_set_duty_type(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;

    case 3:
      mcpwm_set_signal_high(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_B);//Give PH a consistently high level
      mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;

    case 4:
      mcpwm_set_signal_high(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_B);//Give PH a consistently high level
      mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;
  }
}

void retreat(uint8_t motorNumber,uint8_t speed)
{
  switch(motorNumber)
  {
    case 1:
      mcpwm_set_signal_low(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_B);//Give PH a constant low level
      mcpwm_set_duty_type(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;

    case 2:
      mcpwm_set_signal_low(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_B);//Give PH a constant low level
      mcpwm_set_duty_type(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;

    case 3:
      mcpwm_set_signal_low(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_B);//Give PH a constant low level
      mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;

    case 4:
      mcpwm_set_signal_low(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_B);//Give PH a constant low level
      mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_A,MCPWM_DUTY_MODE_0);
      mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_A,speed);//The EN pin outputs a PWM wave with a duty cycle of "speed"
      break;
  }
}

void breake(uint8_t motorNumber)
{
  switch(motorNumber)
  {
    case 1:
      mcpwm_set_signal_low(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A);
      break;

    case 2:
      mcpwm_set_signal_low(MCPWM_UNIT_0,MCPWM_TIMER_1,MCPWM_GEN_A);
      break;

    case 3:
      mcpwm_set_signal_low(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_GEN_A);
      break;

    case 4:
      mcpwm_set_signal_low(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_GEN_A);
      break;
  }
}

void moveCar(uint8_t inputValue,uint8_t speed)
{
  switch(inputValue)
  {
    case UP:
      advance(1,speed);
      advance(2,speed);
      advance(3,speed);
      advance(4,speed);
      break;

    case DOWN:
      retreat(1,speed);
      retreat(2,speed);
      retreat(3,speed);
      retreat(4,speed);
      break;

    case LEFT:
      advance(2,speed);
      advance(4,speed);
      retreat(1,speed);
      retreat(3,speed);
      break;

    case RIGHT:
      advance(1,speed);
      advance(3,speed);
      retreat(2,speed);
      retreat(4,speed);
      break;

    case STOP:
      breake(1);
      breake(2);
      breake(3);
      breake(4);
      break;
  }
}

//Initialize the pins needed to generate the PWM signal
void mcpwm_init(void)
{
  //Configure mcpwm information
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  //A speed B direction
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0A,M1_EN);//Motor 1 GPIO
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0B,M1_PN);
  mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_0,&pwm_config);//Initializes a unit of mcpwm and binds the clock

  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM1A,M2_EN);//Motor 2 GPIO
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM1B,M2_PN);
  mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_1,&pwm_config);//Initializes a unit of mcpwm and binds the clock

  mcpwm_gpio_init(MCPWM_UNIT_1,MCPWM0A,M3_EN);//Motor 3 GPIO
  mcpwm_gpio_init(MCPWM_UNIT_1,MCPWM0B,M3_PN);
  mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_0,&pwm_config);//Initializes a unit of mcpwm and binds the clock

  mcpwm_gpio_init(MCPWM_UNIT_1,MCPWM1A,M4_EN);//Motor 4 GPIO
  mcpwm_gpio_init(MCPWM_UNIT_1,MCPWM1B,M4_PN);
  mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_1,&pwm_config);//Initializes a unit of mcpwm and binds the clock
}
