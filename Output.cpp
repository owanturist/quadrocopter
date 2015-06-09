/*
* TOC
*
* MOTOR PIN ORDER
* WRITES THE MOTORS VALUES TO THE PWM COMPARE REGISTER
* - Specific PWM Timers & Registers for the atmega328P (Promini)
* WRITES THE MINCOMMAND TO ALL MOTORS
* INITIALIZE THE PWM TIMERS AND REGISTERS
* - mark all PWM pins as Output
* - Specific PWM Timers & Registers for the atmega328P (Promini)
* - special version of MultiWii to calibrate all attached ESCs
*/

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);


// > MOTOR PIN ORDER
// since we are uing the PWM generation in a direct way,
// the pin order isjust to inizialie the right pins its
// not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  //for a quad+: rear,right,left,front
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};
#endif



// > WRITES THE MOTORS VALUES TO THE PWM COMPARE REGISTER
// [1000;2000] => [125;250]
void writeMotors() {
  // >> Specific PWM Timers & Registers for the atmega328P (Promini)
  #if defined(PROMINI)
    OCR1A = motor[0]>>3; //  pin 9
    OCR1B = motor[1]>>3; //  pin 10
    OCR2A = motor[2]>>3; //  pin 11
    OCR2B = motor[3]>>3; //  pin 3
  #endif
}

// > WRITES THE MINCOMMAND TO ALL MOTORS
// Sends commands to all motors
void writeAllMotors(int16_t mc) {
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

// > INITIALIZE THE PWM TIMERS AND REGISTERS
void initOutput() {
  // >> mark all PWM pins as Output
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }

  // >> Specific PWM Timers & Registers for the atmega328P (Promini)
  #if defined(PROMINI)
    TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
  #endif

  // >> special version of MultiWii to calibrate all attached ESCs
  #if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    delay(4000);
    writeAllMotors(ESC_CALIB_LOW);
    while (1) {
      delay(5000);
    }
    exit; // statement never reached
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
}




// > MIXES THE COMPUTED STABILIZE VALUES TO THE MOTORS
void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X+ axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  #define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

  // >> main Mix Table
  #if defined( QUADP )
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif

  // >> normalize the Motors values
  maxMotor=motor[0];
  for(i=1; i< NUMBER_MOTOR; i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for(i=0; i< NUMBER_MOTOR; i++) {
    // this is a way to still have good gyro
    // corrections if at least one motor reaches its max.
    if (maxMotor > MAXTHROTTLE) {
      motor[i] -= maxMotor - MAXTHROTTLE;
    }
    motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
    if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE) {
      motor[i] = conf.minthrottle;
    }
    if (!f.ARMED) {
      motor[i] = MINCOMMAND;
    }
  }
}
