#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

void initializeSoftPWM(void);


// motor pin order
/**
 * since we are using the PWM generation in a direct way,
 * the pin order isjust to inizialie the right pins it's not
 * possible to change a PWM output pin just by changing the order
 */
uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};

// writes the motors values to the pwm compare register
// [1000;2000] => [125;250]
void writeMotors() {
  // Specific PWM Timers & Registers for the atmega328P (Promini)
  OCR1A = motor[0] >> 3;   // pin 9
  OCR1B = motor[1] >> 3;   // pin 10
  OCR2A = motor[2] >> 3;   // pin 11
  OCR2B = motor[3] >> 3;   // pin 3
}

// writes the mincommand to all motors
// Sends commands to all motors
void writeAllMotors(int16_t mc) {
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    motor[i] = mc;
  }
  writeMotors();
}

// initialize the pwm timers and registers
void initOutput() {
  // mark all PWM pins as Output
  for(uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
  }

  // Specific PWM Timers & Registers for the atmega328P (Promini)
  TCCR1A |= _BV(COM1A1);  // connect pin  9 to timer 1 channel A
  TCCR1A |= _BV(COM1B1);  // connect pin 10 to timer 1 channel B
  TCCR2A |= _BV(COM2A1);  // connect pin 11 to timer 2 channel A
  TCCR2A |= _BV(COM2B1);  // connect pin  3 to timer 2 channel B

  // special version of MultiWii to calibrate all attached ESCs
  #ifdef ESC_CALIB_CANNOT_FLY
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

// mixes the computed stabilize values to the motors
void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] +            \
                        axisPID[ROLL] * X   +            \
                        axisPID[PITCH] * Y  +            \
                        YAW_DIRECTION * axisPID[YAW] * Z

  #if defined(QUADP)
    motor[0] = PIDMIX( 0,+1,-1); // REAR
    motor[1] = PIDMIX(-1, 0,+1); // RIGHT
    motor[2] = PIDMIX(+1, 0,+1); // LEFT
    motor[3] = PIDMIX( 0,-1,-1); // FRONT
  #elif defined(QUADX)
    motor[0] = PIDMIX(-1,+1,-1); // REAR_R
    motor[1] = PIDMIX(-1,-1,+1); // FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); // REAR_L
    motor[3] = PIDMIX(+1,-1,-1); // FRONT_L
  #else
    #error "missing coptertype mixtable entry"
  #endif

  // normalize the Motors values
  maxMotor = motor[0];
  for(i = 1; i< NUMBER_MOTOR; i++) {
    if (motor[i] > maxMotor) {
      maxMotor = motor[i];
    }
  }

  for(i=0; i < NUMBER_MOTOR; i++) {
    // this is a way to still have good gyro
    // corrections if at least one motor reaches its max.
    if (maxMotor > MAXTHROTTLE) {
      motor[i] -= maxMotor - MAXTHROTTLE;
    }
    motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);

    if (rcData[THROTTLE] < MINCHECK) {
      motor[i] = conf.minthrottle;
    }

    if (!f.ARMED) {
      motor[i] = MINCOMMAND;
    }
  }
}
