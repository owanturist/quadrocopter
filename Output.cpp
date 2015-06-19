#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

void initializeSoftPWM(void);


// порядок контактов моторов
uint8_t PWM_PIN[4] = {9,10,11,3};

// записать значения мощностей двигателей в сравнивающий регистр ШИМ
// [1000;2000] => [125;250]
void writeMotors() {
  OCR1A = motor[0] >> 3;   // pin 9
  OCR1B = motor[1] >> 3;   // pin 10
  OCR2A = motor[2] >> 3;   // pin 11
  OCR2B = motor[3] >> 3;   // pin 3
// величина рабочего цикла ШИМ задаётся в диапазоне от 0 до 255
// например: motor[0] = 1600 => OCR1A = 200,
// тогда рабочий цикл: (200 + 1) / 256 = 78.5%
}

// отправить команду на все моторы
void writeAllMotors(int16_t mc) {
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    motor[i] = mc;
  }
  writeMotors();
}

// инициализация ШИМ таймеров и регистров
void initOutput() {
  // назначить все контакты ШИМ в качестве выходных
  for(uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
  }

  TCCR1A |= _BV(COM1A1);  // назначить контакту  9 таймер 1 канала A
  TCCR1A |= _BV(COM1B1);  // назначить контакту 10 таймер 1 канала B
  TCCR2A |= _BV(COM2A1);  // назначить контакту 11 таймер 2 канала A
  TCCR2A |= _BV(COM2B1);  // назначить контакту  3 таймер 2 канала B

  #ifdef ESC_CALIB_CANNOT_FLY
    writeAllMotors(ESC_CALIB_HIGH);
    delay(4000);
    writeAllMotors(ESC_CALIB_LOW);
    while (1) {
      delay(5000);
    }
    exit; // объявление никогда не будет достигнуто
  #endif

  // записать минимальную команду для всех двигателей
  writeAllMotors(MINCOMMAND);
  delay(300);
}

// смешивает вычисленные значения стабилизации для двигателей
void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] +             \
                        axisPID[ROLL]  * X  +             \
                        axisPID[PITCH] * Y  +             \
                        axisPID[YAW]   * Z  * YAW_DIRECTION

  #if defined(QUADP)
    motor[0] = PIDMIX( 0,+1,-1);  // задний
    motor[1] = PIDMIX(-1, 0,+1);  // правый
    motor[2] = PIDMIX(+1, 0,+1);  // левый
    motor[3] = PIDMIX( 0,-1,-1);  // передний
  #elif defined(QUADX)
    motor[0] = PIDMIX(-1,+1,-1);  // задний правый
    motor[1] = PIDMIX(-1,-1,+1);  // передний правый
    motor[2] = PIDMIX(+1,+1,+1);  // задний левый
    motor[3] = PIDMIX(+1,-1,-1);  // задний правый
  #else
    #error "missing coptertype"
  #endif

  // нормализация значения мощности моторов
  maxMotor = motor[0];
  for(i = 1; i< NUMBER_MOTOR; i++) {
    if (motor[i] > maxMotor) {
      maxMotor = motor[i];
    }
  }

  for(i=0; i < NUMBER_MOTOR; i++) {
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
