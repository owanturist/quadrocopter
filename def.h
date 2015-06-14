#ifndef DEF_H_
#define DEF_H_

// Proc auto detection
#if   defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
  #define PROMINI
#elif defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) \
   || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif


// motor and servo numbers
#define DYNBAL          0
#define FLAP            0
#define NUMBER_MOTOR    4


// atmega328P (Promini)
#ifdef PROMINI
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT)
  // switch LEDPIN state (digital PIN 13)
  #define LEDPIN_TOGGLE              PINB |= 1<<5
  #define LEDPIN_OFF                 PORTB &= ~(1<<5)
  #define LEDPIN_ON                  PORTB |= (1<<5)

  #define POWERPIN_PINMODE
  #define POWERPIN_ON
  #define POWERPIN_OFF

  // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4 PORTC |= 1<<5
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4) PORTC &= ~(1<<5)

  #define PINMODE_LCD                pinMode(0, OUTPUT)
  // switch OFF digital PIN 0
  #define LCDPIN_OFF                 PORTD &= ~1
  #define LCDPIN_ON                  PORTD |= 1
  #define STABLEPIN_PINMODE
  #define STABLEPIN_ON
  #define STABLEPIN_OFF

  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING) //PIN 0
  #define SPEK_SERIAL_PORT           0
  // RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
    
  #define PCINT_PIN_COUNT            5
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define PCINT_RX_PORT              PORTD
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PIND
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
  
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
  #define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
  #define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);

  #define SOFT_PWM_3_PIN_HIGH        PORTC |= 1<<2;
  #define SOFT_PWM_3_PIN_LOW         PORTC &= ~(1<<2);
  #define SOFT_PWM_4_PIN_HIGH        PORTB |= 1<<4;
  #define SOFT_PWM_4_PIN_LOW         PORTB &= ~(1<<4);
#endif


// IMU Orientations and Sensor definitions
#if defined(GY_521)
  #define MPU6050
  #undef INTERNAL_I2C_PULLUPS

  #define ACC_ORIENTATION(X, Y, Z)  { \
    imu.accADC[ROLL]  = -X;           \
    imu.accADC[PITCH] = -Y;           \
    imu.accADC[YAW]   =  Z;           \
  }

  #define GYRO_ORIENTATION(X, Y, Z) { \
    imu.gyroADC[ROLL]  =  Y;          \
    imu.gyroADC[PITCH] = -X;          \
    imu.gyroADC[YAW]   = -Z;          \
  }
#endif


// Sensor Type definitions
#if defined(MPU6050)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(MPU6050)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#define MAG 0
#define BARO 0
#define GPS 0
#define SONAR 0


#if defined(MPU6050)
  #define ACC_1G 512
#endif

#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#if defined(MPU6050)
  // MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)
#endif

// Multitype decleration for the GUI's
#if defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#endif

#define STANDARD_RX

#define RC_CHANS 8


// Error Checking Section
#ifndef NUMBER_MOTOR
  #error "NUMBER_MOTOR is not set"
#endif

#endif /* DEF_H_ */
