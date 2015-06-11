/*
* TOC
*
* BOARD ORIENTATION AND SETUP
* I2C ADDRESS
* MPU6050 GYRO LPF SETTING
* I2C GENERAL FUNCTIONS
* GYRO COMMON PART
* ACC COMMON PART
* I2C GYROSCOPE AND ACCELEROMETER MPU6050
*/

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "eeprom.h"
#include "IMU.h"

void waitTransmissionI2C();
void ACCInit();

// > BOARD ORIENTATION AND SETUP
#ifndef ACC_ORIENTATION
  #define ACC_ORIENTATION(X, Y, Z) { \
    imu.accADC[ROLL]  = X;           \
    imu.accADC[PITCH] = Y;           \
    imu.accADC[YAW]   = Z;           \
  }
#endif

#ifndef GYRO_ORIENTATION
  #define GYRO_ORIENTATION(X, Y, Z) { \
    imu.gyroADC[ROLL]  = X;           \
    imu.gyroADC[PITCH] = Y;           \
    imu.gyroADC[YAW]   = Z;           \
  }
#endif

#ifndef MAG_ORIENTATION
  #define MAG_ORIENTATION(X, Y, Z) {   \
    imu.magADC[ROLL]  = X;             \
    imu.magADC[PITCH] = Y;             \
    imu.magADC[YAW]   = Z;             \
  }
#endif


// > I2C ADDRESS
#ifndef MPU6050_ADDRESS
  #define MPU6050_ADDRESS     0x68
#endif


// > MPU6050 GYRO LPF SETTING
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || \
    defined(MPU6050_LPF_98HZ)  || defined(MPU6050_LPF_42HZ)  || \
    defined(MPU6050_LPF_20HZ)  || defined(MPU6050_LPF_10HZ)  || \
    defined(MPU6050_LPF_5HZ)

  #ifdef MPU6050_LPF_256HZ
    #define MPU6050_DLPF_CFG 0
  #endif
  #ifdef MPU6050_LPF_188HZ
    #define MPU6050_DLPF_CFG 1
  #endif
  #ifdef MPU6050_LPF_98HZ
    #define MPU6050_DLPF_CFG 2
  #endif
  #ifdef MPU6050_LPF_42HZ
    #define MPU6050_DLPF_CFG 3
  #endif
  #ifdef MPU6050_LPF_20HZ
    #define MPU6050_DLPF_CFG 4
  #endif
  #ifdef MPU6050_LPF_10HZ
    #define MPU6050_DLPF_CFG 5
  #endif
  #ifdef MPU6050_LPF_5HZ
    #define MPU6050_DLPF_CFG 6
  #endif
#else
  #define MPU6050_DLPF_CFG   0
#endif


uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;


// > I2C GENERAL FUNCTIONS
void i2cInit(void) {
  TWSR = 0;                               // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;  // change the I2C clock rate
  TWCR = 1 << TWEN;                       // enable twi module, no interrupt
}


void i2cRepStart(uint8_t address) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1 << TWINT) | (1 << TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}


void i2cStop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}


void i2cWrite(uint8_t data ) {
  TWDR = data;  // send data to the previously addressed device
  TWCR = (1 << TWINT) | (1 << TWEN);
  waitTransmissionI2C();
}


uint8_t i2cRead(uint8_t ack) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (ack? (1 << TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2cStop();
  return r;
}


void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1 << TWINT))) {
    count--;
    if (count == 0) {   // we are in a blocking state => we don't insist
      TWCR = 0;         // and we force a reset on TWINT register
      // we take a timestamp here to neutralize the value during a short delay
      neutralizeTime = micros();
      i2c_errors_count++;
      break;
    }
  }
}


size_t i2cReadToBuf(uint8_t add, void *buf, size_t size) {
  i2cRepStart((add << 1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size --) {
    // acknowledge all but the final byte 
    *b++ = i2cRead(size > 0);
    bytes_read++;
  }
  return bytes_read;
}


size_t i2cReadRegToBuf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2cRepStart(add << 1); // I2C write direction
  i2cWrite(reg);          // register selection
  return i2cReadToBuf(add, buf, size);
}


void i2cGetSixRawADC(uint8_t add, uint8_t reg) {
  i2cReadRegToBuf(add, reg, &rawADC, 6);
}


void i2cWriteReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2cRepStart(add << 1);  // I2C write direction
  i2cWrite(reg);          // register selection
  i2cWrite(val);          // value to write in register
  i2cStop();
}


// > GYRO COMMON PART
void GYROCommon() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;


  if (calibratingG > 0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;
      }
      // Sum up 512 readings
      g[axis] += imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis] = 0;
      gyroZero[axis] = 0;
      if (calibratingG == 1) {
        gyroZero[axis] = (g[axis] + 256) >> 9;
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis] -= gyroZero[axis];
    // anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],
                                  previousGyroADC[axis] - 800,
                                  previousGyroADC[axis] + 800);
    previousGyroADC[axis] = imu.gyroADC[axis];
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.gyroADC[PITCH] - imu.gyroADC[ROLL] ) * 7) / 10;
    imu.gyroADC[ROLL] = ((imu.gyroADC[ROLL] + imu.gyroADC[PITCH]) * 7) / 10;
    imu.gyroADC[PITCH] = temp;
  #endif

  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.gyroADC[PITCH] + imu.gyroADC[ROLL] ) * 7) / 10;
    imu.gyroADC[ROLL] = ((imu.gyroADC[ROLL] - imu.gyroADC[PITCH]) * 7) / 10;
    imu.gyroADC[PITCH] = temp;
  #endif
}


// > ACC COMMON PART
void ACCCommon() {
  static int32_t a[3];
  if (calibratingA > 0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] += imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis] = 0;
      global_conf.accZero[axis] = 0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL] + 256) >> 9;
      global_conf.accZero[PITCH] = (a[PITCH] + 256) >> 9;
      global_conf.accZero[YAW]   = ((a[YAW] + 256) >> 9) - ACC_1G;
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.accADC[PITCH] - imu.accADC[ROLL] ) * 7) / 10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL] + imu.accADC[PITCH]) * 7) / 10;
    imu.accADC[PITCH] = temp;
  #endif

  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.accADC[PITCH] + imu.accADC[ROLL] ) * 7) / 10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL] - imu.accADC[PITCH]) * 7) / 10;
    imu.accADC[PITCH] = temp;
  #endif
}


// > I2C GYROSCOPE AND ACCELEROMETER MPU6050
#ifdef MPU6050
  void Gyro_init() {
    // change the I2C clock rate to 400kHz
    TWBR = ((F_CPU / 400000L) - 16) / 2;

    // PWR_MGMT_1    -- DEVICE_RESET 1
    i2cWriteReg(MPU6050_ADDRESS, 0x6B, 0x80);

    delay(5);

    // PWR_MGMT_1
    // SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWriteReg(MPU6050_ADDRESS, 0x6B, 0x03);

    // CONFIG
    // EXT_SYNC_SET 0 (disable input pin for data sync) ;
    // default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2cWriteReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG);

    // GYRO_CONFIG
    // FS_SEL = 3: Full scale set to 2000 deg/sec
    i2cWriteReg(MPU6050_ADDRESS, 0x1B, 0x18);
  }

  void GyroGetADC () {
    i2cGetSixRawADC(MPU6050_ADDRESS, 0x43);
    // range: +/- 8192; +/- 2000 deg/sec
    GYRO_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) >> 2 ,
                      ((rawADC[2] << 8) | rawADC[3]) >> 2 ,
                      ((rawADC[4] << 8) | rawADC[5]) >> 2 );
    GYROCommon();
  }

  void ACCInit () {
    i2cWriteReg(MPU6050_ADDRESS, 0x1C, 0x10);
  }

  void ACC_getADC () {
    i2cGetSixRawADC(MPU6050_ADDRESS, 0x3B);
    ACC_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) >> 3 ,
                     ((rawADC[2] << 8) | rawADC[3]) >> 3 ,
                     ((rawADC[4] << 8) | rawADC[5]) >> 3 );
    ACCCommon();
  }
#endif /* I2C GYROSCOPE AND ACCELEROMETER MPU6050 */


void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2cInit();
  delay(100);
  Gyro_init();
  ACCInit();
  f.I2C_INIT_DONE = 1;
}
