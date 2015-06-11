#ifndef SENSORS_H_
  #define SENSORS_H_

  void ACC_getADC ();
  void GyroGetADC ();

  void initSensors();
  void i2cRepStart(uint8_t address);
  void i2cWrite(uint8_t data );
  void i2cStop(void);
  void i2cWrite(uint8_t data );
  void i2cWriteReg(uint8_t add, uint8_t reg, uint8_t val);
#endif /* SENSORS_H_ */
