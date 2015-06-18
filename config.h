#ifndef CONFIG_H_
  #define CONFIG_H_

  /////////////////////////////
  // CONFIGURABLE PARAMETERS //
  /////////////////////////////

  /* 1 - BASIC SETUP
   * 2 - COPTER TYPE SPECIFIC OPTIONS
   * 3 - ALTERNATE SETUP
   * 4 - TUNING & DEVELOPER
   *
   * 
   * Notes:
   * - parameters marked with (*) in the comment are stored in eeprom
   *   and can be changed via the GUI
   */


  /////////////////////////////
  // SECTION 1 - BASIC SETUP //
  /////////////////////////////

  // THE TYPE OF MULTICOPTER
  // #define QUADP    
  #define QUADX

  // MOTOR MINTHROTTLE
  /* Set the minimum throttle command sent to the ESC
     (Electronic Speed Controller) This is the minimum
     value that allow motors to run at a idle speed  */
  #define MINTHROTTLE 1150 // (*)

  // MOTOR MAXTHROTTLE
  /* this is the maximum value for the ESCs at full power,
     this value can be increased up to 2000 */
  #define MAXTHROTTLE 2000

  // MINCOMMAND
  /* this is the value for the ESCs when they are not armed in some cases,
     this value must be lowered down to 900 for some specific ESCs,
     otherwise they failed to initiate */
  #define MINCOMMAND  1000

  // I2C SPEED
  // 100kHz normal mode, this value must be used for a genuine WMP
  // #define I2C_SPEED 100000L
  // 400kHz fast mode, it works only with some WMP clones
  #define I2C_SPEED 400000L


  // BOARDS AND SENSOR DEFINITIONS
  // Combined IMU Boards
  #define GY_521

  // INDEPENDENT SENSORS
  // I2C gyroscope
  #define MPU6050       // combo + ACC


  // BOARD ORIENTATION SHIFT
  /**
   * If you have frame designed only for + mode and you cannot rotate FC
   * phisycally for flying in X mode (or vice versa) you can use one of this
   * options for virtual sensors rotation by 45 deegres, then set type of
   * multicopter according to flight mode. Check motors order and directions
   * of motors rotation for matching with new front point!
   * 
   * !Uncomment only one option!
   */
  /**
   * rotate the FRONT 45 degres clockwise
   */
  // #define SENSORS_TILT_45DEG_RIGHT
  /**
   * rotate the FRONT 45 degres counterclockwise
   */
  // #define SENSORS_TILT_45DEG_LEFT


  //////////////////////////////////////////////
  // SECTION 2 - COPTER TYPE SPECIFIC OPTIONS //
  //////////////////////////////////////////////

  // PID CONTROLLER
  /**
   * choose one of the alternate PID control algorithms
   * 1 = evolved oldschool algorithm (similar to v2.2)
   * 2 = new experimental algorithm from Alex Khoroshko - unsupported
   */
  #define PID_CONTROLLER 1

  #define YAW_DIRECTION -1


   /////////////////////////////////
   // SECTION 3 - ALTERNATE SETUP //
   /////////////////////////////////

  // SERIAL COM SPEED
  /**
   * This is the speed of the serial interfaces
   */
  #define SERIAL0_COM_SPEED 115200
  #define SERIAL1_COM_SPEED 115200
  #define SERIAL2_COM_SPEED 115200
  #define SERIAL3_COM_SPEED 115200

  // GYRO FILTERS
  /**
   * Lowpass filter for some gyros
   *
   * MPU6050 Low pass filter setting. In case you cannot eliminate
   * all vibrations to the Gyro, you can try to decrease the LPF frequency,
   * only one step per try. As soon as twitching gone, stick with that
   * setting. It will not help on feedback wobbles, so change only when  
   * copter is randomly twiching and all dampening
   * and balancing options ran out.
   * Uncomment only one option!
   * IMPORTANT! Change low pass filter setting changes PID behaviour,
   * so retune your PID's after changing LPF.
   */
  // #define MPU6050_LPF_256HZ     // This is the default setting
  // #define MPU6050_LPF_188HZ
  #define MPU6050_LPF_98HZ
  // #define MPU6050_LPF_42HZ
  // #define MPU6050_LPF_20HZ
  // #define MPU6050_LPF_10HZ
  // #define MPU6050_LPF_5HZ       // Use this only in extreme cases

  // GYRO SMOOTHING
  /**
   * In case you cannot reduce vibrations _and_ _after_ you have tried the
   * low pass filter options, you may try this gyro smoothing via averaging.
   * Not suitable for multicopters! Good results for helicopter, airplanes
   * and flying wings (foamies) with lots of vibrations.
   */
  // separate averaging ranges for roll, pitch, yaw
  #define GYRO_SMOOTHING {20, 20, 3}


  ////////////////////////////////////
  // SECTION 4 - TUNING & DEVELOPER //
  ////////////////////////////////////

  // MOTOR AND OTHER PRESETS
  /**
   * some radios have not a neutral point centered on 1500
   */
  #define MIDRC 1500

  // ESCS CALIBRATION
  /**
   * to calibrate all ESCs connected to MWii at the same time (useful
   * to avoid unplugging/re-plugging each ESC) Warning: this creates a special
   * version of MultiWii Code You cannot fly with this special version.
   * It is only to be used for calibrating ESCs Read How
   * To at http://code.google.com/p/multiwii/wiki/ESCsCalibration
   */
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000
    // #define ESC_CALIB_CANNOT_FLY  // uncomment to activate

  // WMP POWER PIN
  /**
   * disable use of the POWER PIN
   * (allready done if the option RCAUXPIN12 is selected)
   */
  #define DISABLE_POWER_PIN

#endif /* CONFIG_H_ */
