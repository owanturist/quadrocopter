#ifndef CONFIG_H_
#define CONFIG_H_

/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 *     - (ESCs calibration, Dynamic Motor/Prop Balancing, Diagnostics,Memory savings.....)
 * 8 - DEPRECATED - these features will be removed in some future release
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be changed via serial monitor or LCD.
 * 2. parameters marked with (**) in the comment are stored in eeprom and can be changed via the GUI
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    The type of multicopter    ****************************/
    #define QUADX

  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
    //#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
    //#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
    //#define MINTHROTTLE 1064 // special ESC (simonk)
    //#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
    #define MINTHROTTLE 1150 // (*) (**)

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 1850

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    #define MINCOMMAND  1000

  /**********************************    I2C speed   ************************************/
    // #define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
    #define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

  /***************************    Internal i2c Pullups   ********************************/
    /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
    //#define INTERNAL_I2C_PULLUPS

  /**************************************************************************************/
  /*****************          boards and sensor definitions            ******************/
  /**************************************************************************************/

    /***************************    Combined IMU Boards    ********************************/
      #define GY_521          // Chinese 6  DOF with  MPU6050, LLC
      
    /***************************    independent sensors    ********************************/
      /* leave it commented if you already checked a specific board above */
      /* I2C gyroscope */
      #define MPU6050       //combo + ACC

      /* I2C barometer */
      // #define BMP085

      /* I2C magnetometer */
      // #define HMC5883

      /* Board orientation shift */
      /* If you have frame designed only for + mode and you cannot rotate FC phisycally for flying in X mode (or vice versa)
       * you can use one of of this options for virtual sensors rotation by 45 deegres, then set type of multicopter according to flight mode.
       * Check motors order and directions of motors rotation for matching with new front point!  Uncomment only one option! */
      //#define SENSORS_TILT_45DEG_RIGHT        // rotate the FRONT 45 degres clockwise
      //#define SENSORS_TILT_45DEG_LEFT         // rotate the FRONT 45 degres counterclockwise


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/
  /********************************  PID Controller *********************************/
    /* choose one of the alternate PID control algorithms
     * 1 = evolved oldschool algorithm (similar to v2.2)
     * 2 = new experimental algorithm from Alex Khoroshko - unsupported - http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
     * */
    #define PID_CONTROLLER 1

    /* NEW: not used anymore for servo coptertypes  <== NEEDS FIXING - MOVE TO WIKI */
    #define YAW_DIRECTION 1
    //#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction

    #define ONLYARMWHENFLAT //prevent the copter from arming when the copter is tilted

   /********************************    ARM/DISARM    *********************************/
   /* optionally disable stick combinations to arm/disarm the motors.
     * In most cases one of the two options to arm/disarm via TX stick is sufficient */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW
    //#define ALLOW_ARM_DISARM_VIA_TX_ROLL


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200

    /* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
       if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
       it is relevent only for a conf with NK */
    #define INTERLEAVING_DELAY 3000

    /* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
       it is relevent only for a conf with at least a WMP */
    #define NEUTRALIZE_DELAY 100000


  /**************************************************************************************/
  /********                              Gyro filters                ********************/
  /**************************************************************************************/

    /*********************    Lowpass filter for some gyros    ****************************/
      /* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
         to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
         It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
         balancing options ran out. Uncomment only one option!
         IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
      //#define MPU6050_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
      //#define MPU6050_LPF_188HZ
      //#define MPU6050_LPF_98HZ
      //#define MPU6050_LPF_42HZ
      //#define MPU6050_LPF_20HZ
      //#define MPU6050_LPF_10HZ
      //#define MPU6050_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props

    /******                Gyro smoothing    **********************************/
      /* GYRO_SMOOTHING. In case you cannot reduce vibrations _and_ _after_ you have tried the low pass filter options, you
         may try this gyro smoothing via averaging. Not suitable for multicopters!
         Good results for helicopter, airplanes and flying wings (foamies) with lots of vibrations.*/
      //#define GYRO_SMOOTHING {20, 20, 3}    // (*) separate averaging ranges for roll, pitch, yaw

    /************************    Moving Average Gyros    **********************************/
      //#define MMGYRO 10                      // (*) Active Moving Average Function for Gyros
      //#define MMGYROVECTORLENGTH 15          // Length of Moving Average Vector (maximum value for tunable MMGYRO
      /* Moving Average ServoGimbal Signal Output */
      //#define MMSERVOGIMBAL                  // Active Output Moving Average Function for Servos Gimbal
      //#define MMSERVOGIMBALVECTORLENGHT 32   // Lenght of Moving Average Vector

  /************************    Analog Reads              **********************************/
    /* if you want faster analog Reads, enable this. It may result in less accurate results, especially for more than one analog channel */
    //#define FASTER_ANALOG_READS

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************        Angele throttle correction         ********************/
  /* Automatically increase throttle based on the angle of the copter
     Original idea by Kraut Rob, first implementation HAdrian							*/

  //#define THROTTLE_ANGLE_CORRECTION 40
  
 /*************************        Advanced Headfree Mode             ********************/
 /* In Advanced Headfree mode when the copter is farther than ADV_HEADFREE_RANGE meters then 
    the  bearing between home and copter position will become the control direction 
	IF copter come closer than ADV_HEADFREE_RANGE meters, then the control direction freezed to the 
	bearing between home and copter at the point where it crosses the ADV_HEADFREE_RANGE meter distance
	first implementation by HAdrian, mods by EOSBandi
 */

   //#define ADVANCED_HEADFREE									//Advanced headfree mode is enabled when this is uncommented
   //#define ADV_HEADFREE_RANGE 15								//Range where advanced headfree mode activated


  /************************        continuous gyro calibration        ********************/
  /* Gyrocalibration will be repeated if copter is moving during calibration. */
    //#define GYROCALIBRATIONFAILSAFE

  /************************        AP FlightMode        **********************************/
    /* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
    #define AP_MODE 40  // Create a deadspan for GPS.
        
  /************************   Assisted AcroTrainer    ************************************/
    /* Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
       Remember to activate ANGLE_MODE first!...
       A Value on 200 will give a very distinct transfer */
    //#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC or nunchuk is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
       for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
    #define FAILSAFE                                // uncomment  to activate the failsafe function
    #define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
    #define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
    
    #define FAILSAFE_DETECT_TRESHOLD  985

  /********************************************************************/
  /****                             Buzzer                         ****/
  /********************************************************************/
    //#define BUZZER
    //#define RCOPTIONSBEEP         // uncomment this if you want the buzzer to beep at any rcOptions change on channel Aux1 to Aux4
    //#define ARMEDTIMEWARNING 330  // (*) Trigger an alarm after a certain time of being armed [s] to save you lipo (if your TX does not have a countdown)
    //#define PILOTLAMP             //Uncomment if you are using a X-Arcraft Pilot Lamp

  /********************************************************************/
  /****           battery voltage monitoring                       ****/
  /********************************************************************/
    /* for V BAT monitoring
       after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
       with R1=33k and R2=51k
       vbat = [0;1023]*16/VBATSCALE
       must be associated with #define BUZZER ! */
    //#define VBAT              // uncomment this line to activate the vbat code
    #define VBATSCALE       131 // (*) (**) change this value if readed Battery voltage is different than real voltage
    #define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
    #define VBATLEVEL_WARN1 107 // (*) (**) 10,7V
    #define VBATLEVEL_WARN2  99 // (*) (**) 9.9V
    #define VBATLEVEL_CRIT   93 // (*) (**) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
    #define NO_VBAT          16 // Avoid beeping without any battery


  /********************************************************************/
  /****           powermeter (battery capacity monitoring)         ****/
  /********************************************************************/

    /* enable monitoring of the power consumption from battery (think of mAh)
       allows to set alarm value in GUI or via LCD
      Full description and howto here http://www.multiwii.com/wiki/index.php?title=Powermeter
       Two options:
       1 - hard: - (uses hardware sensor, after configuration gives very good results)
       2 - soft: - (good results +-5% for plush and mystery ESCs @ 2S and 3S, not good with SuperSimple ESC)    */
    //#define POWERMETER_SOFT
    //#define POWERMETER_HARD
    #define PSENSORNULL 510 /* (*) hard only: set to analogRead() value for zero current; for I=0A my sensor
                                   gives 1/2 Vss; that is approx 2.49Volt; */
    #define PINT2mA 132     /* (*) hard: one integer step on arduino analog translates to mA (example 4.9 / 37 * 1000) ;
                                   soft: use fictional value, start with 100.
                                   for hard and soft: larger PINT2mA will get you larger value for power (mAh equivalent) */


  /********************************************************************/
  /****           baord naming                                     ****/
  /********************************************************************/

    /*
     * this name is displayed together with the MultiWii version number
     * upon powerup on the LCD.
     * If you are without a DISPLAYD then You may enable LCD_TTY and
     * use arduino IDE's serial monitor to view the info.
     *
     * You must preserve the format of this string!
     * It must be 16 characters total,
     * The last 4 characters will be overwritten with the version number.
     */
    #define BOARD_NAME "MultiWii   V-.--"
    //                  123456789.123456

  /*************      Support multiple configuration profiles in EEPROM     ************/
    //#define MULTIPLE_CONFIGURATION_PROFILES

  /*************      do no reset constants when change of flashed program is detected ***********/
    #define NO_FLASH_CHECK

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********   special ESC with extended range [0-2000] microseconds  ********************/
  /**************************************************************************************/
    #define EXT_MOTOR_RANGE // using this with wii-esc requires to change MINCOMMAND to 1008 for promini and mega

  /**************************************************************************************/
  /***********************     motor, servo and other presets     ***********************/
  /**************************************************************************************/
    /* motors will not spin when the throttle command is in low position
       this is an alternative method to stop immediately the motors */
    //#define MOTOR_STOP

    /* some radios have not a neutral point centered on 1500. can be changed here */
    #define MIDRC 1482


  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/

    /* to calibrate all ESCs connected to MWii at the same time (useful to avoid unplugging/re-plugging each ESC)
       Warning: this creates a special version of MultiWii Code
       You cannot fly with this special version. It is only to be used for calibrating ESCs
       Read How To at http://code.google.com/p/multiwii/wiki/ESCsCalibration */
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000
    // #define ESC_CALIB_CANNOT_FLY  // uncomment to activate

  /****           internal frequencies                             ****/
    /* frequenies for rare cyclic actions in the main loop, depend on cycle time
       time base is main loop cycle time - a value of 6 means to trigger the action every 6th run through the main loop
       example: with cycle time of approx 3ms, do action every 6*3ms=18ms
       value must be [1; 65535] */
    #define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate)
    #define LCD_TELEMETRY_AUTO_FREQ  967// to step to next telemetry page 967 <=> 3s
    #define PSENSOR_SMOOTH 16           // len of averaging vector for smoothing the PSENSOR readings; should be power of 2; set to 1 to disable
    #define VBAT_SMOOTH 16              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable
    #define RSSI_SMOOTH 16              // len of averaging vector for smoothing the RSSI readings; should be power of 2; set to 1 to disable

  /********************************************************************/
  /****           Dynamic Motor/Prop Balancing                     ****/
  /********************************************************************/
  /*                   !!! No Fly Mode !!!                            */ 
		
    //#define DYNBALANCE   // (**) Dynamic balancing controlled from Gui

  /********************************************************************/
  /****           Regression testing                               ****/
  /********************************************************************/

    /* for development only:
       to allow for easier and reproducable config sets for test compiling, different sets of config parameters are kept
       together. This is meant to help detecting compile time errors for various features in a coordinated way.
       It is not meant to produce your flying firmware
       To use:
       - do not set any options in config.h,
       - enable with #define COPTERTEST 1, then compile
       - if possible, check for the size
       - repeat with other values of 2, 3, 4 etc.
        */
    //#define COPTERTEST 1

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  8 - DEPRECATED                                                 *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* these features will be removed in the unforseeable future. Do not build new products or
   * functionality based on such features. The default for all such features is OFF.
   */

  /**************************    WMP power pin     *******************************/
  //#define D12_POWER      // Use D12 on PROMINI to power sensors. Will disable servo[4] on D12
  /* disable use of the POWER PIN (allready done if the option RCAUXPIN12 is selected) */
  #define DISABLE_POWER_PIN


/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/

#endif /* CONFIG_H_ */

