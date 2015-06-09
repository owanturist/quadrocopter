/*
* TOC
*
* RC ALIAS
* AUTOMATIC ACC OFFSET CALIBRATION
* POWER METER
* RC FUNCTIONS
* MOTOR AND SERVO FUNCTIONS
* EEPROM LAYOUT DEFINITION
* GPS COMMON VARIABLES
* SETUP
* GO ARM
* GO DISARM
* LOOP
* - sticks command handler
* - pitch & roll & yaw pid
*/

#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "eeprom.h"
#include "IMU.h"
#include "LCD.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "GPS.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

// > RC ALIAS
const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

// names for dynamic generation of config GUI
const char boxnames[] PROGMEM =
  "ARM;"
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #ifdef VARIOMETER
    "VARIO;"
  #endif
  #if MAG
    "MAG;"
    "HEADFREE;"
    "HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    "CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    "CAMTRIG;"
  #endif
  #if GPS
    "GPS HOME;"
    "GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    "PASSTHRU;"
  #endif
  #if defined(BUZZER)
    "BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    "LEDMAX;"
    "LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    "LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    "CALIB;"
  #endif
  #ifdef GOVERNOR_P
    "GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    "OSD SW;"
  #endif
;

// permanent IDs associated to boxes.
// This way, you can rely on an ID number to identify a BOX function.
const uint8_t boxids[] PROGMEM = {
  0, //"ARM;"
  #if ACC
    1, //"ANGLE;"
    2, //"HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    3, //"BARO;"
  #endif
  #ifdef VARIOMETER
    4, //"VARIO;"
  #endif
  #if MAG
    5, //"MAG;"
    6, //"HEADFREE;"
    7, //"HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    8, //"CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    9, //"CAMTRIG;"
  #endif
  #if GPS
    10, //"GPS HOME;"
    11, //"GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    12, //"PASSTHRU;"
  #endif
  #if defined(BUZZER)
    13, //"BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    14, //"LEDMAX;"
    15, //"LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    16, //"LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    17, //"CALIB;"
  #endif
  #ifdef GOVERNOR_P
    18, //"GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    19, //"OSD_SWITCH;"
  #endif
};


uint32_t currentTime = 0;
uint16_t previousTime = 0;
// this is the number in micro second to achieve a full loop,
// it can differ a little and is taken into account in the PID loop
uint16_t cycleTime = 0;
// the calibration is done in the main loop.
// Calibrating decreases at each cycle down to 0,
// then we enter in a normal mode.
uint16_t calibratingA = 0;
// baro calibration = get new ground pressure value
uint16_t calibratingB = 0;
uint16_t calibratingG;
// [-180;+180]
int16_t  magHold,headFreeModeHold;
// lowest battery voltage in 0.1V steps
uint8_t  vbatMin = VBATNOMINAL;
uint8_t  rcOptions[CHECKBOXITEMS];
// in cm
int32_t  AltHold;
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0,0,0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

#if defined(ARMEDTIMEWARNING)
  uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

int16_t  debug[4];

flags_struct_t f;

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;



#if defined(THROTTLE_ANGLE_CORRECTION)
  // correction of throttle in lateral wind,
  int16_t throttleAngleCorrection = 0;
  // cos(angleZ)*100
  int8_t  cosZ = 100;
#endif



// > AUTOMATIC ACC OFFSET CALIBRATION
#if defined(INFLIGHT_ACC_CALIBRATION)
  uint16_t InflightcalibratingA = 0;
  int16_t AccInflightCalibrationArmed;
  uint16_t AccInflightCalibrationMeasurementDone = 0;
  uint16_t AccInflightCalibrationSavetoEEProm = 0;
  uint16_t AccInflightCalibrationActive = 0;
#endif

// > POWER METER
uint16_t intPowerTrigger1;


// > RC FUNCTIONS
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

// interval [1000;2000]
int16_t rcData[RC_CHANS];
// interval [1000;2000] - is rcData coming from MSP
int16_t rcSerial[8];
// interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t rcCommand[4];
// a counter to select legacy RX when there is no more MSP rc serial data
uint8_t rcSerialCount = 0;
// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupPitchRollRC[5];
// lookup table for expo & mid THROTTLE
int16_t lookupThrottleRC[11];


// > MOTOR AND SERVO FUNCTIONS
int16_t axisPID[3];
int16_t motor[8];
int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};

// > EEPROM LAYOUT DEFINITION
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;


// > GPS COMMON VARIABLES
// the angles that must be applied for GPS correction
int16_t  GPS_angle[2] = { 0, 0};
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
uint8_t  GPS_numSat;
// distance to home - unit: meter
uint16_t GPS_distanceToHome;
// direction to home - unit: degree
int16_t  GPS_directionToHome;
// GPS altitude - unit: meter
uint16_t GPS_altitude;
// GPS speed - unit: cm/s
uint16_t GPS_speed;
// a binary toogle to distinct a GPS position update
uint8_t  GPS_update = 0;
// - unit: degree*10
uint16_t GPS_ground_course = 0;
// Checksum from Gps serial
uint8_t  GPS_Present = 0;
uint8_t  GPS_Enable  = 0;

// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
int16_t  nav[2];
//Adding a rate controller to the navigation to make it smoother
int16_t  nav_rated[2];

uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

uint8_t alarmArray[16];
 
#if BARO
  int32_t baroPressure;
  int32_t baroTemperature;
  int32_t baroPressureSum;
#endif

// this code is excetuted at each loop and won't
// interfere with control loop if it lasts less than 650 microseconds
void annexCode() {
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[THROTTLE]>1500) { // breakpoint is fix: 1500
    if (rcData[THROTTLE]<2000) {
      //  /512 instead of /500
      prop2 -=  ((uint16_t)conf.dynThrPID*(rcData[THROTTLE]-1500)>>9);
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) {  //ROLL & PITCH
      tmp2 = tmp>>7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp-(tmp2<<7))
                      * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7);
      // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = 128-((uint16_t)conf.rollPitchRate*tmp>>9);
      // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      prop1 = (uint16_t)prop1*prop2>>7;
      // was /100, is /128 now
      dynP8[axis] = (uint16_t)conf.pid[axis].P8*prop1>>7;
      // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8*prop1>>7;
    } else {  // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  // [MINCHECK;2000] -> [0;2559]
  tmp = (uint32_t)(tmp-MINCHECK)*2559/(2000-MINCHECK);
  tmp2 = tmp/256; // range [0;9]
  // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*256)
                      * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 256;

  if(f.HEADFREE_MODE) { //to optimize
    // where PI/180 ~= 0.0174533
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f;
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader =0;
  switch (analogReader++%3) {

  } // end of switch()


  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }



  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  //Only one serial port on ProMini.
  //Skip serial com if Spektrum Sat in use.
  //Note: Spek code will auto-call serialCom if GUI data detected on serial0.
  #if !(defined(SPEKTRUM) && defined(PROMINI))
    serialCom();
  #endif


  if (f.ARMED)  {
    #if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
      armedTime += (uint32_t)cycleTime;
    #endif
    #if defined(VBAT)
      if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
    #endif
    #ifdef LCD_TELEMETRY
      #if BARO
        if ( (alt.EstAlt > BAROaltMax) ) BAROaltMax = alt.EstAlt;
      #endif
      #if GPS
        if ( (GPS_speed > GPS_speedMax) ) GPS_speedMax = GPS_speed;
      #endif
    #endif
  }
}

// > SETUP
void setup() {
  // LEDPIN_PINMODE;
  // POWERPIN_PINMODE;
  // BUZZERPIN_PINMODE;
  // STABLEPIN_PINMODE;
  // POWERPIN_OFF;

  SerialOpen(0,SERIAL0_COM_SPEED);
  initOutput();
  readGlobalSet();

  #ifndef NO_FLASH_CHECK
    #if defined(MEGA)
      // only first ~64K for mega board due to
      // pgm_read_byte limitation
      uint16_t i = 65000;
    #else
      uint16_t i = 32000;
    #endif

    uint16_t flashsum = 0;
    uint8_t pbyt;
    while(i--) {
      // calculate flash checksum
      pbyt =  pgm_read_byte(i);
      flashsum += pbyt;
      flashsum ^= (pbyt<<8);
    }
  #endif

  global_conf.currentSet=0;

  // check settings integrity
  while(1) {
    #ifndef NO_FLASH_CHECK
      // check current setting integrity
      if(readEEPROM()) {
        // update constants if firmware is changed and integrity is OK
        if(flashsum != global_conf.flashsum) update_constants();
      }
    #else
      // check current setting integrity
      readEEPROM();
    #endif
    // all checks is done
    if(global_conf.currentSet == 0) break;
    // next setting for check
    global_conf.currentSet--;
  }

  // reload global settings for get last profile number
  readGlobalSet();

  #ifndef NO_FLASH_CHECK
    if(flashsum != global_conf.flashsum) {
      // new flash sum
      global_conf.flashsum = flashsum;
      // update flash sum in global config
      writeGlobalSet(1);
    }
  #endif

  readEEPROM();  // load setting data from last used profile
  configureReceiver();
  initSensors();

  previousTime = micros();
  calibratingG = 512;
  // 10 seconds init_delay + 200 * 25 ms = 15
  // seconds before ground pressure settles
  calibratingB = 200;

  f.SMALL_ANGLES_25 = 1; // important for gyro only conf

  debugmsg_append_str("initialization completed\n");
}

// > GO ARM
void go_arm() {
  if(calibratingG == 0
  #if defined(ONLYARMWHENFLAT)
    && f.ACC_CALIBRATED 
  #endif
    ) {
    if(!f.ARMED && !f.BARO_MODE) { // arm now!
      f.ARMED = 1;
      headFreeModeHold = att.heading;
      magHold = att.heading;
    }
  }
}

// > GO DISARM
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
  }
}

// > LOOP
void loop () {
  // this indicates the number of time
  // (multiple of RC measurement at 50Hz)
  // the sticks must be maintained to run or switch off motors
  static uint8_t rcDelayCommand;

  // this hold sticks position for command combos
  static uint8_t rcSticks;

  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta;
  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0,0};
  static int16_t errorAngleI[2] = {0,0};

  #if PID_CONTROLLER == 1
    static int32_t errorGyroI_YAW;
    static int16_t delta1[2],delta2[2];
    static int16_t errorGyroI[2] = {0,0};

  #elif PID_CONTROLLER == 2
    static int16_t delta1[3],delta2[3];
    static int32_t errorGyroI[3] = {0,0,0};
    static int16_t lastError[3] = {0,0,0};
    int16_t deltaSum;
    int16_t AngleRateTmp, RateError;
  #endif

  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
  int16_t rc;
  int32_t prop = 0;


  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();

    // >> sticks command handler
    // checking sticks positions
    uint8_t stTmp = 0;
    for(i=0;i<4;i++) {
      stTmp >>= 2;
      if(rcData[i] > MINCHECK) stTmp |= 0x80;  // check for MIN
      if(rcData[i] < MAXCHECK) stTmp |= 0x40;  // check for MAX
    }
    if(stTmp == rcSticks) {
      if(rcDelayCommand<250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) {  // THROTTLE at minimum
      if (conf.activate[BOXARM] > 0) {   // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM )
          go_arm(); else if (f.ARMED) go_disarm();
      }
    }

    if(rcDelayCommand == 20) {
      if(f.ARMED) {  // actions during armed
        if (conf.activate[BOXARM] == 0 &&
          rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
            go_disarm();
          }

      } else {  // actions during not armed
        i=0;
        // GYRO calibration
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
          calibratingG=512;

          #if BARO
            // calibrate baro to new ground
            // level (10 * 25 ms = ~250 ms non blocking)
            calibratingB=10;
          #endif
        }

        else if (conf.activate[BOXARM] == 0 &&
          rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
            go_arm();
          }

        #if ACC
          else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
            calibratingA=512;  // throttle=max, yaw=left, pitch=min
          }
        #endif
        #if MAG
          else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
            f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
          }
        #endif

        i=0;
        if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
          conf.angleTrim[PITCH]+=2; i=1;
        } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
          conf.angleTrim[PITCH]-=2; i=1;
        } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
          conf.angleTrim[ROLL] +=2; i=1;
        } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
          conf.angleTrim[ROLL] -=2; i=1;
        }

        if (i) {
          writeParams(1);
          rcDelayCommand = 0;  // allow autorepetition
        }
      }
    }


    uint16_t auxState = 0;
    for(i=0;i<4;i++) {
      auxState |= (rcData[AUX1+i]<1300)<<(3*i)
               | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1)
               | (rcData[AUX1+i]>1700)<<(3*i+2);

    }
    for(i=0;i<CHECKBOXITEMS;i++) {
      rcOptions[i] = (auxState & conf.activate[i])>0;
    }

    #if ACC
      if ( rcOptions[BOXANGLE] ) { 
        // bumpless transfer to Level mode
        if (!f.ANGLE_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.ANGLE_MODE = 1;
        }
      } else {
        // failsafe support
        f.ANGLE_MODE = 0;
      }
      if ( rcOptions[BOXHORIZON] ) {
        f.ANGLE_MODE = 0;
        if (!f.HORIZON_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.HORIZON_MODE = 1;
        }
      } else {
        f.HORIZON_MODE = 0;
      }
    #endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;

    #if BARO
      #if (!defined(SUPPRESS_BARO_ALTHOLD))
        if (rcOptions[BOXBARO]) {
          if (!f.BARO_MODE) {
            f.BARO_MODE = 1;
            AltHold = alt.EstAlt;
            initialThrottleHold = rcCommand[THROTTLE];
            errorAltitudeI = 0;
            BaroPID=0;
          }
        } else {
          f.BARO_MODE = 0;
        }
      #endif
    #endif

    #if MAG
      if (rcOptions[BOXMAG]) {
        if (!f.MAG_MODE) {
          f.MAG_MODE = 1;
          magHold = att.heading;
        }
      } else {
        f.MAG_MODE = 0;
      }
      if (rcOptions[BOXHEADFREE]) {
        if (!f.HEADFREE_MODE) {
          f.HEADFREE_MODE = 1;
        }
      } else {
        f.HEADFREE_MODE = 0;
      }
      if (rcOptions[BOXHEADADJ]) {
        headFreeModeHold = att.heading; // acquire new heading
      }
    #endif

  } else { // not in rc loop
    // never call all functions in the same loop, to avoid high delay spikes
    static uint8_t taskOrder=0;

    if(taskOrder>4) taskOrder-=5;
    switch (taskOrder) {
      case 0:
        taskOrder++;
        #if MAG
          // max 350 µs (HMC5883) // only break when we actually did something
          if (Mag_getADC()) break;
        #endif
      case 1:
        taskOrder++;
        #if BARO
          if (Baro_update() != 0 ) break;
        #endif
      case 2:
        taskOrder++;
        #if BARO
          if (getEstimatedAltitude() !=0) break;
        #endif
      case 3:
        taskOrder++;
      case 4:
        taskOrder++;
        break;
    }
  }
 
  computeIMU();

  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  #if MAG
    if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
      int16_t dif = att.heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.pid[PIDMAG].P8>>5;
    } else magHold = att.heading;
  #endif

  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    /**
     * Smooth alt change routine , for slow auto and aerophoto
     * modes(in general solution from alexmos). It's slowly 
     * increase/decrease altitude proportional to stick movement
     * (+/-100 throttle gives about +/-50 cm in 1 second with
     * cycle time about 3-4ms)
     */
    if (f.BARO_MODE) {
      static uint8_t isAltHoldChanged = 0;
      static int16_t AltHoldCorr = 0;
      if (abs(rcCommand[THROTTLE]-initialThrottleHold)
          >ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        /**
         * Slowly increase/decrease AltHold proportional
         * to stick movement ( +100 throttle gives ~ +50 cm
         * in 1 second with cycle time about 3-4ms)
         */
        AltHoldCorr+= rcCommand[THROTTLE] - initialThrottleHold;
        if(abs(AltHoldCorr) > 512) {
          AltHold += AltHoldCorr/512;
          AltHoldCorr %= 512;
        }
        isAltHoldChanged = 1;
      } else if (isAltHoldChanged) {
        AltHold = alt.EstAlt;
        isAltHoldChanged = 0;
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
  #endif

  #if defined(THROTTLE_ANGLE_CORRECTION)
    if(f.ANGLE_MODE || f.HORIZON_MODE) {
       rcCommand[THROTTLE]+= throttleAngleCorrection;
    }
  #endif


  // >> pitch & roll & yaw pid
  #if PID_CONTROLLER == 1 // evolved oldschool
    if ( f.HORIZON_MODE ) {
      prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),512);
    }

    // PITCH & ROLL
    for(axis=0;axis<2;axis++) {
      rc = rcCommand[axis]<<1;
      error = rc - imu.gyroData[axis];
      // WindUp   16 bits is ok here
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);

      if (abs(imu.gyroData[axis])>640) {
        errorGyroI[axis] = 0;
      }

      // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
      ITerm = (errorGyroI[axis]>>7)*conf.pid[axis].I8>>6;

      PTerm = (int32_t)rc*conf.pid[axis].P8>>6;

      if (f.ANGLE_MODE || f.HORIZON_MODE) { // axis relying on ACC
        // 50 degrees max inclination
        errorAngle = constrain(rc + GPS_angle[axis],-500,+500)
                   - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here

        // WindUp, 16 bits is ok here
        errorAngleI[axis] = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);

        // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768
        // 16 bits is ok for result
        PTermACC = ((int32_t)errorAngle*conf.pid[PIDLEVEL].P8)>>7;
        
        int16_t limit      = conf.pid[PIDLEVEL].D8*5;
        PTermACC           = constrain(PTermACC,-limit,+limit);

        // 32 bits is needed for calculation:10000*I8 could exceed 32768
        // 16 bits is ok for result
        ITermACC = ((int32_t)errorAngleI[axis]*conf.pid[PIDLEVEL].I8)>>12;

        ITerm = ITermACC + ((ITerm-ITermACC)*prop>>9);
        PTerm = PTermACC + ((PTerm-PTermACC)*prop>>9);
      }

      // 32 bits is needed for calculation   
      PTerm -= ((int32_t)imu.gyroData[axis]*dynP8[axis])>>6;
      
      // 16 bits is ok here, the dif between 2
      // consecutive gyro reads is limited to 800
      delta          = imu.gyroData[axis] - lastGyro[axis];
      lastGyro[axis] = imu.gyroData[axis];
      DTerm          = delta1[axis]+delta2[axis]+delta;
      delta2[axis]   = delta1[axis];
      delta1[axis]   = delta;

      // 32 bits is needed for calculation
      DTerm = ((int32_t)DTerm*dynD8[axis])>>5;

      axisPID[axis] =  PTerm + ITerm - DTerm;
    }

    //YAW
    #define GYRO_P_MAX 300
    #define GYRO_I_MAX 250

    rc = (int32_t)rcCommand[YAW] * (2*conf.yawRate + 30)  >> 5;

    error = rc - imu.gyroData[YAW];
    errorGyroI_YAW  += (int32_t)error*conf.pid[YAW].I8;
    errorGyroI_YAW  = constrain(errorGyroI_YAW, 2-((int32_t)1<<28), -2+((int32_t)1<<28));
    if (abs(rc) > 50) errorGyroI_YAW = 0;
    
    PTerm = (int32_t)error*conf.pid[YAW].P8>>6;

    ITerm = constrain((int16_t)(errorGyroI_YAW>>13),-GYRO_I_MAX,+GYRO_I_MAX);

    axisPID[YAW] =  PTerm + ITerm;

  #elif PID_CONTROLLER == 2
    #define GYRO_I_MAX 256
    #define ACC_I_MAX 256
    // range [0;500]
    prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500);

    // PID controller
    for(axis=0;axis<3;axis++) {
      // Get the desired angle rate depending on flight mode
      if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2 ) { // MODE relying on ACC
        // calculate error and limit the angle to 50 degrees max inclination
        // 16 bits is ok here
        errorAngle = constrain((rcCommand[axis]<<1) + GPS_angle[axis],-500,+500)
                   - att.angle[axis] + conf.angleTrim[axis];
      }

      // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
      if (axis == 2) {
        AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[2]) >> 5);
      } else {
        // control is GYRO based 
        // ACRO and HORIZON - direct sticks control is applied to rate PID
        if (!f.ANGLE_MODE) {
          AngleRateTmp =((int32_t) (conf.rollPitchRate + 27)
                       * rcCommand[axis]) >> 4;
          if (f.HORIZON_MODE) {
            // mix up angle error to desired AngleRateTmp 
            // to add a little auto-level feel
            AngleRateTmp += ((int32_t) errorAngle * conf.pid[PIDLEVEL].I8)>>8;
          }
        // it's the ANGLE mode - control is angle based,
        // so control loop is needed
        } else {
          AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8)>>4;
        }
      }

      // low-level gyro-based PID
      // Used in stand-alone mode for ACRO,
      // controlled by higher level regulators in other modes

      // calculate scaled error.AngleRates
      // multiplication of rcCommand corresponds to
      // changing the sticks scaling here
      RateError = AngleRateTmp - imu.gyroData[axis];

      // calculate P component
      PTerm = ((int32_t) RateError * conf.pid[axis].P8)>>7;

      // calculate I component
      //there should be no division before accumulating the error
      //to integrator, because the precision would be reduced.
      //Precision is critical, as I prevents from long-time drift.
      //Thus, 32 bits integrator is used. Time correction
      //(to avoid different I scaling for different builds
      //based on average cycle time) is normalized to cycle time = 2048.
      errorGyroI[axis] += (((int32_t) RateError * cycleTime)>>11)
                       * conf.pid[axis].I8;

      //limit maximum integrator value to prevent WindUp -
      //accumulating extreme values when system is saturated.
      //I coefficient (I8) moved before integration to make
      //limiting independent from PID settings
      errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t)
                        - GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);
      ITerm = errorGyroI[axis]>>13;

      // calculate D-term
      // 16 bits is ok here, the dif between 2 consecutive
      // gyro reads is limited to 800
      delta = RateError - lastError[axis];
      lastError[axis] = RateError;

      // Correct difference by cycle time. Cycle time is jittery
      // (can be different 2 times), so calculated difference would
      // be scaled by different dt each time. Division by dT fixes that.
      delta = ((int32_t) delta * ((uint16_t)0xFFFF / (cycleTime>>4)))>>6;
      // add moving average here to reduce noise
      deltaSum       = delta1[axis]+delta2[axis]+delta;
      delta2[axis]   = delta1[axis];
      delta1[axis]   = delta;

      DTerm = (deltaSum*conf.pid[axis].D8)>>8;

      // calculate total PID output
      axisPID[axis] =  PTerm + ITerm + DTerm;
    }
  #else
    #error "*** you must set PID_CONTROLLER to one existing implementation"
  #endif
  mixTable();
  // do not update servos during unarmed calibration of
  // sensors which are sensitive to vibration
  writeMotors();
}
