#ifndef TYPES_H_
  #define TYPES_H_

  enum rc {
    ROLL,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8
  };

  enum pid {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
  };

  enum box {
    BOXARM,
    #if ACC
      BOXANGLE,
      BOXHORIZON,
    #endif
    #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
      BOXBARO,
    #endif
    #ifdef VARIOMETER
      BOXVARIO,
    #endif
    #if MAG
      BOXMAG,
      BOXHEADFREE,
      BOXHEADADJ, // acquire heading for HEADFREE mode
    #endif
    #if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
      BOXCAMSTAB,
    #endif
    #if defined(CAMTRIG)
      BOXCAMTRIG,
    #endif
    #if GPS
      BOXGPSHOME,
      BOXGPSHOLD,
    #endif
    #if defined(FIXEDWING) || defined(HELICOPTER)
      BOXPASSTHRU,
    #endif
    #if defined(BUZZER)
      BOXBEEPERON,
    #endif
    #if defined(LED_FLASHER)
      BOXLEDMAX, // we want maximum illumination
      BOXLEDLOW, // low/no lights
    #endif
    #if defined(LANDING_LIGHTS_DDR)
      BOXLLIGHTS, // enable landing lights at any altitude
    #endif
    #ifdef INFLIGHT_ACC_CALIBRATION
      BOXCALIB,
    #endif
    #ifdef GOVERNOR_P
      BOXGOV,
    #endif
    #ifdef OSD_SWITCH
      BOXOSD,
    #endif
    CHECKBOXITEMS
  };

  typedef struct {
    int16_t  accSmooth[3];
    int16_t  gyroData[3];
    int16_t  magADC[3];
    int16_t  gyroADC[3];
    int16_t  accADC[3];
  } imu_t;

  typedef struct {
    uint8_t  vbat;               // battery voltage in 0.1V steps
    uint16_t intPowerMeterSum;
    uint16_t rssi;               // range: [0;1023]
    uint16_t amperage;
  } analog_t;

  typedef struct {
    int32_t  EstAlt;             // in cm
    int16_t  vario;              // variometer in cm/s
  } alt_t;

  typedef struct {
    int16_t angle[2];            // absolute angle inclination
    int16_t heading;             // variometer in cm/s
  } att_t;

  typedef struct {
    uint8_t OK_TO_ARM:       1;
    uint8_t ARMED:           1;
    uint8_t I2C_INIT_DONE:   1;
    uint8_t ACC_CALIBRATED:  1;
    uint8_t NUNCHUKDATA:     1;
    uint8_t ANGLE_MODE:      1;
    uint8_t HORIZON_MODE:    1;
    uint8_t MAG_MODE:        1;
    uint8_t BARO_MODE:       1;
    uint8_t GPS_HOME_MODE:   1;
    uint8_t GPS_HOLD_MODE:   1;
    uint8_t HEADFREE_MODE:   1;
    uint8_t PASSTHRU_MODE:   1;
    uint8_t GPS_FIX:         1;
    uint8_t GPS_FIX_HOME:    1;
    uint8_t SMALL_ANGLES_25: 1;
    uint8_t CALIBRATE_MAG:   1;
    uint8_t VARIO_MODE:      1;
  } flags_struct_t;

  typedef struct {
    uint8_t currentSet;
    int16_t accZero[3];
    int16_t magZero[3];
    uint16_t flashsum;
    uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
  } global_conf_t;

  struct pid_ {
    uint8_t P8;
    uint8_t I8;
    uint8_t D8;
  };

  typedef struct {
    pid_    pid[PIDITEMS];
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t rollPitchRate;
    uint8_t yawRate;
    uint8_t dynThrPID;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    int16_t angleTrim[2];
    uint16_t activate[CHECKBOXITEMS];
    uint8_t powerTrigger1;
    #if MAG
      int16_t mag_declination;
    #endif
    #if defined(GYRO_SMOOTHING)
      uint8_t Smoothing[3];
    #endif
    #if defined (FAILSAFE)
      int16_t failsafe_throttle;
    #endif
    #ifdef VBAT
      uint8_t vbatscale;
      uint8_t vbatlevel_warn1;
      uint8_t vbatlevel_warn2;
      uint8_t vbatlevel_crit;
    #endif
    #ifdef POWERMETER
      uint8_t pint2ma;
    #endif
    #ifdef POWERMETER_HARD
      uint16_t psensornull;
    #endif
    #ifdef MMGYRO
      uint8_t mmgyro;
    #endif
    #ifdef ARMEDTIMEWARNING
      uint16_t armedtimewarning;
    #endif
    int16_t minthrottle;
    #ifdef GOVERNOR_P
     int16_t governorP;
     int16_t governorD;
    #endif
    uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
  } conf_t;

#endif /* TYPES_H_ */
