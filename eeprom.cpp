#include <avr/eeprom.h>
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

void LoadDefaults(void);

uint8_t calculateSum(uint8_t *cb , uint8_t siz) {
  uint8_t sum = 0x55;  // checksum init
  while(--siz) {
    sum += *cb++;      // calculate checksum (without checksum byte)
  }
  return sum;
}

void readGlobalSet() {
  uint8_t *cb = (uint8_t*)&global_conf;
  uint8_t siz = sizeof(global_conf);

  eeprom_read_block((void*)&global_conf, (void*)0, siz);

  if(calculateSum(cb, siz) != global_conf.checksum) {
    global_conf.currentSet = 0;
    global_conf.accZero[ROLL] = 5000;    // for config error signalization
  }
}

bool readEEPROM() {
  uint8_t i;
  global_conf.currentSet = 0;

  eeprom_read_block(
    (void*)&conf,
    (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)),
    sizeof(conf)
  );

  if(calculateSum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
    // force load defaults 
    LoadDefaults();
    // defaults loaded, don't reload constants (EEPROM life saving)
    return false;
  }

  for(i = 0; i < 5; i++) {
    int32_t expo = 1526 + conf.rcExpo8 * (i * i - 15);
    int32_t rate = (int32_t)conf.rcRate8;
    lookupPitchRollRC[i] = expo * i * rate / 1992;
  }

  for(i = 0; i < 11; i++) {
    int16_t tmp = 10 * i - conf.thrMid8;
    uint8_t y = 1;
    int32_t scale;
    int32_t expo;
    int32_t maxthr;

    if (tmp > 0) {
      y = 100 - conf.thrMid8;
    } else if (tmp < 0) {
      y = conf.thrMid8;
    }

    // [0;1000]
    expo   = (int32_t)conf.thrExpo8 * (tmp * tmp) / (y * y);
    scale  = 10 * conf.thrMid8 + tmp * (100 - conf.thrExpo8 + expo ) / 10;
    maxthr = (int32_t)(MAXTHROTTLE-conf.minthrottle) * scale / 1000;

    // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
    lookupThrottleRC[i] = conf.minthrottle + maxthr;
  }

  return true;  // setting is OK
}

void writeGlobalSet(uint8_t b) {
  uint8_t *cb = (uint8_t*)&global_conf;
  uint8_t siz = sizeof(global_conf);
  global_conf.checksum = calculateSum(cb, siz);

  eeprom_write_block(
    (const void*)&global_conf,
    (void*)0,
    sizeof(global_conf)
  );
}

void writeParams(uint8_t b) {
  global_conf.currentSet = 0;

  conf.checksum = calculateSum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block(
    (const void*)&conf,
    (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)),
    sizeof(conf)
  );

  readEEPROM();
}

void updateConstants() { 
  #ifdef GYRO_SMOOTHING
    uint8_t s[3] = GYRO_SMOOTHING;
    for(uint8_t i=0; i < 3; i++) {
      conf.Smoothing[i] = s[i];
    }
  #endif

  writeParams(0);
}

void LoadDefaults() {
  #if PID_CONTROLLER == 1
    conf.pid[ROLL].P8 = 33;
    conf.pid[ROLL].I8 = 30;
    conf.pid[ROLL].D8 = 23;

    conf.pid[PITCH].P8 = 33;
    conf.pid[PITCH].I8 = 30;
    conf.pid[PITCH].D8 = 23;

    conf.pid[PIDLEVEL].P8 = 90;
    conf.pid[PIDLEVEL].I8 = 10;
    conf.pid[PIDLEVEL].D8 = 100;

  #elif PID_CONTROLLER == 2
    conf.pid[ROLL].P8 = 28;
    conf.pid[ROLL].I8 = 10;
    conf.pid[ROLL].D8 = 7;

    conf.pid[PITCH].P8 = 28;
    conf.pid[PITCH].I8 = 10;
    conf.pid[PITCH].D8 = 7;

    conf.pid[PIDLEVEL].P8 = 30;
    conf.pid[PIDLEVEL].I8 = 32;
    conf.pid[PIDLEVEL].D8 = 0;
  #endif

  conf.pid[YAW].P8 = 68;
  conf.pid[YAW].I8 = 45;
  conf.pid[YAW].D8 = 0;

  conf.pid[PIDALT].P8 = 64;
  conf.pid[PIDALT].I8 = 25;
  conf.pid[PIDALT].D8 = 24;

  conf.pid[PIDPOS].P8 = POSHOLD_P * 100;
  conf.pid[PIDPOS].I8 = POSHOLD_I * 100;
  conf.pid[PIDPOS].D8 = 0;

  conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10;
  conf.pid[PIDPOSR].I8 = POSHOLD_RATE_I * 100;
  conf.pid[PIDPOSR].D8 = POSHOLD_RATE_D * 1000;

  conf.pid[PIDNAVR].P8 = NAV_P * 10;
  conf.pid[PIDNAVR].I8 = NAV_I * 100;
  conf.pid[PIDNAVR].D8 = NAV_D * 1000;

  conf.pid[PIDMAG].P8   = 40;

  conf.pid[PIDVEL].P8 = 0;
  conf.pid[PIDVEL].I8 = 0;
  conf.pid[PIDVEL].D8 = 0;

  conf.rcRate8       = 90;
  conf.rcExpo8       = 65;
  conf.rollPitchRate = 0;
  conf.yawRate       = 0;
  conf.dynThrPID     = 0;
  conf.thrMid8       = 50;
  conf.thrExpo8      = 0;

  for(uint8_t i=0; i<CHECKBOXITEMS; i++) {
    conf.activate[i] = 0;
  }

  conf.angleTrim[0] = 0;
  conf.angleTrim[1] = 0;
  conf.powerTrigger1 = 0;


  // this will also write to eeprom
  updateConstants();
}
