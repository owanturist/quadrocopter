#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"


// GLOBAL RX RELATED VARIABLES

// RAW RC values will be store here
// interval [1000;2000]
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502,
  1502, 1502, 1502, 1502
};

// Standard Channel order
static uint8_t rcChannel[RC_CHANS]  = {   \
  ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, \
  AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN         \
};

// if this slowes the PCINT readings we'
// can switch to a define for each pcint bit
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};

void rxInt(void);

// RX PIN SETUP
void configureReceiver() {
  // Configure each rc pin for PCINT
  // PCINT activation
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){
    PCINT_RX_PORT |= PCInt_RX_Pins[i];
    PCINT_RX_MASK |= PCInt_RX_Pins[i];
  }
  PCICR = PCIR_PORT_BIT;
}

// STANDARD RX PINS READING
// predefined PC pin block (thanks to lianj)  - Version without failsafe
#define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
  if (mask & PCInt_RX_Pins[pin_pos]) {                             \
    if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
      dTime = cTime-edgeTime[pin_pos];                             \
      if (900<dTime && dTime<2200) {                               \
        rcValue[rc_value_pos] = dTime;                             \
      }                                                            \
    } else edgeTime[pin_pos] = cTime;                              \
  }

// PORT CHANGE INTERRUPT
// this ISR is common to every receiver channel,
// it is call everytime a change state occurs on a RX input pin
ISR(RX_PC_INTERRUPT) {
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;

  // RX_PCINT_PIN_PORT indicates the state of each
  // PIN for the arduino port dealing with Ports digital pins
  pin = RX_PCINT_PIN_PORT;
 
  // doing a ^ between the current interruption and the
  // last one indicates wich pin changed
  mask = pin ^ PCintLast;

  // micros() return a uint32_t, but it is not usefull to
  // keep the whole bits => we keep only 16 bits
  cTime = micros();

  // re enable other interrupts at this point, the rest of this
  // interrupt is not so time critical and can be interrupted safely
  sei();

  // we memorize the current state of all PINs [D0-D7]
  PCintLast = pin;

  #if (PCINT_PIN_COUNT > 0)
    RX_PIN_CHECK(0,2);
  #endif
  #if (PCINT_PIN_COUNT > 1)
    RX_PIN_CHECK(1,4);
  #endif
  #if (PCINT_PIN_COUNT > 2)
    RX_PIN_CHECK(2,5);
  #endif
  #if (PCINT_PIN_COUNT > 3)
    RX_PIN_CHECK(3,6);
  #endif
  #if (PCINT_PIN_COUNT > 4)
    RX_PIN_CHECK(4,7);
  #endif
  #if (PCINT_PIN_COUNT > 5)
    RX_PIN_CHECK(5,0);
  #endif
  #if (PCINT_PIN_COUNT > 6)
    RX_PIN_CHECK(6,1);
  #endif
  #if (PCINT_PIN_COUNT > 7)
    RX_PIN_CHECK(7,3);
  #endif
}


// COMBINE AND SORT THE RX DATAS
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli();              // Let's disable interrupts
  data = rcValue[rcChannel[chan]];    // Let's copy the data Atomically
  SREG = oldSREG;                     // Let's restore interrupt state
  // We return the value correctly copied when the IRQ's where disabled
  return data;
}

// COMPUTE AND FILTER THE RX DATA
void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][4];
  static uint16_t rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;

  if (++rc4ValuesIndex == 4) {
    rc4ValuesIndex = 0;
  }

  for (uint8_t chan = 0; chan < RC_CHANS; chan++) {
    rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
    rcDataMean[chan] = 0;

    for (uint8_t a=0;a<4;a++) {
      rcDataMean[chan] += rcData4Values[chan][a];
    }

    rcDataMean[chan]= (rcDataMean[chan]+2)>>2;

    if ( rcDataMean[chan] < (uint16_t)rcData[chan] - 3) {
      rcData[chan] = rcDataMean[chan] + 2;
    } else if ( rcDataMean[chan] > (uint16_t)rcData[chan] + 3) {
      rcData[chan] = rcDataMean[chan] - 2;
    }

    // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
    if (chan < 8 && rcSerialCount > 0) {
      rcSerialCount --;
      // only relevant channels are overridden
      if (rcSerial[chan] > 900) {
        rcData[chan] = rcSerial[chan];
      }
    }
  }
}
