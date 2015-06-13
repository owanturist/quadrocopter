#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"

void getEstimatedAttitude();

void computeIMU () {
  uint8_t axis;
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static int16_t gyroADCprevious[3] = {0,0,0};
  static uint32_t timeInterleave = 0;

  #if ACC
    accGetADC();
    getEstimatedAttitude();
  #endif

  #if GYRO
    gyroGetADC();
  #endif

  for (axis = 0; axis < 3; axis++) {
    gyroADCp[axis] =  imu.gyroADC[axis];
  }

  timeInterleave = micros();
  annexCode();
  uint8_t _time = 0;

  //empirical, interleaving delay between 2 consecutive reads
  while((uint16_t)(micros() - timeInterleave) < 650) {
    _time = 1;
  }

  if (!_time) {
    annex650_overrun_count++;
  }

  #if GYRO
    gyroGetADC();
  #endif

  for (axis = 0; axis < 3; axis++) {
    gyroADCinter[axis] =  imu.gyroADC[axis] + gyroADCp[axis];
    // empirical, we take a weighted
    // value of the current and the previous values
    imu.gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
    gyroADCprevious[axis] = gyroADCinter[axis] >> 1;

    if (!ACC) {
      imu.accADC[axis] = 0;
    }
  }

  #if defined(GYRO_SMOOTHING)
    static int16_t gyroSmooth[3] = {0,0,0};
    int32_t smoothing, gyroData;

    for (axis = 0; axis < 3; axis++) {
      smoothing = (int32_t)gyroSmooth[axis] * (conf.Smoothing[axis] - 1);
      gyroData = (int32_t)(smoothing) + imu.gyroData[axis] + 1;
      imu.gyroData[axis] = (int16_t)( gyroData / conf.Smoothing[axis]);
      gyroSmooth[axis] = imu.gyroData[axis];
    }
  #endif
}


// advanced users settings
/**
 * Set the Low Pass Filter factor for ACC Increasing this value would
 * reduce ACC noise (visible in GUI), but would increase ACC lag time
 * Comment this if  you do not want filter at all. unit = n power of 2
 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/**
 * Set the Gyro Weight for Gyro/Acc complementary filter Increasing this
 * value would reduce and delay Acc influence on the output of the filter
 */
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 600
#endif

/**
 * Set the Gyro Weight for Gyro/Magnetometer complementary filter
 * Increasing this value would reduce and delay Magnetometer
 * influence on the output of the filter
 */
#ifndef GYR_CMPFM_FACTOR
  #define GYR_CMPFM_FACTOR 250
#endif
// end of advanced users settings

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

typedef struct fp_vector {
  float X,Y,Z;
} t_fp_vector_def;

typedef union {
  float A[3];
  t_fp_vector_def V;
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if (abs(y) < abs(x)) {
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x < 0) {
      if (y < 0) {
        a -= 1800;
      } else {
        a += 1800;
      }
    }
  } else {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y < 0) {
      a -= 1800;
    }
  }
  return a;
}

float invSqrt (float x) {
  union {
    int32_t i;
    float   f;
  } conv;
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation,
// according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;
  static uint16_t previousT;
  uint16_t currentT = micros();

  // GYRO_SCALE unit: radian/microsecond
  scale = (currentT - previousT) * GYRO_SCALE;
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = imu.gyroADC[axis] * scale; // radian

    accLPF32[axis] -= accLPF32[axis] >> ACC_LPF_FACTOR;
    accLPF32[axis] += imu.accADC[axis];
    imu.accSmooth[axis] = accLPF32[axis] >> ACC_LPF_FACTOR;

    accMag += (int32_t)imu.accSmooth[axis] * imu.accSmooth[axis] ;
  }

  rotateV(&EstG.V,deltaGyroAngle);

  accMag = accMag * 100 / ( (int32_t)ACC_1G * ACC_1G);
  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;

  /**
   * Apply complimentary filter (Gyro drift correction) If accel
   * magnitude >1.15G or <0.85G and ACC vector outside of the limit
   * range => we neutralize the effect of accelerometers in the
   * angle estimation. To do that, we just skip filter, as EstV
   * already rotated by Gyro
   */
  for (axis = 0; axis < 3; axis++) {
    if ( validAcc ) {
      EstG.A[axis] *= GYR_CMPF_FACTOR;
      EstG.A[axis] += imu.accSmooth[axis];
      EstG.A[axis] *= INV_GYR_CMPF_FACTOR;
    }
    //int32_t cross calculation is a little bit faster than float
    EstG32.A[axis] = EstG.A[axis];
  }
  
  if ((int16_t)EstG32.A[2] > ACCZ_25deg) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  // Attitude of the estimated vector
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = invSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  att.angle[ROLL]  = _atan2(EstG32.V.X, EstG32.V.Z);
  att.angle[PITCH] = _atan2(EstG32.V.Y, invSqrt(sqGX_sqGZ) * sqGX_sqGZ);
}
