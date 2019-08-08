/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.
 */

#include "mpu.h"

Semaphore bootMPU(0);
Semaphore watchdogMPU;

#include "display.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "RunningMedian.h"
RunningMedian roll_samples_filtered   = RunningMedian(5);
RunningMedian roll_samples_unfiltered = RunningMedian(5);


// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

MPU6050 mpu;

float         max_roll_filtered[DATA_SAMPLES_ROLL_FILTERED];
unsigned int  max_roll_filtered_recent_pointer;
float         max_roll_filtered_trip_left, max_roll_filtered_trip_right;

float         max_roll_unfiltered[DATA_SAMPLES_ROLL_UNFILTERED];
unsigned  int max_roll_unfiltered_recent_pointer;

void resetMaxRollFiltered(){
  for (int i = 0; i < DATA_SAMPLES_ROLL_FILTERED; i++)
    max_roll_filtered[i] = 0;
  max_roll_filtered_recent_pointer = 0;
  max_roll_filtered_trip_left = 0;
  max_roll_filtered_trip_right = 0;
}

void setupMPU() {
  uint8_t devStatus;

  _PL("MPU\tSetup starting...")

  mpu.initialize(); 

  if (!mpu.testConnection()) {
    _PL("MPU\tmpu.testConnection() failed");
    displayStateSetFatal("MPU dead ?");
    delay(2000);
    _PL("MPU\tPower down")
    shutdown();
  }
  
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(61);
  mpu.setYGyroOffset(-28);
  mpu.setZGyroOffset(-6);
  mpu.setZAccelOffset(1471);

  if (devStatus == 0) {

    // See new library version
        // Calibration Time: generate offsets and calibrate our MPU6050
//        mpu.CalibrateAccel(6);
//        mpu.CalibrateGyro(6);
//        mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  } else {
    _PL("MPU\tmpu.dmpInitialize() failed");
    displayStateSetFatal("MPU dead ?");
    delay(2000);
    _PL("MPU\tPower down")
    shutdown();
  }

  max_roll_filtered_trip_left = 0;
  max_roll_filtered_trip_right = 0;
  max_roll_filtered_recent_pointer = 0;
  for (int i = 0; i < DATA_SAMPLES_ROLL_FILTERED; i++)
    max_roll_filtered[i] = 0;

  max_roll_unfiltered_recent_pointer = 0;
  for (int i = 0; i < DATA_SAMPLES_ROLL_UNFILTERED; i++)
    max_roll_unfiltered[i] = 0;

  _PL("MPU\tStarted")

  bootMPU.signal();

  while(!bootOk) // Wait for other threads
    yield();
}

void loopMPU() {
  uint8_t     fifoBuffer[64];
  static int  samples_skipped_init = 0;
  static int  samples_skipped = 0;

  watchdogMPU.signal();

  // Wait for DMP data...
  while (!(mpu.getIntStatus() & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)))
    yield();
    
  // Rest of the loop runs in some 5340us, at approx 92 samples per second max...
  while (mpu.getFIFOCount() < packetSize)
    yield();

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.resetFIFO();
  
  static float previous_roll = 0;
  
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
  yield();

  // Keep a reccord of unfiltered roll, including any weird MPU data
  // Store several values from here out to SD card, for later weird/noise cleanup
  max_roll_unfiltered[max_roll_unfiltered_recent_pointer++] = abs(ypr[2]);
  if (max_roll_unfiltered_recent_pointer == DATA_SAMPLES_ROLL_UNFILTERED)
    max_roll_unfiltered_recent_pointer = 0;

  // Keep a reccord of filtered roll, excluding any weird MPU data
  if (roll_samples_filtered.getCount() < 3) {
    // Initialize... filtering out the occasionnal MPU6050 weird data
    roll_samples_unfiltered.add(ypr[2]);
    samples_skipped_init++;
    if (roll_samples_unfiltered.getCount() >= 3) {
      if ((abs(roll_samples_unfiltered.getHighest() - roll_samples_unfiltered.getLowest()) < 6 * M_PI/180)) {
        previous_roll = ypr[2];
        roll_samples_filtered.add(ypr[2]);
        _PP("MPU\tInitialise ");
        _PL(ypr[2] * 180/M_PI);
        if (roll_samples_filtered.getCount() == 3) { // TODO shall display blink so long?
          _PP("MPU\tInitialised, ");
          _PP(samples_skipped_init);
          _PL(" samples where skipped");
          samples_skipped_init = 0;
        }
      } else {
        roll_samples_filtered.clear();
        roll_samples_unfiltered.clear();
       }
    }
  
  } else {  
    if (abs(previous_roll - ypr[2]) > 6 * M_PI/180) {
      // filter the occasionnal huge weird data out...
      if (samples_skipped++ < 3) {
        _PP("MPU\tRoll oops, skipped, was ");
        _PL(ypr[2] * 180/M_PI);
      } else if (samples_skipped >= 3) {
        // We rolled away or weird things happened...
        _PP("MPU\tRoll oops, skipped, was ");
        _PL(ypr[2] * 180/M_PI);
        _PL("MPU\tReinit roll buffer");
        samples_skipped_init = 0;
        roll_samples_filtered.clear();
        roll_samples_unfiltered.clear();
      }
    } else {
      samples_skipped = 0;

      // Check and record roll samples
      previous_roll = ypr[2];

// TODO Check this. Would denoise but  won't allow to sample/display max...
//      roll_samples_filtered.add(ypr[2]);
//      float current_roll = roll_samples_filtered.getMedian();

      float current_roll = ypr[2];
      max_roll_filtered[max_roll_filtered_recent_pointer++] = abs(current_roll);
      if (max_roll_filtered_recent_pointer == DATA_SAMPLES_ROLL_FILTERED)
        max_roll_filtered_recent_pointer = 0;
      if (current_roll > 0 )
        max_roll_filtered_trip_right = max(current_roll, max_roll_filtered_trip_right);
      if (current_roll < 0)
        max_roll_filtered_trip_left = max(abs(current_roll), max_roll_filtered_trip_left);
    }

#ifdef __DEBUG_
    static int samples = 0;
    
    static unsigned long  previousMillisSamples = 0;

    samples++;
    
    unsigned long currentMillisSamples = millis();
    if (currentMillisSamples - previousMillisSamples > 10000) {
      previousMillisSamples = currentMillisSamples;
      _PP("MPU\tRate was ");
      _PP(((float)samples) / 10);
      _PL(" samples per sec");
      samples = 0;
    }
    
#endif

#ifdef __DEBUG_MPU_ROLL
    if (!(sample % 100)) {
      float roll = 0;
      _PP(ypr[2] * 180/M_PI);
      for (int i = 0; i < DATA_SAMPLES_ROLL_FILTERED; i++)
      roll = max(roll, max_roll_filtered[i]);

      _PP(", roll max ");
      _PP(roll * 180/M_PI);
      _PP(" ");
      _PP(max_roll_filtered_trip_left * 180/M_PI);
      _PP("/");
      _PL(max_roll_filtered_trip_right * 180/M_PI);
    }
#endif // __DEBUG_MPU_ROLL
  }

  yield();

#ifdef __DEBUG_STACK
  static unsigned long  previousMillisStack = 0;
 
  unsigned long currentMillisStack = millis();
  if (currentMillisStack - previousMillisStack > 5000) {
    previousMillisStack = currentMillisStack;
    _PP("loopMPU\t\tfree stack ");
    _PL(Scheduler.stack());
#endif // __DEBUG_STACK
}
