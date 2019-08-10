
/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.
 */
 
/*  Using https://github.com/greiman/ChRt
    ChibiOS/RT for Arduino AVR, SAMD, Due, Teensy 3.x. 

    With a Teensy 3.2, max short RF24network packet rate is approx 27 packets/second

    Output example with RF24 At 10pps, MPU6050 at 100 samples per second:

MPU  Configuring...
MPU Waiting......................
MPU Ok
Radio Configuring...
Radio Ok
MPU 100/sec, 100 min, 100 max, 1661us
MPU 0.04, 0.00 min, 0.05 max
Radio 52 out, 0 failed, 11pps
Stack 100 760 1020 164 52648 312
...
MPU  100/sec, 100 min, 100 max, 1661us
MPU 0.06, 0.00 min, 0.08 max
Radio 3052 out, 0 failed, 10pps
Stack 100 752 948 164 52648 300
MPU 100/sec, 100 min, 100 max, 1657us
MPU 0.06, 0.00 min, 0.08 max
Radio 3062 out, 0 failed, 10pps
Stack 100 752 948 164 52648 300

    With RF24 at some 27-30pps, close to max, MPU6050 at 100 samples per second:

MPU  100/sec, 100 min, 100 max, 1678us
MPU 0.05, 0.00 min, 0.08 max
Radio 8802 out, 0 failed, 27pps
Stack 100 752 956 128 52648 248
MPU 100/sec, 100 min, 100 max, 1675us
MPU 0.06, 0.00 min, 0.08 max
Radio 8836 out, 0 failed, 31pps
Stack 100 752 956 128 52648 248
MPU 100/sec, 100 min, 100 max, 1677us
MPU 0.06, 0.00 min, 0.08 max
Radio 8869 out, 0 failed, 29pps
Stack 100 752 956 128 52648 248

*/

#include "ChRt.h"

SEMAPHORE_DECL(serialFree, 0);
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);

#include <SPI.h>
#include <Wire.h>

#undef  PIN_SPI_SCK // Teensy 3.2 LED is on PIN 13
#define PIN_SPI_SCK   14 // We will use 14
#define INTERRUPT_PIN 2

#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
volatile uint8_t  packetSize;    // expected DMP packet size (default is 42 bytes)
volatile uint8_t  mpuDataRate = 0;
volatile uint16_t mpuFIFOReset;
volatile uint16_t mpuExecute;
SEMAPHORE_DECL(mpuDataFree, 0);

volatile float          roll;
volatile float          roll_min;
volatile float          roll_max;
volatile unsigned long  roll_oops;

#include <RF24Network.h>
#include <RF24.h>
#define CSN_RF24           16
#define CE_RF24            15
RF24 radio(CE_RF24, CSN_RF24); 
RF24Network network(radio);
#define NETID "Hj9G"
const uint16_t this_node = 00; 
#include <FastCRC.h>
FastCRC16 CRC16;
#include <time.h>
#include <TimeLib.h>

struct payload_t {
  uint16_t      crc16;
  char          netid[5];
  unsigned long stamp;
  unsigned long counter;
  char data[175]; // Payload size : 192
};

struct packet_t {
  unsigned char type;
  payload_t     payload;
};

uint16_t payloadCRC (payload_t payload) {
  char  buffer[200];
  sprintf(buffer, "%2s%lu%lu%s", payload.netid, payload.stamp, payload.counter, payload.data);
  return CRC16.ccitt((uint8_t *)buffer, strlen(buffer));
}

unsigned long packets_out_multicast;
unsigned long packets_out_failed;
unsigned long radioPackets;

void packetBuildMulticast(packet_t *packet, char *data, unsigned char type = 0) {
  strcpy(packet->payload.netid, NETID);
  packet->payload.stamp = now();
  packet->payload.counter = ++packets_out_multicast;
  strcpy(packet->payload.data, data);
  packet->payload.crc16 = payloadCRC(packet->payload);
  packet->type = type;
}

bool packetSendMulticast(packet_t *packet) {
  RF24NetworkHeader header( /* multicast */ 64, packet->type);
  bool ok = network.multicast(header, &packet->payload, sizeof(packet->payload), /* level */ 01);
  if (!ok)
    packets_out_failed++;
  return ok;
}

//------------------------------------------------------------------------------
static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    chThdSleepMilliseconds(50);
    digitalWrite(LED_BUILTIN, LOW);
    chThdSleepMilliseconds(150);
  }
}
//------------------------------------------------------------------------------
static thread_reference_t trpMPU = NULL;

CH_IRQ_HANDLER(myIRQMPU) {
  CH_IRQ_PROLOGUE();
 
  /* Wakes up the thread.*/
  chSysLockFromISR();
  chThdResumeI(&trpMPU, (msg_t)0x1337);  /* Resuming the thread with message.*/
  chSysUnlockFromISR();
 
  CH_IRQ_EPILOGUE();
}
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waThread2, 1024);
static THD_FUNCTION(Thread2, arg) {
  (void)arg;
  while (true) {
    //msg_t msg;
 
    /* Waiting for the IRQ to happen.*/
    chSysLock();
    //msg = chThdSuspendS(&trpMPU);
    chThdSuspendS(&trpMPU);
    chSysUnlock();
    
    unsigned long stamp;
    stamp = micros();  

    uint16_t fifoCount;
    while ((fifoCount = mpu.getFIFOCount()) < packetSize);
  
    uint8_t mpuIntStatus = mpu.getIntStatus();
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount > packetSize) { // was >= 1024) {
      mpu.resetFIFO();
      mpuFIFOReset++;
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      uint8_t fifoBuffer[64];

      mpu.getFIFOBytes(fifoBuffer, packetSize);

      Quaternion q;           // [w, x, y, z]         quaternion container
      VectorFloat gravity;    // [x, y, z]            gravity vector
      float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      static float previous_roll;

      if (abs(roll - previous_roll) > 2 * M_PI/180) {
        roll = previous_roll;
        roll_oops++;
        mpu.resetFIFO();
        mpuFIFOReset++;
      }

      roll = ypr[2];
      if (ypr[2] > 0)
        roll_max = max(roll_max, ypr[2]);
      if (ypr[2] < 0)
        roll_min = max(roll_min, abs(ypr[2]));
    
      static uint8_t mpuDMPInterrupts = 0;
      static unsigned long last_millis = millis();
      unsigned long now_millis = millis();
      if (now_millis - last_millis >= 1000) {
        chSemWait(&mpuDataFree);
        mpuDataRate = mpuDMPInterrupts;
        chSemSignal(&mpuDataFree);
        mpuDMPInterrupts = 0;
        last_millis = now_millis;
      }
      ++mpuDMPInterrupts;
    }
    chSemWait(&mpuDataFree);
    mpuExecute = max(micros() - stamp, mpuExecute);
    chSemSignal(&mpuDataFree);
  }
}
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waThread3, 2048);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  systime_t time = chVTGetSystemTimeX();
  while (true) {
    packet_t packet;
    
/*  Lines for 10pps:
    time += MS2ST(100);
    packetBuildMulticast(&packet, (char *)"Hello!", 0);
    chThdYield();
    packetSendMulticast(&packet);
    radioPackets++;
    chThdSleepUntil(time); */

    // Lines for some 27-30 pps:
    packetBuildMulticast(&packet, (char *)"Hello!", 0);
    chThdYield();
    packetSendMulticast(&packet);
    radioPackets++;
    chThdSleepMilliseconds(5);
  }
}
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waThread4, 256);
static THD_FUNCTION(Thread4, arg) {
  (void)arg;
  uint8_t data_rate_min = 100;
  uint8_t data_rate_max = 0;
  uint8_t wait = 0;
  systime_t time = chVTGetSystemTimeX();
  while (true) {
    time += MS2ST(1000);
    if (wait < 5) { // Skip initial data
      wait++;
      roll_min = 0; // TODO missing semaphores...
      roll_max = 0;
      roll_oops = 0;
    } else {
      uint8_t data_rate_current;
      uint32_t data_execute_current;
      chSemWait(&mpuDataFree);
      data_rate_current = mpuDataRate;
      data_execute_current = mpuExecute;
      mpuExecute = 0;
      chSemSignal(&mpuDataFree);
      data_rate_min = min(data_rate_min, data_rate_current);
      data_rate_max = max(data_rate_max, data_rate_current);
      
      chSemWait(&serialFree);
      _PP("MPU\t");
      _PP(data_rate_current);
      _PP("/sec, ");
      _PP(data_rate_min);
      _PP(" min, ");
      _PP(data_rate_max);
      _PP(" max, ");
      _PP(data_execute_current);
      _PP("us");
      if (data_rate_current == 100) {
        _PL("");
      } else {
        _PL(", oops...");
      }
      chSemSignal(&serialFree);
      chThdSleepMilliseconds(50);
      chSemWait(&serialFree);
      _PP("MPU\t");
      _PP(roll * 180/M_PI);
      _PP(", ");
      _PP(roll_min * 180/M_PI);
      _PP(" min, ");
      _PP(roll_max * 180/M_PI);
      _PP(" max");
      if (roll_oops) {
        _PP(", ");
        _PP(roll_oops);
        _PP(" oopsed, ");
        _PP(mpuFIFOReset);
        _PP(" FIFO resets");
      }
      _PL("");
      chSemSignal(&serialFree);
      chThdSleepMilliseconds(50);
      chSemWait(&serialFree);
      _PP("Radio\t");
      _PP(packets_out_multicast);
      _PP(" out, ");
      _PP(packets_out_failed);
      _PP(" failed, ");
      _PP(radioPackets); // TODO to be probed inside senter loop (see mpuDataRate / mpuDMPInterrupts)
      _PL("pps");
      chSemSignal(&serialFree);
#define __DEBUG_STACK
#ifdef __DEBUG_STACK
      chThdSleepMilliseconds(50);
      chSemWait(&serialFree);
      // Print unused stack space in bytes.
      _PP(F("Stack\t"));
      _PP(chUnusedThreadStack(waThread1, sizeof(waThread1)));
      _PP(" ");
      _PP(chUnusedThreadStack(waThread2, sizeof(waThread2)));
      _PP(" ");
      _PP(chUnusedThreadStack(waThread3, sizeof(waThread3)));
      _PP(" ");
      _PP(chUnusedThreadStack(waThread4, sizeof(waThread4)));
      _PP(" ");
      _PP(chUnusedMainStack());
#ifdef __arm__
      // ARM has separate stack for ISR interrupts. 
      _PP(" ");
      _PP(chUnusedHandlerStack());
#endif  // __arm__
      _PL();
      chSemSignal(&serialFree);
#endif // __DEBUG_STACK
    }
    radioPackets = 0; // TODO missing semaphores... plus to be probed inside sender loop
    chThdSleepUntil(time);
  }
}

//------------------------------------------------------------------------------
// Continue setup() after chBegin().
void chSetup() {
  if (CH_CFG_TIME_QUANTUM != 2) {
    _PL("You must set CH_CFG_TIME_QUANTUM to 2 in");
#if defined(__arm__)
    _PP("src/arm/chconfig_arm.h");
#elif defined(__AVR__)
    _PP(F("src/avr/chconfig_avr.h")); 
#endif 
    while (true) {}
  }

  chSemSignal(&serialFree);
  chSemSignal(&mpuDataFree);

  // LED
  chThdCreateStatic(waThread1, sizeof(waThread1),
    NORMALPRIO - 4, Thread1, NULL);

  // MPU
  chThdCreateStatic(waThread2, sizeof(waThread2),
    NORMALPRIO + 2, Thread2, NULL);

  // Radio
  chThdCreateStatic(waThread3, sizeof(waThread3),
    NORMALPRIO - 2, Thread3, NULL);

  // Summary
  chThdCreateStatic(waThread4, sizeof(waThread4),
    NORMALPRIO - 4, Thread4, NULL);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), myIRQMPU, RISING);
  mpu.getIntStatus();
}
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // Wait for USB Serial.
  while (!Serial) {}

  SPI.setSCK(PIN_SPI_SCK);
  SPI.begin();
  
  Wire.begin(); // Dedicated to MPU
  Wire.setClock(400000L);
  
  _PL("MPU\tConfiguring...");
  uint8_t devStatus;
  mpu.reset();
  delay(100);
  mpu.initialize(); 
  if (!mpu.testConnection())
    while (true) {}
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);    
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    _PL("MPU\tDead");
    while (true) {};
  }
  mpu.setXGyroOffset(61);
  mpu.setYGyroOffset(-28);
  mpu.setZGyroOffset(-6);
  mpu.setZAccelOffset(1471);
  _PP("MPU\tWaiting..");
  for (int i = 0; i < 20; i++) { // Initial values are out of range (MPU calibrating?)
    delay(1000);
    _PP(".");
  }
  _PL("");
  _PL("MPU\tOk");
   
  _PL("Radio\tConfiguring...");
  radio.begin();
  
  if (!radio.isChipConnected() || radio.failureDetected) {
    _PL("Radio\tDead");
    while (true) {};
  } else {
    radio.setDataRate( RF24_250KBPS ); // Some 34 packets/s, slower to process...
    //radio.setDataRate( RF24_1MBPS ); // Some 50 packets/s...
    radio.setPALevel(RF24_PA_MIN);
    network.begin(/* channel */90, this_node);
  }
  _PL("Radio\tOk");
  
  // Start ChibiOS.
  chBegin(chSetup);
  // chBegin() resets stacks and should never return.
  while (true) {}  
}
//------------------------------------------------------------------------------
void loop() {
  while (true) {
    chThdSleepMilliseconds(60000);
  }
}
