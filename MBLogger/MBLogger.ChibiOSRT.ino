
/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.
 */
 
/*  Using https://github.com/greiman/ChRt
    ChibiOS/RT for Arduino AVR, SAMD, Due, Teensy 3.x. 

    With a Teensy 3.2, outputs are (1 FIFO reset from startup):

MPU  : 100 samples/sec, 100 min, 101 max, 1 FIFO resets, last roll is 0.12, 0.00 min, 0.14 max
Radio : 3850 out, 0 failed, 50 packets/second
MPU : 100 samples/sec, 100 min, 101 max, 1 FIFO resets, last roll is 0.11, 0.00 min, 0.14 max
Radio : 3900 out, 0 failed, 50 packets/second
MPU : 100 samples/sec, 100 min, 101 max, 1 FIFO resets, last roll is 0.13, 0.00 min, 0.14 max
Radio : 3950 out, 0 failed, 50 packets/second
MPU : 100 samples/sec, 100 min, 101 max, 1 FIFO resets, last roll is 0.11, 0.00 min, 0.14 max
Radio : 4000 out, 0 failed, 50 packets/second
Stack : 100 724 192 724 52724 280
MPU : 100 samples/sec, 100 min, 101 max, 1 FIFO resets, last roll is 0.11, 0.00 min, 0.14 max
Radio : 4050 out, 0 failed, 50 packets/second
MPU : 100 samples/sec, 100 min, 101 max, 1 FIFO resets, last roll is 0.11, 0.00 min, 0.14 max
Radio : 4100 out, 0 failed, 50 packets/second

*/

#include "ChRt.h"

#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);

#include <SPI.h>
#include <Wire.h>

#undef  PIN_SPI_SCK // Teensy 3.2 LED is on PIN 13
#define PIN_SPI_SCK         14 // We will use 14

#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
volatile uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
volatile uint16_t mpuFIFOReset;
volatile uint32_t mpuData = 0;
volatile float    roll;
volatile float    roll_min;
volatile float    roll_max;

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
static THD_WORKING_AREA(waThread2, 1024);
static THD_FUNCTION(Thread2, arg) {
  (void)arg;
  while (true) {
    uint16_t fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
      uint8_t mpuIntStatus = mpu.getIntStatus();
      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        mpuFIFOReset++;
      } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {      
        uint8_t fifoBuffer[64];
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
      
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        roll = ypr[2];
        if (ypr[2] > 0)
          roll_max = max(roll_max, ypr[2]);
        if (ypr[2] < 0)
          roll_min = max(roll_min, abs(ypr[2]));
        mpuData++;
      }
    }
    chThdYield();
  }
}

//------------------------------------------------------------------------------
static THD_WORKING_AREA(waThread3, 64);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  unsigned long data_min = 100;
  unsigned long data_max = 0;
  unsigned long wait = 0;
  while (true) {
    if (wait < 3) { // Skip also initial data
      wait++;
    } else {
      data_min = min(data_min, mpuData);
      data_max = max(data_max, mpuData);
      _PP("MPU\t: ");
      _PP(mpuData);
      _PP(" samples/sec, ");
      _PP(data_min);
      _PP(" min, ");
      _PP(data_max);
      _PP(" max, ");
      _PP(mpuFIFOReset);
      _PP(" FIFO resets, last roll is ");
      _PP(roll * 180/M_PI);
      _PP(", ");
      _PP(roll_min * 180/M_PI);
      _PP(" min, ");
      _PP(roll_max * 180/M_PI);
      _PL(" max");
      _PP("Radio\t: ");
      _PP(packets_out_multicast);
      _PP(" out, ");
      _PP(packets_out_failed);
      _PP(" failed, ");
      _PP(radioPackets);
      _PL(" packets/second");
    }
    radioPackets = 0;
    mpuData = 0;
    
    // Allow other threads to run for 1 sec.
    chThdSleepMilliseconds(1000);
  }
}

//------------------------------------------------------------------------------
static THD_WORKING_AREA(waThread4, 1024);
static THD_FUNCTION(Thread4, arg) {
  (void)arg;
  while (true) {
    packet_t packet;
    packetBuildMulticast(&packet, (char *)"Hello!", 0);
    packetSendMulticast(&packet);
    chThdSleepMilliseconds(1); // Some 50 packets/second
    //chThdSleepMilliseconds(20); // 26/second
    //chThdSleepMilliseconds(100); // 8 to 9 
    radioPackets++;
  }
}
//------------------------------------------------------------------------------
// Continue setup() after chBegin().
void chSetup() {
  if (CH_CFG_TIME_QUANTUM != 1) {
    _PL("You must set CH_CFG_TIME_QUANTUM to 1 in");
#if defined(__arm__)
    _PP("src/arm/chconfig_arm.h");
#elif defined(__AVR__)
    _PP(F("src/avr/chconfig_avr.h")); 
#endif 
    while (true) {}
  }

  // LED
  chThdCreateStatic(waThread1, sizeof(waThread1),
    NORMALPRIO, Thread1, NULL);

  // MPU
  chThdCreateStatic(waThread2, sizeof(waThread2),
    NORMALPRIO, Thread2, NULL);

  // Summary
  chThdCreateStatic(waThread3, sizeof(waThread3),
    NORMALPRIO, Thread3, NULL);

  // Radio
  chThdCreateStatic(waThread4, sizeof(waThread4),
    NORMALPRIO, Thread4, NULL);
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
  for (int i = 0; i < 20; i++) { // Initial values are out of range
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
    //radio.setDataRate( RF24_250KBPS ); // Some 34 packets/s
    radio.setDataRate( RF24_1MBPS ); // Some 50 packets/s...
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
  chThdSleepMilliseconds(10000);
  
  // Print unused stack space in bytes.
  _PP(F("Stack\t: "));
  _PP(chUnusedThreadStack(waThread1, sizeof(waThread1)));
  _PP(" ");
  _PP(chUnusedThreadStack(waThread2, sizeof(waThread2)));
  _PP(" ");
  _PP(chUnusedThreadStack(waThread2, sizeof(waThread3)));
  _PP(" ");
  _PP(chUnusedThreadStack(waThread2, sizeof(waThread4)));
  _PP(" ");
  _PP(chUnusedMainStack());
#ifdef __arm__
  // ARM has separate stack for ISR interrupts. 
  _PP(" ");
  _PP(chUnusedHandlerStack());
#endif  // __arm__
  _PL();
}
