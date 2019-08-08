/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.

 */

#define __DEBUG_ // Via Serial
#ifdef __DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);

#else
#define _PP(a)
#define _PL(a)

#endif // __DEBUG_

struct payload_t {
  uint16_t      crc16;
  char          netid[5];
  unsigned long stamp;
  unsigned long counter;
  char          data[64];
};

struct packet_t {
  unsigned char type;
  payload_t     payload;
};

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#include <time.h>
#include <TimeLib.h>
// Had to comment out following in TimeLib.h...
//#define DAYS_PER_WEEK ((time_t)(7UL))

RF24 radio(18,19);

RF24Network network(radio);

const uint16_t this_node = 01;
const uint16_t master_node = 00;

const unsigned long interval = 1000;

unsigned long data_interval = 1000;
unsigned long data_last_sec;
unsigned long data_last_sent;

unsigned long last_stamp;

unsigned long packets_out_sent;
unsigned long packets_out_resent;
unsigned long packets_out_failed;
unsigned long packets_in_time_incomming;
unsigned long packets_in_time_ourtime;
         long packets_in_time_maxdrift;
unsigned long packets_in_time_drifts;
unsigned long packets_in_received;
unsigned long packets_in_bad_crc;
unsigned long packets_in_bad_netid;

#define PACKET_TIME   5   // Multicast, for dicovery and keep alive
#define PACKET_REBOOT 6   // Multicast, to restart all satelites
#define PACKET_ALIVE  65  // For keep alive and satelites stats
#define PACKET_SYNC   66  // For time/RTC initial synchronization
#define PACKET_QUIT   66  // Reboot request ack

bool got_packet_time = false;
bool got_packet_reboot = false;

bool time_was_set = false;

#include <FastCRC.h>
FastCRC16 CRC16;

#define NETID "Hj9G"

time_t getTeensy3Time(void) {
  _PL("SysTime update via RTC");
  return Teensy3Clock.get();
}

void reboot() {
  cli();
  delay(100);
  SCB_AIRCR = 0x05FA0004; // See Teensy SCB_AIRCR_SYSRESETREQ_MASK
  while(1);
}

void setup(void)
{
  Serial.begin(115200);
  int timeout = 200;
  while (!Serial && timeout--)
    delay(10);
 
  setSyncProvider(getTeensy3Time);
  setSyncInterval(300); // Default is 5min. set the number of seconds between re-sync
      Serial.print("now() is ");
      Serial.println(now());

  SPI.setSCK(14);
  SPI.begin();
  radio.begin();
  if (!radio.isChipConnected())
    Serial.println("Radio module dead ?");

  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_MIN);
  radio.setRetries (5, 10);
  network.begin(90, this_node);
}

uint32_t rtc_ms();

unsigned long syncOnRTC(unsigned long last_sec) {
  unsigned long now_sec;
  while ((now_sec = now()) <= data_last_sec) // TODO lock here, add a yield()
    network.update();
  return now_sec;
}


uint16_t payloadCRC (payload_t payload) {
  char  buffer[200];
  sprintf(buffer, "%2s%lu%lu%s", payload.netid, payload.stamp, payload.counter, payload.data);
  return CRC16.ccitt((uint8_t *)buffer, strlen(buffer));
}

void packetBuild(packet_t *packet, char *data, unsigned char type = 0) {
  strcpy(packet->payload.netid, NETID);
  packet->payload.stamp = now();
  packet->payload.counter = ++packets_out_sent;
  strcpy(packet->payload.data, data);
  packet->payload.crc16 = payloadCRC(packet->payload);
  packet->type = type;
}

bool packetSend(packet_t *packet, int &retry_attempts, unsigned long retry_delay, unsigned long &time_next_attempt) {
  RF24NetworkHeader header(master_node, packet->type);
  bool ok = network.write(header, &packet->payload, sizeof(packet->payload));
  if (ok) {
    Serial.print("ok");
    retry_attempts = 0;
  } else {
    Serial.print("failed");
    if (--retry_attempts) { // Retry later
      packets_out_resent++;
      time_next_attempt += retry_delay;
    } else {
      packets_out_failed++;
    }
  }

  return ok;
}

bool packetSend(packet_t *packet) {
  RF24NetworkHeader header(master_node, packet->type);
  bool ok = network.write(header, &packet->payload, sizeof(packet->payload));
  if (ok) {
    Serial.print("ok");
  } else {
    Serial.print("failed");
    packets_out_failed++;
  }

  return ok;
}

void loop() {
  
  network.update();

  while ( network.available() ) {
    
    RF24NetworkHeader header;
    payload_t payload;

    Serial.print(now());
    Serial.print(" ");

    // network.peek(header);
    network.read(header, &payload, sizeof(payload));
    if (payload.crc16 != payloadCRC(payload)) { // Drop any "noise"
      Serial.print("Bad CRC... received ");
      Serial.print(payload.crc16, HEX);
      Serial.print(" expected ");
      Serial.println(payloadCRC(payload), HEX);
      packets_in_bad_crc++;
    } else if (strcmp(payload.netid, NETID)) { // Drop any "noise"
      _PP("Bad NetID... we are ");
      _PP(NETID);
      _PP(" but we got ");
      _PP(payload.netid);
      _PL(" so packet dropped.");
      packets_in_bad_netid++;
    } else {
      packets_in_received++;
      
      switch (header.type) {
      case PACKET_TIME:
        packets_in_time_incomming = payload.stamp;
        packets_in_time_ourtime = now();
        
        long long diff = (long long)packets_in_time_incomming - (long long)packets_in_time_ourtime;
        if (!time_was_set) { // TODO improve initial time synchronization, dont rely on a single first packet
          Teensy3Clock.set(packets_in_time_incomming);
          setTime(packets_in_time_incomming);
        } else if (abs(diff) > 0) { 
          packets_in_time_maxdrift = max(packets_in_time_maxdrift, abs(diff));
        }
        Serial.print("Got PACKET_TIME ");
        Serial.print(payload.stamp);
        Serial.print(" ");
        Serial.print(header.to_node);
        if (packets_in_time_maxdrift) {
          Serial.print(" max diff ");
          Serial.print(packets_in_time_maxdrift);
        }
        if (!time_was_set && abs(diff) > 0) {
          packets_in_time_drifts++;
          Serial.print(" updated");
        }
        Serial.println("");
        
        got_packet_time = true;
        time_was_set = true;
        data_last_sent = millis() - data_interval;
        break;
     
      case PACKET_SYNC:
        // TODO improve initial time synchronization using dedicated unicast packets
        break;

      case PACKET_REBOOT:
        Serial.print("Got PACKET_REBOOT (from node was");
        Serial.print(header.from_node);
        Serial.println(")");
        got_packet_reboot = true;
        break;
      
      default:
        Serial.print("From ");
        Serial.print(header.from_node);
        Serial.print(" to ");
        Serial.print(header.to_node);
        Serial.print(" type ");
        Serial.print(header.to_node);
        if (header.to_node == 64)
          Serial.print(" - Multicast - ");
        Serial.print(" ");
        Serial.print(payload.netid);
        Serial.print(" ");
        Serial.print(payload.stamp);
        Serial.print(" ");
        Serial.print(payload.counter);
        Serial.print(" ");
        Serial.print(payload.data);
        Serial.print(" (data length ");
        Serial.print(strlen(payload.data));
        Serial.println(")");
        break;
      }
    }
  }

    // TODO cleanup
  unsigned long         DataSendNow = millis();
  static packet_t       DataPacket;
  static unsigned long  DataSendOn = millis() - 100;
  static int            DataSendAttempts = 0;

  // First get in sync with RTC
  if ( time_was_set && DataSendNow - data_last_sent >= (data_interval - 20)  ) {
    // Get in sync whith RTC
    data_last_sec = syncOnRTC(data_last_sec);
    data_last_sent =  millis();

    // Build our packet
    char buffer[100];
    sprintf(buffer, "Dummy packet from %02d - 012345678901234567890123456789", this_node);
    packetBuild(&DataPacket, buffer);

    // Get out of synch/collisions with at least PACKET_TIME
    DataSendOn = data_last_sent + 200;
    DataSendAttempts = 5;
  }
  
  if ( DataSendAttempts && DataSendNow > DataSendOn) { // There is the millis() rollover issue (but every 49 days only)
    Serial.print(now());
    Serial.print(" ");
    Serial.print("Sending dummy packet");
    if (DataSendAttempts <= 4) {
    Serial.print(" attempt ");
      Serial.print(5 - DataSendAttempts);
    }
    Serial.print("... ");
    unsigned long stamp = micros();
    // Send the packet
    packetSend(&DataPacket, DataSendAttempts, /* delay befor retry */  100, DataSendOn);    
    stamp = micros() - stamp;
    Serial.print(" (");
    Serial.print((float)stamp / 1000);
    Serial.println("ms)");
  }

  unsigned long         TimeSendNow = millis();
  static packet_t       TimePacket;
  static unsigned long  TimeSendOn = millis() - 100;
  static int            TimeSendAttempts = 0;

  static unsigned long time_last_sec;
  static unsigned long time_last_sent;
  
  if (got_packet_time) {
    got_packet_time = false;
    // Get in sync whith RTC
    time_last_sec = syncOnRTC(time_last_sec);
    time_last_sent =  millis();
    
    // Build our packet
    char buffer[100];
    sprintf(buffer, "%lu;%lu;%lu;%lu;%lu;%lu;%lu;", packets_out_sent, packets_out_resent, packets_out_failed,
      packets_in_bad_crc, packets_in_bad_netid, packets_in_time_maxdrift, packets_in_time_incomming, packets_in_time_ourtime);
    packetBuild(&TimePacket, buffer, PACKET_TIME);

    // Get out of synch/collisions with at least PACKET_TIME and data packets
    TimeSendOn = time_last_sent + 500;
    TimeSendAttempts = 5;
  }

  if ( TimeSendAttempts && TimeSendNow > TimeSendOn) { // There is the millis() rollover issue (but every 49 days only)
    Serial.print(now());
    Serial.print(" ");
    Serial.print("Sending PACKET_ALIVE");
    if (TimeSendAttempts < 4) {
    Serial.print(" attempt ");
      Serial.print(5 - TimeSendAttempts);
    }
    Serial.print("... ");
    unsigned long stamp = micros();
    // Send the packet
    packetSend(&TimePacket, TimeSendAttempts, /* delay befor retry */ 200, TimeSendOn);    
    stamp = micros() - stamp;
    Serial.print(" (");
    Serial.print((float)stamp / 1000);
    Serial.println("ms)");
  }

  if (got_packet_reboot) {
    got_packet_reboot = false;
    
//    delay(10 * this_node); // Attempt to avoid collisions...
    Serial.print(now());
    Serial.print(" ");
    Serial.print("Sending PACKET_QUIT... ");
    packet_t RebootPacket;
    packetBuild(&RebootPacket, "Leaving for reboot...", PACKET_QUIT);
    packetSend(&RebootPacket);    
    
    // Anyway, reboot, we had that packet
    // TODO Any cleanup before? Let the master deal with such lost packets?
    Serial.print("Rebooting... ");
    reboot();
  }
      
  unsigned long nowStamp = millis();
  if (  nowStamp - last_stamp >= interval * 20 ) {
    last_stamp = nowStamp;

    Serial.print("Packets out : ");
    Serial.print(packets_out_sent);
    Serial.print(" sent, ");
    Serial.print(packets_out_resent);
    Serial.print(" resent, ");
    Serial.print(packets_out_failed);
    Serial.println(" failed");
    Serial.print("Packets in : ");
    Serial.print(packets_in_bad_crc);
    Serial.print(" bad CRC, ");
    Serial.print(packets_in_bad_netid);
    Serial.print(" bad NetID, ");
    Serial.print(packets_in_received);
    Serial.println(" accepted");
    Serial.print("Time synch status : "); 
    Serial.print(packets_in_time_maxdrift);
    Serial.print(" sec max diff, current diff is ");
    Serial.print(packets_in_time_incomming);
    Serial.print("(in) / ");
    Serial.print(packets_in_time_ourtime);
    Serial.println("(RTC)");
  }
  
}

