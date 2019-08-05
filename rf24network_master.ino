/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.

 */

/* 
 * Using RF24Network transmit, multicast and header.type == 'T'
 *
 * MASTER (this_node = 00; other_node = 01;)
 * Sends out multicast + multicast time frames
 * 
 */

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>


RF24 radio(18,19); 

RF24Network network(radio);
const uint16_t this_node = 00;
const uint16_t other_node = 01;

const unsigned long interval = 5000;

unsigned long last_sent;
unsigned long packets_sent;

unsigned long last_sent_multicast;
unsigned long packets_sent_multicast;

unsigned long last_sent_time;
unsigned long packets_sent_time;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
  char data[64];
};


void setup(void)
{
  Serial.begin(115200);
 
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);
}

void loop(void){
  
  network.update();

  while ( network.available() ) {
    
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header,&payload,sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.print(payload.ms);
    Serial.print(" is ");
    Serial.print(payload.data);
    Serial.print(" (");
    Serial.print(strlen(payload.data));
    Serial.println(")");
  }

  unsigned long now = millis();
  if ( now - last_sent >= interval  )
  {
    last_sent = now;

    Serial.print("Sending...");
    payload_t payload = { millis(), packets_sent++, "00 data" };
    RF24NetworkHeader header(other_node);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");
  }

  unsigned long nowMulticast = millis();
  if ( nowMulticast - last_sent_multicast >= interval / 2  )
  {
    last_sent_multicast = now;

    Serial.print("Sending multicast...");
    payload_t payload = { millis(), packets_sent_multicast++, "00 data to multicast level 01" };
    RF24NetworkHeader header();
    bool ok = network.multicast(header,&payload,sizeof(payload), 01);
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");
  }

  unsigned long nowTime = millis(); 
  if ( nowTime - last_sent_time >= interval / 3  )
  {
    last_sent_time = now;

    Serial.print("Sending time...");
    payload_t payload = { millis(), packets_sent_multicast++, "00 data to multicast level 01" };
    uint32_t time = millis();
    uint16_t to = 64;
    RF24NetworkHeader header(to, 'T');
    network.write(header,&time,sizeof(time));
    bool ok = network.multicast(header,&payload,sizeof(payload), 01);
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");
  }
}
