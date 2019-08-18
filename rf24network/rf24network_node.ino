/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.

 */

/* 
 * Using RF24Network transmit, multicast and header.type == 'T'
 *
 * NODE (master_node = 00; this_node = 01;)
 * Receives multicast + multicast time frames
 * 
 */

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

RF24 radio(18,19);

RF24Network network(radio);

const uint16_t this_node = 01;
const uint16_t master_node = 00;

const unsigned long interval = 2000;

unsigned long last_sent;
unsigned long packets_sent;


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

void loop() {
  
  network.update();

  while ( network.available() ) {
    
    RF24NetworkHeader header;
    payload_t payload;

    network.peek(header);
    if(header.type == 'T'){
      uint32_t time;
      network.read(header,&time,sizeof(time));
      Serial.print("Got time: ");
      Serial.print(time);
      Serial.print(" (header.type == 'T') (to_node was ");
      Serial.print(header.to_node);
      Serial.println(")");
    } else {
      network.read(header,&payload,sizeof(payload));
      if (header.to_node == 64)
        Serial.print("Multicast packet #");
      else
        Serial.print("Received packet #");
      Serial.print(payload.counter);
      Serial.print(" at ");
      Serial.print(payload.ms);
      Serial.print(" is ");
      Serial.print(payload.data);
      Serial.print(" (length ");
      Serial.print(strlen(payload.data));
      Serial.print(") (to_node was ");
      Serial.print(header.to_node);
      Serial.println(")");
    }
  }
  
  unsigned long now = millis();
  if ( now - last_sent >= interval  )
  {
    last_sent = now;
    
    Serial.print("Sending...");
    payload_t payload = { millis(), packets_sent++, "01 data 0123456789012345678901234567890123456789" };
    RF24NetworkHeader header(master_node);
    unsigned long stamp = micros();
    bool ok = network.write(header,&payload,sizeof(payload));
    stamp = micros() - stamp;
    if (ok)
      Serial.print("ok. (");
    else
      Serial.print("failed. (");
    Serial.print((float)stamp / 1000);
    Serial.println("ms)");
  }
}
