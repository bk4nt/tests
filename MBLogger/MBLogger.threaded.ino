/*
 Copyright (C) 2019 Bruno KANT <bkant@cloppy.net>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 3.0 as published by the Free Software Foundation.
 */

#include "mblogger.threaded.h"

bool bootOk = false;

void setup() {
  // Force power on (via MOSFET and battery management chips)
  pinMode(PIN_POWERDOWN, OUTPUT);
  digitalWrite(PIN_POWERDOWN, HIGH);

#ifdef __DEBUG_
  Serial.begin(115200);
  int timeout = 200;
  while (!Serial && timeout--)
    delay(10);
#endif //  __DEBUG_

  _PL("Main\tAlive ! Configuring SPI + Wire...");

  SPI.setSCK(PIN_SPI_SCK);
  SPI.begin();

  Wire.begin();
  Wire.setClock(400000L);

  _PL("Main\tNow checking the battery...");
  Scheduler.start(setupBattery, loopBattery);
  bootBattery.wait(); // Might fail (power goes down on low bat)
  _PL("Main\tIf we get here, battery is Ok");

  _PL("Main\tStarting watchdog task...");
  Scheduler.start(setupWatchdog, loopWatchdog);
  bootWatchdog.wait(); // Shall occur... might be ignored

  displayStateSet(BOOTING);
  _PL("Main\tStarting display and buttons tasks...");
  Scheduler.start(setupDisplay, loopDisplay, 2048);
  Scheduler.start(setupButtons, loopButtons);
  bootDisplay.wait(); // Shall occur
  bootButtons.wait(); // Shall occur
  delay(1000); // Minimal boot splash
  
  _PL("Main\tNow read any config file...")
  Scheduler.start(setupSDCard, loopSDCard, 2048);
  bootSDCard.wait();

  _PL("Main\tNow starting other tasks...")
  Scheduler.start(setupMPU, loopMPU, 2048);
  Scheduler.start(setupGPS, loopGPS, 2048);
  Scheduler.start(setupRadio, loopRadio, 2048);

  _PL("Main\tWaiting for other tasks setups...")
  
  bootMPU.wait();
  bootGPS.wait();
  bootRadio.wait();

  bootOk = true;
  _PL("Main\tTasks started")

  displayStateSet(RUNNING);
}

void loop() {
  yield();
}
