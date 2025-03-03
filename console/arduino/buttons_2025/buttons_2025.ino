#include "Data.h"
#include "Transceiver.h"
#include "Sensor.h"

ReportRx reportRx_;
ReportTx reportTx_;
Sensor sensor_;
Transceiver transceiver_(Transceiver::SubConsole::BUTTONS_2025, reportRx_);

void setup() {
  sensor_.initialize();
}

void loop() {
  sensor_.sense(reportTx_);
  transceiver_.send(reportTx_);
  // The sensor has an issue triggered by state changes during I2C transmission:
  // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/issues/96
  // To minimize the incidence of that bug, minimize the sample rate.
  delay(100);
}
