#include "Data.h"
#include "Transceiver.h"
#include "Sensor.h"

ReportRx reportRx_;
ReportTx reportTx_;
Sensor sensor_;
Transceiver transceiver_(Transceiver::SubConsole::FIELD_2025, reportRx_);

void setup() {
  sensor_.initialize();
}

void loop() {
  sensor_.sense(reportTx_);
  transceiver_.send(reportTx_);
}
