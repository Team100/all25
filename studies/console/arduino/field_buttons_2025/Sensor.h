#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"

class Sensor {
public:
  void initialize() {
  }
  /**
   * Reads the key array state and writes the values into reportTx.
   */
  void sense(ReportTx& reportTx) {
    // col 0
    // col 1
    // col 2
    // col 3
    // col 4
    // col 5
    // col 6
    // row 0
    // row 1
    // row 2
    // row 3
    // row 4
    reportTx.b1 = false;
    reportTx.b2 = false;
    reportTx.b3 = false;
    reportTx.b4 = false;
    reportTx.b5 = false;
    reportTx.b6 = false;
    reportTx.b7 = false;
    reportTx.b8 = false;
    reportTx.b9 = false;
    reportTx.b10 = false;
    reportTx.b11 = false;
    reportTx.b12 = false;
    reportTx.b13 = false;
    reportTx.b14 = false;
    reportTx.b15 = false;
    reportTx.b16 = false;
    reportTx.b17 = false;
    reportTx.b18 = false;
    reportTx.b19 = false;
    reportTx.b20 = false;
    reportTx.b21 = false;
    reportTx.b22 = false;
    reportTx.b23 = false;
    reportTx.b24 = false;
    reportTx.b25 = false;
    reportTx.b26 = false;
    reportTx.b27 = false;
    reportTx.b28 = false;
    reportTx.b29 = false;
    reportTx.b30 = false;
    reportTx.b31 = false;
    reportTx.b32 = false;
  }

};
#endif  // SENSOR_H
