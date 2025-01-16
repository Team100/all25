#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"

// TODO: correct pin numbers here.
constexpr uint8_t COL0 = 0;
constexpr uint8_t COL1 = 0;
constexpr uint8_t COL2 = 0;
constexpr uint8_t COL3 = 0;
constexpr uint8_t COL4 = 0;
constexpr uint8_t COL5 = 0;
constexpr uint8_t COL6 = 0;
constexpr uint8_t ROW0 = 0;
constexpr uint8_t ROW1 = 0;
constexpr uint8_t ROW2 = 0;
constexpr uint8_t ROW3 = 0;
constexpr uint8_t ROW4 = 0;

/**
 * We use a 7x5 grid with three holes, resulting in 32 buttons.
 */
class Sensor {
public:
  /**
   * Set pin modes.
   */
  void initialize() {
    // All columns are pull-up digital inputs.
    pinMode(COL0, INPUT_PULLUP);
    pinMode(COL1, INPUT_PULLUP);
    pinMode(COL2, INPUT_PULLUP);
    pinMode(COL3, INPUT_PULLUP);
    pinMode(COL4, INPUT_PULLUP);
    pinMode(COL5, INPUT_PULLUP);
    // All rows start as high-Z digital inputs.
    rowHigh(ROW0);
    rowHigh(ROW1);
    rowHigh(ROW2);
    rowHigh(ROW3);
    rowHigh(ROW4);
  }

  /**
   * Pull the row low
   */
  void rowLow(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(5);
  }

  /**
   * Set the row back to high-Z
   */
  void rowHigh(uint8_t pin) {
    pinMode(pin, INPUT);
    delayMicroseconds(5);
  }

  /**
   * Return true if col is pulled low, i.e. key is pressed.
   */
  boolean colRead(uint8_t pin) {
    return digitalRead(pin) == LOW;
  }

  /**
   * Reads the key array state and writes the values into reportTx.
   */
  void sense(ReportTx& reportTx) {
    rowLow(ROW0);
    reportTx.b1 = colRead(COL0);
    reportTx.b2 = colRead(COL1);
    // note missing col2
    reportTx.b3 = colRead(COL3);
    reportTx.b4 = colRead(COL4);
    reportTx.b5 = colRead(COL5);
    reportTx.b6 = colRead(COL6);
    rowHigh(ROW0);
    rowLow(ROW1);
    reportTx.b7 = colRead(COL0);
    reportTx.b8 = colRead(COL1);
    reportTx.b9 = colRead(COL2);
    reportTx.b10 = colRead(COL3);
    reportTx.b11 = colRead(COL4);
    reportTx.b12 = colRead(COL5);
    reportTx.b13 = colRead(COL6);
    rowHigh(ROW1);
    rowLow(ROW2);
    // note missing col0
    reportTx.b14 = colRead(COL1);
    reportTx.b15 = colRead(COL2);
    reportTx.b16 = colRead(COL3);
    reportTx.b17 = colRead(COL4);
    reportTx.b18 = colRead(COL5);
    reportTx.b19 = colRead(COL6);
    rowHigh(ROW2);
    rowLow(ROW3);
    // note missing col0
    reportTx.b20 = colRead(COL1);
    reportTx.b21 = colRead(COL2);
    reportTx.b22 = colRead(COL3);
    reportTx.b23 = colRead(COL4);
    reportTx.b24 = colRead(COL5);
    reportTx.b25 = colRead(COL6);
    rowHigh(ROW3);
    rowLow(ROW4);
    reportTx.b26 = colRead(COL0);
    reportTx.b27 = colRead(COL1);
    reportTx.b28 = colRead(COL2);
    reportTx.b29 = colRead(COL3);
    reportTx.b30 = colRead(COL4);
    reportTx.b31 = colRead(COL5);
    reportTx.b32 = colRead(COL6);
    rowHigh(ROW4);
  }

};
#endif  // SENSOR_H
