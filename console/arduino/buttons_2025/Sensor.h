#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"
#include <Adafruit_MCP23X17.h>

/**
 * The sensor consists of two MCP23017 boards.
 */
class Sensor {
public:
  /**
   * Sets pin modes.
   */
  void initialize() {
    mcp0.begin_I2C(0x20);
    mcp1.begin_I2C(0x21);
    // These are the "A" pins.
    mcp0.pinMode(0, INPUT_PULLUP);
    mcp0.pinMode(1, INPUT_PULLUP);
    mcp0.pinMode(2, INPUT_PULLUP);
    mcp0.pinMode(3, INPUT_PULLUP);
    mcp0.pinMode(4, INPUT_PULLUP);
    mcp0.pinMode(5, INPUT_PULLUP);
    mcp0.pinMode(6, INPUT_PULLUP);
    mcp0.pinMode(7, INPUT_PULLUP);
    // These are the "B" pins.
    mcp0.pinMode(8, INPUT_PULLUP);
    mcp0.pinMode(9, INPUT_PULLUP);
    mcp0.pinMode(10, INPUT_PULLUP);
    mcp0.pinMode(11, INPUT_PULLUP);
    mcp0.pinMode(12, INPUT_PULLUP);
    mcp0.pinMode(13, INPUT_PULLUP);
    mcp0.pinMode(14, INPUT_PULLUP);
    mcp0.pinMode(15, INPUT_PULLUP);
    // These are the "A" pins.
    mcp1.pinMode(0, INPUT_PULLUP);
    mcp1.pinMode(1, INPUT_PULLUP);
    mcp1.pinMode(2, INPUT_PULLUP);
    mcp1.pinMode(3, INPUT_PULLUP);
    mcp1.pinMode(4, INPUT_PULLUP);
    mcp1.pinMode(5, INPUT_PULLUP);
    mcp1.pinMode(6, INPUT_PULLUP);
    mcp1.pinMode(7, INPUT_PULLUP);
    // These are the "B" pins.
    mcp1.pinMode(8, INPUT_PULLUP);
    mcp1.pinMode(9, INPUT_PULLUP);
    mcp1.pinMode(10, INPUT_PULLUP);
    mcp1.pinMode(11, INPUT_PULLUP);
    mcp1.pinMode(12, INPUT_PULLUP);
    mcp1.pinMode(13, INPUT_PULLUP);
    mcp1.pinMode(14, INPUT_PULLUP);
    mcp1.pinMode(15, INPUT_PULLUP);
  }

  /**
   * Reads the switch state and writes the values into reportTx.
   *
   * The GPIOs are pulled up so when the button is pressed, they go low.
   */
  void sense(ReportTx& reportTx) {
    uint16_t gpio0 = mcp0.readGPIOAB();
    uint16_t gpio1 = mcp1.readGPIOAB();
    reportTx.b1 = !(gpio0 & 0b0000000000000001);
    reportTx.b2 = !(gpio0 & 0b0000000000000010);
    reportTx.b3 = !(gpio0 & 0b0000000000000100);
    reportTx.b4 = !(gpio0 & 0b0000000000001000);
    reportTx.b5 = !(gpio0 & 0b0000000000010000);
    reportTx.b6 = !(gpio0 & 0b0000000000100000);
    reportTx.b7 = !(gpio0 & 0b0000000001000000);
    reportTx.b8 = !(gpio0 & 0b0000000010000000);
    reportTx.b9 = !(gpio0 & 0b0000000100000000);
    reportTx.b10 = !(gpio0 & 0b0000001000000000);
    reportTx.b11 = !(gpio0 & 0b0000010000000000);
    reportTx.b12 = !(gpio0 & 0b0000100000000000);
    reportTx.b13 = !(gpio0 & 0b0001000000000000);
    reportTx.b14 = !(gpio0 & 0b0010000000000000);
    reportTx.b15 = !(gpio0 & 0b0100000000000000);
    reportTx.b16 = !(gpio0 & 0b1000000000000000);
    reportTx.b17 = !(gpio1 & 0b0000000000000001);
    reportTx.b18 = !(gpio1 & 0b0000000000000010);
    reportTx.b19 = !(gpio1 & 0b0000000000000100);
    reportTx.b20 = !(gpio1 & 0b0000000000001000);
    reportTx.b21 = !(gpio1 & 0b0000000000010000);
    reportTx.b22 = !(gpio1 & 0b0000000000100000);
    reportTx.b23 = !(gpio1 & 0b0000000001000000);
    reportTx.b24 = !(gpio1 & 0b0000000010000000);
    reportTx.b25 = !(gpio1 & 0b0000000100000000);
    reportTx.b26 = !(gpio1 & 0b0000001000000000);
    reportTx.b27 = !(gpio1 & 0b0000010000000000);
    reportTx.b28 = !(gpio1 & 0b0000100000000000);
    reportTx.b29 = !(gpio1 & 0b0001000000000000);
    reportTx.b30 = !(gpio1 & 0b0010000000000000);
    reportTx.b31 = !(gpio1 & 0b0100000000000000);
    reportTx.b32 = !(gpio1 & 0b1000000000000000);
  }

private:
  Adafruit_MCP23X17 mcp0;
  Adafruit_MCP23X17 mcp1;
};
#endif  // SENSOR_H
