/**
 * SPDX-FileCopyrightText: 2024 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Uncomment the following line if you want to use custom callbacks
#define MCP23017_USE_CALLBACKS
#include "MCP23017.h"

// Pins definition
const uint8_t BUTTON_PIN = 0;
const uint8_t INT_PIN    = 1;

// Instantiation
using namespace MCP23017;
MCP23017_IO<> io(I2CAddress::A0_0_A1_0_A2_0, Wire);

// Interrupt service routine
// ESP32 example (IRAM_ATTR is required)
volatile bool interrupt_detected = false;
void IRAM_ATTR ISR() { interrupt_detected = true; }

#ifdef MCP23017_USE_CALLBACKS
// Custom callback
void callback(const uint8_t pin, const bool value) {
  Serial.print("Callback called - pin: ");
  Serial.print(pin);
  Serial.print(", value: ");
  Serial.println(value);
}
#endif

void setup() {
  Serial.begin(115200);

  // Initialize I²C
  Wire.begin();

  // Check if the MCP23017 is connected
  if (!io.isConnected()) {
    Serial.println("MCP23017 not found");

    while (true)
      delay(1000);
  }

  // Initialize MCP23017 with OpenDrain interrupt pins
  // We assume that the INT_PIN has a pull-up resistor
  Status status = io.init(IntPinType::OpenDrain);

  if (status != Status::Ok) {
    Serial.print("Failed to initialize MCP23017. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("MCP23017 initialized");

  // Set button pin mode
  status = io.pinMode(BUTTON_PIN, Mode::Input);

  if (status != Status::Ok) {
    Serial.println("Failed to set button pin mode. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Button pin mode set");

  /**
   * Enable pull-up resistor on button pin.
   * We assume that the button is connected between the pin and GND.
   *
   * NOTE: add a capacitor between the button and GND to avoid bouncing, otherwise the interrupt
   * will be triggered multiple times.
   */
  status = io.pinPullUp(BUTTON_PIN, PullUp::Enable);

  if (status != Status::Ok) {
    Serial.println("Failed to enable pull-up resistor on button pin. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Pull-up resistor enabled on button pin");

  // Set interrupt mode on button pin
  status = io.pinInterruptMode(BUTTON_PIN, IntMode::Change);

  if (status != Status::Ok) {
    Serial.println("Failed to set interrupt mode on button pin. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Interrupt mode set on button pin");

  // Enable interrupt on button pin
  status = io.pinInterruptEnable(BUTTON_PIN, IntEnable::On);

  if (status != Status::Ok) {
    Serial.println("Failed to enable interrupt on button pin. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Interrupt enabled on button pin");

#ifdef MCP23017_USE_CALLBACKS
  // Set custom callback on button pin
  status = io.setCallback(BUTTON_PIN, callback, IntMode::Change);

  if (status != Status::Ok) {
    Serial.println("Failed to set callback on button pin. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Callback set on button pin");
#endif

  // Attach interrupt to INT_PIN
  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, ISR, FALLING);
}

void loop() {
  if (!interrupt_detected) return;

  interrupt_detected = false;

  int8_t detected_pin;
  uint16_t intcap_value;

  // If custom callbacks are enabled, calling this function will trigger them
  Status status = io.interruptedBy(detected_pin, intcap_value);

  if (status != Status::Ok) {
    Serial.print("Failed to get interrupt info. Error: ");
    Serial.println(static_cast<uint8_t>(status));
    return;
  }

  Serial.print("Interrupt on pin ");
  Serial.print(detected_pin);
  Serial.print(", INTCAP value: 0x");
  Serial.println(intcap_value, HEX);
}