/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "MCP23017.h"

// Pins definition
const uint8_t BUTTON_PIN = 0;
const uint8_t LED_PIN    = 8;

// Instantiation
using namespace MCP23017;
MCP23017_IO<> io(I2CAddress::A0_0_A1_0_A2_0, Wire);

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

  // Initialize MCP23017
  Status status = io.init();

  if (status != Status::Ok) {
    Serial.print("Failed to initialize MCP23017. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("MCP23017 initialized");

  // Set pin modes
  status = io.pinMode(BUTTON_PIN, Mode::Input, LED_PIN, Mode::Output);

  if (status != Status::Ok) {
    Serial.println("Failed to set pin modes. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Pin modes set");

  // Enable pull-up resistor on button pin
  status = io.pinPullUp(BUTTON_PIN, PullUp::Enable);

  if (status != Status::Ok) {
    Serial.println("Failed to enable pull-up resistor on button pin. Error: ");
    Serial.println(static_cast<uint8_t>(status));

    while (true)
      delay(1000);
  }

  Serial.println("Pull-up resistor enabled on button pin");
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t current_time     = millis();

  // Poll every second
  if ((current_time - last_time) < 1000) return;

  last_time = current_time;

  // Read and print button state
  bool button_state = false;
  Status status     = io.pinDigitalRead(BUTTON_PIN, button_state);

  if (status != Status::Ok) {
    Serial.print("Failed to read button state. Error: ");
    Serial.println(static_cast<uint8_t>(status));
    return;
  }

  Serial.print("Button state: ");
  Serial.println(button_state);

  // Toggle and print new LED state
  static Output led_state = Output::Low;
  led_state               = (led_state == Output::Low ? Output::High : Output::Low);

  status = io.pinDigitalWrite(LED_PIN, led_state);

  if (status != Status::Ok) {
    Serial.print("Failed to write LED state. Error: ");
    Serial.println(static_cast<uint8_t>(status));
    return;
  }

  Serial.print("LED state: ");
  Serial.println(static_cast<uint8_t>(led_state));
}
