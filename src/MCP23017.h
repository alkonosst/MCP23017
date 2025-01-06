/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#define MCP23017_PAIR_ERROR_TEXT "This method requires pairs of pin and value arguments"

namespace MCP23017 {

/// @brief I2C address combinations according to A0, A1 and A2 pins
enum class I2CAddress : uint8_t {
  A0_0_A1_0_A2_0 = 0x20,
  A0_1_A1_0_A2_0,
  A0_0_A1_1_A2_0,
  A0_1_A1_1_A2_0,
  A0_0_A1_0_A2_1,
  A0_1_A1_0_A2_1,
  A0_0_A1_1_A2_1,
  A0_1_A1_1_A2_1,
};

/// @brief Interrupt pins physical configuration
enum class IntPinType : uint8_t { PushPull, OpenDrain };

/// @brief Interrupt pins polarity (only for PushPull configuration)
enum class IntPinPol : uint8_t { ActiveLow, ActiveHigh };

/// @brief Port selection
enum class Port : uint8_t { PortA, PortB };

/// @brief Pin modes
enum class Mode : uint8_t { Output, Input };

/// @brief Pull-up resistor configuration
enum class PullUp : uint8_t { Disable, Enable };

/// @brief Input polarity
enum class InputPol : uint8_t { NoInverted, Inverted };

/// @brief Output value
enum class Output : uint8_t { Low, High };

/// @brief Interrupt enable
enum class IntEnable : uint8_t { Off, On };

/// @brief Interrupt mode
enum class IntMode : uint8_t { Change, Rising, Falling };

/// @brief Status codes
enum class Status : uint8_t {
  Ok,
  DataTooLong,
  ReceivedNackOnTxAddress,
  ReceivedNackOnTxData,
  OtherError,
  Timeout,
  FailedToRequestBytes,
  PinOutOfRange,
  InterruptNotDetected,
#ifdef MCP23017_USE_CALLBACKS
  InvalidCallback
#endif
};

template <typename PinEnum = uint8_t>
class MCP23017_IO {
#ifdef MCP23017_USE_CALLBACKS
  using Callback = void (*)(const PinEnum pin, const bool value);
#endif

  public:
  MCP23017_IO(I2CAddress address = I2CAddress::A0_0_A1_0_A2_0, TwoWire& i2c = Wire)
      : _address(address)
      , _i2c(i2c) {}

  /**
   * @brief Test if the device is connected to the I2C bus.
   * @return true Connected
   * @return false Not connected, check wiring or power
   */
  bool isConnected() {
    _i2c.beginTransmission(static_cast<uint8_t>(_address));
    return _i2c.endTransmission() == 0;
  }

  /**
   * @brief Initialize MCP23017. This method must be called before using any other method. This
   * will: disable Slew Rate control on SDA pin, enable sequential operation, enable INT pins mirror
   * and configure INT pins as desired.
   * @param int_pin_type Interrupt pins physical configuration
   * @param int_pin_pol Interrupt pins polarity (only for PushPull configuration)
   * @return Status Status code
   */
  Status init(IntPinType int_pin_type = IntPinType::OpenDrain,
              IntPinPol int_pin_pol   = IntPinPol::ActiveLow) {
    Status status;

    IOCON int_pin_type_flag =
      (int_pin_type == IntPinType::OpenDrain ? IOCON::odr : IOCON::Unimplemented);

    IOCON int_pin_pol_flag =
      (int_pin_pol == IntPinPol::ActiveHigh ? IOCON::intpol : IOCON::Unimplemented);

    IOCON port_config =
      IOCON::disslw | IOCON::seqop | IOCON::mirror | int_pin_type_flag | int_pin_pol_flag;

    // Config
    status = writeRegisters(
      Register::ioconA, static_cast<uint8_t>(port_config), static_cast<uint8_t>(port_config));
    if (status != Status::Ok) return status;

    // All inputs
    status = writeRegisters(Register::iodirA, 0xFF, 0xFF);
    if (status != Status::Ok) return status;

    // Pull-ups disabled
    status = writeRegisters(Register::gppuA, 0x00, 0x00);
    if (status != Status::Ok) return status;

    // Non-inverted inputs
    status = writeRegisters(Register::ipolA, 0x00, 0x00);
    if (status != Status::Ok) return status;

    // Outputs low
    status = writeRegisters(Register::olatA, 0x00, 0x00);
    if (status != Status::Ok) return status;

    // Interrupts disabled
    status = writeRegisters(Register::gpintenA, 0x00, 0x00);
    if (status != Status::Ok) return status;

    // Interrupt change/rising
    status = writeRegisters(Register::defvalA, 0x00, 0x00);
    if (status != Status::Ok) return status;

    // Interrupt mode change
    status = writeRegisters(Register::intconA, 0x00, 0x00);
    if (status != Status::Ok) return status;

    // Clear interrupts
    uint16_t intcap_value;
    return readRegisters(Register::intcapA, intcap_value);
  }

  /**
   * @brief Set the direction of a pin.
   * @tparam Ts (Optional) Pair of pin and mode values
   * @param pin Selected pin
   * @param mode Pin mode
   * @param vs (Optional) Pair of pin and mode values
   * @return Status Status code
   */
  template <typename... Ts>
  Status pinMode(const PinEnum pin, const Mode mode, const Ts&... vs) {
    static_assert(sizeof...(vs) % 2 == 0, MCP23017_PAIR_ERROR_TEXT);

    _temp_reg = Register::iodirA;

    Status status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    return writePin(pin, mode, vs...);
  }

  /**
   * @brief Set the direction of a complete port.
   * @param port Selected port
   * @param mode Port mode
   * @return Status Status code
   */
  Status portMode(const Port port, const Mode mode) {
    _temp_reg = (port == Port::PortA ? Register::iodirA : Register::iodirB);
    return writeRegister(_temp_reg, mode == Mode::Input ? 0xFF : 0x00);
  }

  /**
   * @brief Set the direction of the two ports.
   * @param mode Port mode
   * @return Status Status code
   */
  Status portsMode(const Mode mode) {
    uint8_t value = (mode == Mode::Input ? 0xFF : 0x00);
    return writeRegisters(Register::iodirA, value, value);
  }

  /**
   * @brief Enable or disable pull-up resistor on a pin.
   * @tparam Ts (Optional) Pair of pin and pull-up values
   * @param pin Selected pin
   * @param pullup Pull-up resistor configuration
   * @param vs (Optional) Pair of pin and pull-up values
   * @return Status Status code
   */
  template <typename... Ts>
  Status pinPullUp(const PinEnum pin, const PullUp pullup, const Ts&... vs) {
    static_assert(sizeof...(vs) % 2 == 0, MCP23017_PAIR_ERROR_TEXT);

    _temp_reg = Register::gppuA;

    Status status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    return writePin(pin, pullup, vs...);
  }

  /**
   * @brief Enable or disable pull-up resistor on a complete port.
   * @param port Selected port
   * @param pullup Pull-up resistor configuration
   * @return Status Status code
   */
  Status portPullUp(const Port port, const PullUp pullup) {
    _temp_reg = (port == Port::PortA ? Register::gppuA : Register::gppuB);
    return writeRegister(_temp_reg, pullup == PullUp::Enable ? 0xFF : 0x00);
  }

  /**
   * @brief Enable or disable pull-up resistor on the two ports.
   * @param pullup Pull-up resistor configuration
   * @return Status Status code
   */
  Status portsPullUp(const PullUp pullup) {
    uint8_t value = (pullup == PullUp::Enable ? 0xFF : 0x00);
    return writeRegisters(Register::gppuA, value, value);
  }

  /**
   * @brief Invert the input polarity of a pin.
   * @tparam Ts (Optional) Pair of pin and input polarity values
   * @param pin Selected pin
   * @param polarity Input polarity
   * @param vs (Optional) Pair of pin and input polarity values
   * @return Status Status code
   */
  template <typename... Ts>
  Status pinInputPolarity(const PinEnum pin, const InputPol polarity, const Ts&... vs) {
    static_assert(sizeof...(vs) % 2 == 0, MCP23017_PAIR_ERROR_TEXT);

    _temp_reg = Register::ipolA;

    Status status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    return writePin(pin, polarity, vs...);
  }

  /**
   * @brief Invert the input polarity of a complete port.
   * @param port Selected port
   * @param polarity Input polarity
   * @return Status Status code
   */
  Status portInputPolarity(const Port port, const InputPol polarity) {
    _temp_reg = (port == Port::PortA ? Register::ipolA : Register::ipolB);
    return writeRegister(_temp_reg, polarity == InputPol::Inverted ? 0xFF : 0x00);
  }

  /**
   * @brief Invert the input polarity of the two ports.
   * @param polarity Input polarity
   * @return Status Status code
   */
  Status portsInputPolarity(const InputPol polarity) {
    uint8_t value = (polarity == InputPol::Inverted ? 0xFF : 0x00);
    return writeRegisters(Register::ipolA, value, value);
  }

  /**
   * @brief Set the interrupt mode on a pin.
   * @tparam Ts (Optional) Pair of pin and interrupt mode values
   * @param pin Selected pin
   * @param mode Interrupt mode
   * @param vs (Optional) Pair of pin and interrupt mode values
   * @return Status Status code
   */
  template <typename... Ts>
  Status pinInterruptMode(const PinEnum pin, const IntMode mode, const Ts&... vs) {
    static_assert(sizeof...(vs) % 2 == 0, MCP23017_PAIR_ERROR_TEXT);

    _temp_reg = Register::defvalA;

    Status status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    status = writePin(pin, mode, vs...);
    if (status != Status::Ok) return status;

    _temp_reg = Register::intconA;

    status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    return writePin(pin, mode, vs...);
  }

  /**
   * @brief Set the interrupt mode on a complete port.
   * @param port Selected port
   * @param mode Interrupt mode
   * @return Status Status code
   */
  Status portInterruptMode(const Port port, const IntMode mode) {
    _temp_reg = (port == Port::PortA ? Register::defvalA : Register::defvalB);

    Status status = writeRegister(_temp_reg, mode == IntMode::Falling ? 0xFF : 0x00);
    if (status != Status::Ok) return status;

    _temp_reg = (port == Port::PortA ? Register::intconA : Register::intconB);
    return writeRegister(_temp_reg, mode == IntMode::Change ? 0x00 : 0xFF);
  }

  /**
   * @brief Set the interrupt mode on the two ports.
   * @param mode Interrupt mode
   * @return Status Status code
   */
  Status portsInterruptMode(const IntMode mode) {
    uint8_t value = (mode == IntMode::Falling ? 0xFF : 0x00);

    Status status = writeRegisters(Register::defvalA, value, value);
    if (status != Status::Ok) return status;

    value = (mode == IntMode::Change ? 0x00 : 0xFF);
    return writeRegisters(Register::intconA, value, value);
  }

  /**
   * @brief Enable or disable interrupt on a pin.
   * @tparam Ts (Optional) Pair of pin and interrupt enable values
   * @param pin Selected pin
   * @param enable Interrupt enable
   * @param vs (Optional) Pair of pin and interrupt enable values
   * @return Status Status code
   */
  template <typename... Ts>
  Status pinInterruptEnable(const PinEnum pin, const IntEnable enable, const Ts&... vs) {
    static_assert(sizeof...(vs) % 2 == 0, MCP23017_PAIR_ERROR_TEXT);

    _temp_reg = Register::gpintenA;

    Status status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    return writePin(pin, enable, vs...);
  }

  /**
   * @brief Enable or disable interrupt on a complete port.
   * @param port Selected port
   * @param enable Interrupt enable
   * @return Status Status code
   */
  Status portInterruptEnable(const Port port, const IntEnable enable) {
    _temp_reg = (port == Port::PortA ? Register::gpintenA : Register::gpintenB);
    return writeRegister(_temp_reg, enable == IntEnable::On ? 0xFF : 0x00);
  }

  /**
   * @brief Enable or disable interrupt on the two ports.
   * @param enable Interrupt enable
   * @return Status Status code
   */
  Status portsInterruptEnable(const IntEnable enable) {
    uint8_t value = (enable == IntEnable::On ? 0xFF : 0x00);
    return writeRegisters(Register::gpintenA, value, value);
  }

  /**
   * @brief Write a digital value to a pin.
   * @tparam Ts (Optional) Pair of pin and output values
   * @param pin Selected pin
   * @param output Output value
   * @param vs (Optional) Pair of pin and output values
   * @return Status Status code
   */
  template <typename... Ts>
  Status pinDigitalWrite(const PinEnum pin, const Output output, const Ts&... vs) {
    static_assert(sizeof...(vs) % 2 == 0, MCP23017_PAIR_ERROR_TEXT);

    _temp_reg = Register::olatA;

    Status status = readRegisters(_temp_reg, _temp_reg_last_value[0], _temp_reg_last_value[1]);
    if (status != Status::Ok) return status;

    _temp_reg_new_value[0] = _temp_reg_last_value[0];
    _temp_reg_new_value[1] = _temp_reg_last_value[1];

    return writePin(pin, output, vs...);
  }

  /**
   * @brief Write a digital value to a complete port.
   * @param port Selected port
   * @param output Port value
   * @return Status Status code
   */
  Status portDigitalWrite(const Port port, const Output output) {
    _temp_reg = (port == Port::PortA ? Register::gpioA : Register::gpioB);
    return writeRegister(_temp_reg, output == Output::High ? 0xFF : 0x00);
  }

  /**
   * @brief Write a digital value to the two ports.
   * @param output Port value
   * @return Status Status code
   */
  Status portsDigitalWrite(const Output output) {
    uint8_t value = (output == Output::High ? 0xFF : 0x00);
    return writeRegisters(Register::gpioA, value, value);
  }

  /**
   * @brief Read the current value of a pin.
   * @param pin Selected pin
   * @param pin_value Pin value
   * @return Status Status code
   */
  Status pinDigitalRead(const PinEnum pin, bool& pin_value) {
    if (static_cast<uint8_t>(pin) < 0 || static_cast<uint8_t>(pin) > 15)
      return Status::PinOutOfRange;

    uint16_t pin_mask;

    Status status = readRegisters(Register::gpioA, pin_mask);
    if (status != Status::Ok) return status;

    pin_value = bitRead(pin_mask, static_cast<uint8_t>(pin));
    return Status::Ok;
  }

  /**
   * @brief Read the current value of a port.
   * @param port Selected port
   * @param port_value Port value mask
   * @return Status Status code
   */
  Status portDigitalRead(Port port, uint8_t& port_value) {
    Register reg = (port == Port::PortA ? Register::gpioA : Register::gpioB);
    return readRegister(reg, port_value);
  }

  /**
   * @brief Read the current value of the two ports.
   * @param port_values Port values mask
   * @return Status Status code
   */
  Status portsDigitalRead(uint16_t& port_values) {
    uint8_t port_a, port_b;

    Status status = readRegisters(Register::gpioA, port_a, port_b);
    if (status != Status::Ok) return status;

    port_values = port_a | (port_b << 8);
    return Status::Ok;
  }

  /**
   * @brief Get the pin that triggered the interrupt and the value of the INTCAP register
   * (interrupt captured). Must be called after an interrupt is detected.
   * @param intcap_value INTCAP register value at the time of the interrupt
   * @param detected_pin Detected pin
   * @return Status Status code
   */
  Status interruptedBy(int8_t& detected_pin, uint16_t& intcap_value) {
    uint16_t pin_mask;

    Status status = readRegisters(Register::intfA, pin_mask);
    if (status != Status::Ok) return status;

    detected_pin = -1;

    for (uint8_t i = 0; i < 16; i++) {
      if (bitRead(pin_mask, i)) {
        detected_pin = i;
        break;
      }
    }

    status = readRegisters(Register::intcapA, intcap_value);
    if (status != Status::Ok) return status;

    if (detected_pin == -1) return Status::InterruptNotDetected;

#ifdef MCP23017_USE_CALLBACKS
    CallbackData& cb = _callbacks[detected_pin];

    if (cb.callback == nullptr) return Status::Ok;

    bool value  = bitRead(intcap_value, detected_pin);
    PinEnum pin = static_cast<PinEnum>(detected_pin);

    switch (cb.edge) {
      case IntMode::Change:
      {
        cb.callback(pin, value);
      } break;

      case IntMode::Rising:
      {
        if (value) cb.callback(pin, value);
      } break;

      case IntMode::Falling:
      {
        if (!value) cb.callback(pin, value);
      } break;
    }
#endif

    return Status::Ok;
  }

#ifdef MCP23017_USE_CALLBACKS
  /**
   * @brief Set a callback function to be called when an interrupt is detected on a pin.
   * @param pin Selected pin
   * @param callback Callback function
   * @param edge Interrupt edge
   * @return Status Status code
   */
  Status setCallback(const PinEnum pin, Callback callback, IntMode edge) {
    if (static_cast<uint8_t>(pin) < 0 || static_cast<uint8_t>(pin) > 15)
      return Status::PinOutOfRange;

    if (callback == nullptr) return Status::InvalidCallback;

    _callbacks[static_cast<uint8_t>(pin)] = {callback, edge};
    return Status::Ok;
  }

  /**
   * @brief Remove the callback function from a pin.
   * @param pin Selected pin
   * @return Status Status code
   */
  Status removeCallback(const PinEnum pin) {
    if (static_cast<uint8_t>(pin) < 0 || static_cast<uint8_t>(pin) > 15)
      return Status::PinOutOfRange;

    _callbacks[static_cast<uint8_t>(pin)] = {nullptr, IntMode::Change};
    return Status::Ok;
  }
#endif

  private:
  enum class Register : uint8_t {
    iodirA   = 0x00,
    iodirB   = 0x01,
    ipolA    = 0x02,
    ipolB    = 0x03,
    gpintenA = 0x04,
    gpintenB = 0x05,
    defvalA  = 0x06,
    defvalB  = 0x07,
    intconA  = 0x08,
    intconB  = 0x09,
    ioconA   = 0x0A,
    ioconB   = 0x0B,
    gppuA    = 0x0C,
    gppuB    = 0x0D,
    intfA    = 0x0E,
    intfB    = 0x0F,
    intcapA  = 0x10,
    intcapB  = 0x11,
    gpioA    = 0x12,
    gpioB    = 0x13,
    olatA    = 0x14,
    olatB    = 0x15,
  };

  enum class IOCON : uint8_t {
    Unimplemented = 0,
    intpol        = 1 << 1,
    odr           = 1 << 2,
    haen          = 1 << 3,
    disslw        = 1 << 4,
    seqop         = 1 << 5,
    mirror        = 1 << 6,
    bank          = 1 << 7
  };

  friend inline IOCON operator|(IOCON a, IOCON b) {
    return static_cast<IOCON>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
  }

  template <typename Value, typename... Ts>
  Status writePin(PinEnum pin, Value value, const Ts&... vs) {
    if (static_cast<uint8_t>(pin) < 0 || static_cast<uint8_t>(pin) > 15)
      return Status::PinOutOfRange;

    switch (_temp_reg) {
      case Register::defvalA:
      {
        switch (value) {
          case static_cast<Value>(IntMode::Change):
          case static_cast<Value>(IntMode::Rising):
          {
            value = static_cast<Value>(0);
          } break;

          case static_cast<Value>(IntMode::Falling):
          {
            value = static_cast<Value>(1);
          } break;
        }
      } break;

      case Register::intconA:
      {
        switch (value) {
          case static_cast<Value>(IntMode::Change):
          {
            value = static_cast<Value>(0);
          } break;

          case static_cast<Value>(IntMode::Rising):
          case static_cast<Value>(IntMode::Falling):
          {
            value = static_cast<Value>(1);
          } break;
        }
      } break;
    }

    uint8_t reg_index = 0;

    if (static_cast<uint8_t>(pin) > 7) {
      pin       = static_cast<PinEnum>(static_cast<uint8_t>(pin) - 8);
      reg_index = 1;
    }

    uint8_t last_value = bitRead(_temp_reg_last_value[reg_index], static_cast<uint8_t>(pin));

    if (last_value != static_cast<uint8_t>(value)) {
      bitToggle(_temp_reg_new_value[reg_index], static_cast<uint8_t>(pin));
    }

    return writePin(vs...);
  }

  Status writePin() {
    bool write_a = (_temp_reg_new_value[0] != _temp_reg_last_value[0]);
    bool write_b = (_temp_reg_new_value[1] != _temp_reg_last_value[1]);

    if (write_a && write_b) {
      return writeRegisters(_temp_reg, _temp_reg_new_value[0], _temp_reg_new_value[1]);
    } else if (write_a) {
      return writeRegister(_temp_reg, Port::PortA, _temp_reg_new_value[0]);
    } else if (write_b) {
      return writeRegister(_temp_reg, Port::PortB, _temp_reg_new_value[1]);
    }

    return Status::Ok;
  }

  Status writeRegister(Register reg, uint8_t value) {
    _i2c.beginTransmission(static_cast<uint8_t>(_address));
    _i2c.write(static_cast<uint8_t>(reg));
    _i2c.write(value);
    return static_cast<Status>(_i2c.endTransmission());
  }

  Status writeRegister(Register reg, Port port, uint8_t value) {
    uint8_t selected_reg = static_cast<uint8_t>(reg) + static_cast<uint8_t>(port);
    _i2c.beginTransmission(static_cast<uint8_t>(_address));
    _i2c.write(selected_reg);
    _i2c.write(value);
    return static_cast<Status>(_i2c.endTransmission());
  }

  Status writeRegisters(Register reg, uint8_t port_a_value, uint8_t port_b_value) {
    _i2c.beginTransmission(static_cast<uint8_t>(_address));
    _i2c.write(static_cast<uint8_t>(reg));
    _i2c.write(port_a_value);
    _i2c.write(port_b_value);
    return static_cast<Status>(_i2c.endTransmission());
  }

  Status readRegister(Register reg, uint8_t& reg_value) {
    _i2c.beginTransmission(static_cast<uint8_t>(_address));
    _i2c.write(static_cast<uint8_t>(reg));

    Status status = static_cast<Status>(_i2c.endTransmission());
    if (status != Status::Ok) return status;

    const uint8_t bytes = 1;

    if (_i2c.requestFrom(static_cast<uint8_t>(_address), bytes) != bytes)
      return Status::FailedToRequestBytes;

    reg_value = _i2c.read();
    return Status::Ok;
  }

  Status readRegisters(Register reg, uint8_t& port_a, uint8_t& port_b) {
    _i2c.beginTransmission(static_cast<uint8_t>(_address));
    _i2c.write(static_cast<uint8_t>(reg));

    Status status = static_cast<Status>(_i2c.endTransmission());
    if (status != Status::Ok) return status;

    const uint8_t bytes = 2;

    if (_i2c.requestFrom(static_cast<uint8_t>(_address), bytes) != bytes)
      return Status::FailedToRequestBytes;

    port_a = _i2c.read();
    port_b = _i2c.read();
    return Status::Ok;
  }

  Status readRegisters(Register reg, uint16_t& reg_values) {
    uint8_t port_a, port_b;

    Status status = readRegisters(reg, port_a, port_b);
    if (status != Status::Ok) return status;

    reg_values = port_a | (port_b << 8);
    return Status::Ok;
  }

  I2CAddress _address;
  TwoWire& _i2c;
  Register _temp_reg;
  uint8_t _temp_reg_last_value[2];
  uint8_t _temp_reg_new_value[2];

#ifdef MCP23017_USE_CALLBACKS
  struct CallbackData {
    Callback callback;
    IntMode edge;
  } _callbacks[16];
#endif
};

} // namespace MCP23017