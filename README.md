# L9026 Driver Library

A comprehensive Arduino/PlatformIO library for the **STMicroelectronics L9026** highly configurable multi-channel driver designed for automotive applications.

## Features

- **8 Output Channels**
  - 2 fixed high-side channels (CH0, CH1)
  - 6 configurable high-side/low-side channels (CH2-CH7)
- **PWM Control**
  - LED PWM generator
  - General-purpose PWM generator
  - Input pin (IN0/IN1) mapping to channels
- **Comprehensive Diagnostics**
  - Off-mode: Open load detection, short circuit detection
  - On-mode: Open load detection, overcurrent/overtemperature detection
- **Daisy-Chain Support**
  - Control multiple L9026 devices via SPI
  - Parallel diagnostic execution across devices
- **Operating Modes**
  - Sleep, Idle, Limp-Home, Active modes
- **Additional Features**
  - Bulb inrush mode (BIM)
  - Hardware reset and disable pin control
  - Per-channel configuration and diagnostics

## Hardware Requirements

- L9026 IC (VFQFPN32 or other package variants)
- SPI-capable microcontroller (Arduino, ESP32, STM32, etc.)
- Pull-up resistors on SPI lines (recommended)

## Installation

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps =
    https://github.com/outlandnish/st-l9026-driver.git
```

Or place this library in your project's `lib/` folder.

### Arduino IDE

1. Download this library from https://github.com/outlandnish/st-l9026-driver
2. Place it in your Arduino `libraries` folder
3. Restart the Arduino IDE

## Quick Start

```cpp
#include <l9026.h>

// Define device configuration
#define CS_PIN 10
#define IDLE_PIN 1

L9026Device devices[] = {
  L9026Device(0, -1, -1, IDLE_PIN)  // device_id, in0, in1, idle
};

L9026 driver(&SPI, CS_PIN, devices, 1);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  if (!driver.begin()) {
    Serial.println("Failed to initialize L9026");
    while(1);
  }

  // Configure channel 2 as low-side output
  driver.configureOutputSide(0, OutputChannel::CHANNEL_2, OutputType::LOW_SIDE);

  // Enable active mode
  driver.setActiveMode(0, true);

  // Write configuration
  driver.updateConfiguration();

  // Enable output on channel 2
  driver.setOutput(0, OutputChannel::CHANNEL_2, true);
  driver.updateOutputs();
}

void loop() {
  // Your code here
}
```

## API Reference

### Initialization

```cpp
L9026(SPIClass *spi, uint32_t cs, L9026Device devices[], uint8_t device_count);
bool begin();
void end();
bool getChipID();
```

### Configuration

```cpp
// Configure channel as high-side or low-side (channels 2-7 only)
void configureOutputSide(uint8_t device_id, OutputChannel channel, OutputType type);

// Enable/disable active mode
void setActiveMode(uint8_t device_id, bool enable);

// Map input pins to output channels
void mapInput(uint8_t device_id, L9026PWMInput input, OutputChannel channel);

// Configure PWM
void configureLEDPWM(uint8_t device_id, L9026LEDPWMConfiguration *config);
void configureGeneralPWM(uint8_t device_id, L9026GeneralPWMConfiguration *config);

// Configure bulb inrush mode
void configureBulbInrushMode(uint8_t device_id, OutputChannel channel, bool enable);

// Write configuration to device
void updateConfiguration();
```

### Output Control

```cpp
// Enable/disable output channel
void setOutput(uint8_t device_id, OutputChannel channel, bool enable);

// Set PWM duty cycle
void setGeneralPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle);
void setLEDPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle);

// Update outputs
void updateOutputs();
```

### Diagnostics

```cpp
// Off-mode diagnostics
void readAllDeviceOffModeDiagnostics();
void enableOffModeDiagnostics(uint8_t device_id, bool enable, uint8_t channel_mask = 0xFF, bool commit = true);
void readDeviceOffModeOpenLoadDiagnostics(uint8_t device_id);
void readDeviceOffModeShortedLoadDiagnostics(uint8_t device_id);

// On-mode diagnostics
void readAllDeviceOnModeDiagnostics();
void readDeviceOnModeOvercurrentOvertemperatureDiagnostics(uint8_t device_id, bool commit = true);
void enableDeviceOnModeOpenLoadDiagnostics(uint8_t device_id, OutputChannel channel, bool commit = true);
void readDeviceOnModeOpenLoadDiagnosticsResults(uint8_t device_id, bool commit = true);

// Clear errors
void clearAllDeviceOvercurrentOvertemperatureErrors();

// Get diagnostics results
L9026ChannelDiagnostics getChannelDiagnostics(uint8_t device_id, OutputChannel channel);
```

### Status

```cpp
void readAllDeviceStatus();
L9026DeviceStatus getDeviceStatus(uint8_t device_id);
```

### Hardware Control

```cpp
// Reset pin control (NRES - active low)
void assertReset(uint8_t device_id);
void deassertReset(uint8_t device_id);

// Disable pin control (DIS - active high)
void assertDisable(uint8_t device_id);
void deassertDisable(uint8_t device_id);

// Software reset
void softReset();

// Power modes
void toggleSleepMode(bool enable);
```

## Datasheet

[L9026 Datasheet](https://www.st.com/resource/en/datasheet/l9026.pdf)

## License

MIT License

## Contributing

Contributions are welcome! Please open an issue or submit a pull request on the [GitHub repository](https://github.com/outlandnish/st-l9026-driver).
