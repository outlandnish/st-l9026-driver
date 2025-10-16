/**
 * BulbInrushMode.ino
 *
 * Demonstrates bulb inrush mode (BIM) with the L9026 driver.
 * This example shows how to:
 * - Configure bulb inrush mode for a channel
 * - Control high current inductive loads (bulbs, relays)
 * - Prevent false overcurrent detection during inrush
 */

#include <SPI.h>
#include <l9026.h>

// Pin definitions
#define OUTPUT_CS 10
#define OUTPUT_IDLE 1

// Create single L9026 device
L9026Device outputDevices[] = {
  L9026Device(0, -1, -1, OUTPUT_IDLE)
};

L9026 outputs(&SPI, OUTPUT_CS, outputDevices, 1);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9026 Bulb Inrush Mode Example");
  Serial.println("===============================");

  // Initialize L9026
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026");
    while (1);
  }

  outputs.softReset();

  // Configure channel 2 as high-side (for bulb)
  Serial.println("Configuring channel 2 as high-side output");
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_2, OutputType::HIGH_SIDE);

  // Enable bulb inrush mode on channel 2
  // BIM provides higher current limit during turn-on to handle inrush
  Serial.println("Enabling bulb inrush mode on channel 2");
  outputs.configureBulbInrushMode(0, OutputChannel::CHANNEL_2, true);

  // Enable active mode
  outputs.setActiveMode(0, true);

  // Write configuration
  outputs.updateConfiguration();

  Serial.println("\nBulb inrush mode configured!");
  Serial.println("Channel 2 can now handle high inrush currents");
  Serial.println("Typical use: incandescent bulbs, relays, solenoids");
}

void loop() {
  // Turn on bulb (high inrush current handled by BIM)
  Serial.println("\nTurning ON bulb (BIM active)");
  outputs.setOutput(0, OutputChannel::CHANNEL_2, true);
  outputs.updateOutputs();
  delay(3000);

  // Turn off bulb
  Serial.println("Turning OFF bulb");
  outputs.setOutput(0, OutputChannel::CHANNEL_2, false);
  outputs.updateOutputs();
  delay(2000);

  // Check for overcurrent/overtemperature
  outputs.readDeviceOnModeOvercurrentOvertemperatureDiagnostics(0);
  auto diag = outputs.getChannelDiagnostics(0, OutputChannel::CHANNEL_2);

  if (diag.overcurrent_overtemperature_detected) {
    Serial.println("WARNING: Overcurrent or overtemperature detected!");
  } else {
    Serial.println("Status: Normal operation");
  }
}
