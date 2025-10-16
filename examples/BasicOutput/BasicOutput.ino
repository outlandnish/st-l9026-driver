/**
 * BasicOutput.ino
 *
 * Demonstrates basic output control with the L9026 driver.
 * This example shows how to:
 * - Initialize the L9026
 * - Configure channels as high-side or low-side
 * - Enable/disable outputs
 * - Toggle outputs in a loop
 */

#include <SPI.h>
#include <l9026.h>

// Pin definitions
#define OUTPUT_CS 10
#define OUTPUT_IDLE 1

// Create single L9026 device
L9026Device outputDevices[] = {
  L9026Device(0, -1, -1, OUTPUT_IDLE)  // device_id, in0, in1, idle
};

L9026 outputs(&SPI, OUTPUT_CS, outputDevices, 1);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9026 Basic Output Example");

  // Initialize L9026
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026");
    while (1);
  }
  Serial.println("L9026 initialized successfully");

  // Soft reset
  outputs.softReset();

  // Configure channels 2-7 as low-side outputs
  Serial.println("Configuring channels as low-side outputs");
  for (auto j = OutputChannel::CHANNEL_2; j <= MAX_OUTPUT_CHANNEL; j++) {
    outputs.configureOutputSide(0, j, OutputType::LOW_SIDE);
  }

  // Enable active mode
  Serial.println("Enabling active mode");
  outputs.setActiveMode(0, true);

  // Write configuration to device
  outputs.updateConfiguration();

  Serial.println("Setup complete!");
}

void loop() {
  // Toggle channel 2
  Serial.println("Enabling channel 2");
  outputs.setOutput(0, OutputChannel::CHANNEL_2, true);
  outputs.updateOutputs();
  delay(1000);

  Serial.println("Disabling channel 2");
  outputs.setOutput(0, OutputChannel::CHANNEL_2, false);
  outputs.updateOutputs();
  delay(1000);
}
