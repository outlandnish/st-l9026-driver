/**
 * DaisyChain.ino
 *
 * Demonstrates daisy-chain operation with multiple L9026 devices.
 * This example shows how to:
 * - Configure multiple L9026 devices on the same SPI bus
 * - Control outputs on different devices
 * - Run diagnostics across all devices in parallel
 */

#include <SPI.h>
#include <l9026.h>

// Pin definitions
#define OUTPUT_CS 10
#define DEVICE_0_IDLE 1
#define DEVICE_1_IDLE 2

// Create two L9026 devices in daisy-chain
L9026Device outputDevices[] = {
  L9026Device(0, -1, -1, DEVICE_0_IDLE),  // First device in chain
  L9026Device(1, -1, -1, DEVICE_1_IDLE)   // Second device in chain
};

L9026 outputs(&SPI, OUTPUT_CS, outputDevices, 2);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9026 Daisy-Chain Example");
  Serial.println("=========================");
  Serial.printf("Devices in chain: %d\n\n", outputs.getDeviceCount());

  // Initialize L9026 devices
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026 devices");
    while (1);
  }

  outputs.softReset();

  // Configure all devices
  for (uint8_t i = 0; i < outputs.getDeviceCount(); i++) {
    Serial.printf("Configuring device %d\n", i);

    // Configure channels 2-7 as low-side
    for (auto j = OutputChannel::CHANNEL_2; j <= MAX_OUTPUT_CHANNEL; j++) {
      outputs.configureOutputSide(i, j, OutputType::LOW_SIDE);
    }

    // Enable active mode
    outputs.setActiveMode(i, true);
  }

  // Write configuration to all devices
  outputs.updateConfiguration();

  // Read status of all devices
  Serial.println("\nDevice Status:");
  outputs.readAllDeviceStatus();
  for (uint8_t i = 0; i < outputs.getDeviceCount(); i++) {
    auto status = outputs.getDeviceStatus(i);
    Serial.printf("  Device %d: Mode=%d IDLE=%d\n",
                  i, status.operating_mode, status.idle_status);
  }

  Serial.println("\nSetup complete!");
}

void loop() {
  // Control channel 2 on device 0
  Serial.println("Device 0, Channel 2 ON");
  outputs.setOutput(0, OutputChannel::CHANNEL_2, true);
  outputs.updateOutputs();
  delay(1000);

  // Control channel 2 on device 1
  Serial.println("Device 1, Channel 2 ON");
  outputs.setOutput(1, OutputChannel::CHANNEL_2, true);
  outputs.updateOutputs();
  delay(1000);

  // Turn off both
  Serial.println("All OFF");
  outputs.setOutput(0, OutputChannel::CHANNEL_2, false);
  outputs.setOutput(1, OutputChannel::CHANNEL_2, false);
  outputs.updateOutputs();
  delay(1000);

  // Run diagnostics on all devices (parallel)
  Serial.println("\nRunning diagnostics on all devices...");
  outputs.readAllDeviceOffModeDiagnostics();

  for (uint8_t i = 0; i < outputs.getDeviceCount(); i++) {
    uint8_t open_load = 0;
    uint8_t shorted = 0;

    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
      auto diag = outputs.getChannelDiagnostics(i, (OutputChannel)j);
      if (diag.off_state_open_load_detected) open_load |= (1 << j);
      if (diag.shorted_load_detected) shorted |= (1 << j);
    }

    Serial.printf("Device %d: OpenLoad=0x%02X Shorted=0x%02X\n",
                  i, open_load, shorted);
  }

  delay(2000);
}
