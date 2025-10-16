/**
 * Diagnostics.ino
 *
 * Demonstrates diagnostic features of the L9026 driver.
 * This example shows how to:
 * - Run off-mode diagnostics (open load, short circuit)
 * - Run on-mode diagnostics (open load, overcurrent/overtemperature)
 * - Read and display diagnostic results
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

void printBinary(uint8_t value) {
  for (int8_t b = 7; b >= 0; b--) {
    Serial.print((value >> b) & 1);
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9026 Diagnostics Example");
  Serial.println("=========================");

  // Initialize L9026
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026");
    while (1);
  }

  outputs.softReset();

  // Configure channels 0-1 as high-side (fixed)
  // Configure channels 2-7 as low-side
  for (auto j = OutputChannel::CHANNEL_2; j <= MAX_OUTPUT_CHANNEL; j++) {
    outputs.configureOutputSide(0, j, OutputType::LOW_SIDE);
  }

  // Enable active mode
  outputs.setActiveMode(0, true);
  outputs.updateConfiguration();

  // Read device status
  Serial.println("\nDevice Status:");
  outputs.readAllDeviceStatus();
  auto status = outputs.getDeviceStatus(0);
  Serial.printf("  Mode: %d (0=Sleep, 1=LimpHome, 2=Idle, 3=Active)\n", status.operating_mode);
  Serial.printf("  IDLE pin: %d\n", status.idle_status);
  Serial.printf("  Power-on reset: %d\n", status.power_on_reset_condition_detected);

  // Run off-mode diagnostics
  Serial.println("\n--- Off-Mode Diagnostics ---");
  outputs.readAllDeviceOffModeDiagnostics();

  uint8_t open_load_mask = 0;
  uint8_t shorted_mask = 0;

  for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
    auto diag = outputs.getChannelDiagnostics(0, (OutputChannel)j);
    if (diag.off_state_open_load_detected) open_load_mask |= (1 << j);
    if (diag.shorted_load_detected) shorted_mask |= (1 << j);
  }

  Serial.print("Open Load: ");
  printBinary(open_load_mask);
  Serial.println(" (1=detected)");

  Serial.print("Shorted:   ");
  printBinary(shorted_mask);
  Serial.println(" (1=detected)");

  // Enable channel 0 and 1 for on-mode diagnostics
  Serial.println("\n--- On-Mode Diagnostics ---");
  Serial.println("Enabling channels 0 and 1...");
  outputs.setOutput(0, OutputChannel::CHANNEL_0, true);
  outputs.setOutput(0, OutputChannel::CHANNEL_1, true);
  outputs.updateOutputs();

  delay(100);

  // Run on-mode diagnostics
  outputs.readAllDeviceOnModeDiagnostics();

  uint8_t oc_ot_mask = 0;
  uint8_t on_open_load_mask = 0;

  for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
    auto diag = outputs.getChannelDiagnostics(0, (OutputChannel)j);
    if (diag.overcurrent_overtemperature_detected) oc_ot_mask |= (1 << j);
    if (diag.on_state_open_load_detected) on_open_load_mask |= (1 << j);
  }

  Serial.print("OC/OT:     ");
  printBinary(oc_ot_mask);
  Serial.println(" (1=detected)");

  Serial.print("Open Load: ");
  printBinary(on_open_load_mask);
  Serial.println(" (1=detected)");

  // Disable outputs
  outputs.setOutput(0, OutputChannel::CHANNEL_0, false);
  outputs.setOutput(0, OutputChannel::CHANNEL_1, false);
  outputs.updateOutputs();

  Serial.println("\nDiagnostics complete!");
}

void loop() {
  // Diagnostics run once in setup
  delay(1000);
}
