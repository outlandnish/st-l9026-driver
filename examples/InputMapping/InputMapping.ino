/**
 * InputMapping.ino
 *
 * Demonstrates input pin mapping with the L9026 driver.
 * This example shows how to:
 * - Configure IN0 and IN1 pins
 * - Map input pins to output channels
 * - Control outputs via hardware pins instead of SPI
 */

#include <SPI.h>
#include <l9026.h>

// Pin definitions
#define OUTPUT_CS 10
#define OUTPUT_IDLE 1
#define OUTPUT_IN0 2
#define OUTPUT_IN1 3

// Create L9026 device with IN0 and IN1 configured
L9026Device outputDevices[] = {
  L9026Device(0, OUTPUT_IN0, OUTPUT_IN1, OUTPUT_IDLE)
};

L9026 outputs(&SPI, OUTPUT_CS, outputDevices, 1);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9026 Input Mapping Example");

  // Initialize L9026
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026");
    while (1);
  }

  outputs.softReset();

  // Configure channels 2 and 3 as low-side outputs
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_2, OutputType::LOW_SIDE);
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_3, OutputType::LOW_SIDE);

  // Map IN0 to channel 2, IN1 to channel 3
  Serial.println("Mapping IN0 to channel 2");
  outputs.mapInput(0, L9026PWMInput::IN0, OutputChannel::CHANNEL_2);

  Serial.println("Mapping IN1 to channel 3");
  outputs.mapInput(0, L9026PWMInput::IN1, OutputChannel::CHANNEL_3);

  // Enable active mode
  outputs.setActiveMode(0, true);

  // Write configuration
  outputs.updateConfiguration();

  Serial.println("Input mapping configured!");
  Serial.println("Channels 2 and 3 now controlled by IN0 and IN1 pins");
}

void loop() {
  // Toggle IN0 (controls channel 2)
  Serial.println("IN0 HIGH -> Channel 2 ON");
  digitalWrite(OUTPUT_IN0, HIGH);
  delay(1000);

  Serial.println("IN0 LOW -> Channel 2 OFF");
  digitalWrite(OUTPUT_IN0, LOW);
  delay(1000);

  // Toggle IN1 (controls channel 3)
  Serial.println("IN1 HIGH -> Channel 3 ON");
  digitalWrite(OUTPUT_IN1, HIGH);
  delay(1000);

  Serial.println("IN1 LOW -> Channel 3 OFF");
  digitalWrite(OUTPUT_IN1, LOW);
  delay(1000);
}
