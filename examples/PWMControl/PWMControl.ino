/**
 * PWMControl.ino
 *
 * Demonstrates PWM control with the L9026 driver.
 * This example shows how to:
 * - Configure LED PWM on a channel
 * - Set PWM duty cycle
 * - Fade an LED using PWM
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

// LED PWM configuration for channel 4
L9026LEDPWMConfiguration ledPWMConfig = {
  .channel = OutputChannel::CHANNEL_4,
  .active = true,
  .frequency = PWMFrequency::PWM_FREQUENCY_490_2_HZ
};

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9026 PWM Control Example");

  // Initialize L9026
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026");
    while (1);
  }

  outputs.softReset();

  // Configure channel 4 as high-side output
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_4, OutputType::HIGH_SIDE);

  // Configure LED PWM on channel 4
  Serial.println("Configuring LED PWM on channel 4");
  outputs.configureLEDPWM(0, &ledPWMConfig);

  // Enable active mode
  outputs.setActiveMode(0, true);

  // Write configuration
  outputs.updateConfiguration();

  // Enable the PWM output
  outputs.setOutput(0, OutputChannel::CHANNEL_4, true);
  outputs.updateOutputs();

  Serial.println("PWM setup complete!");
}

void loop() {
  // Fade in
  Serial.println("Fading in...");
  for (uint8_t duty = 0; duty <= 255; duty += 5) {
    outputs.setLEDPWMDutyCycle(0, duty);
    outputs.updateOutputs();
    delay(50);
  }

  delay(500);

  // Fade out
  Serial.println("Fading out...");
  for (uint8_t duty = 255; duty > 0; duty -= 5) {
    outputs.setLEDPWMDutyCycle(0, duty);
    outputs.updateOutputs();
    delay(50);
  }

  delay(500);
}
