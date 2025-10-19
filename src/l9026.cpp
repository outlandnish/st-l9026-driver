#include "l9026.h"

bool L9026::begin() {
  // setup chip select first
  pinMode(cs, OUTPUT);

  digitalWrite(cs, HIGH);
  delay(1);

  for (uint8_t i = 0; i < device_count; i++) {
    // setup in0, in1 if specified
    if (devices[i].in0 != -1) {
      pinMode(devices[i].in0, OUTPUT);
      digitalWrite(devices[i].in0, LOW);
    }

    if (devices[i].in1 != -1) {
      pinMode(devices[i].in1, OUTPUT);
      digitalWrite(devices[i].in1, LOW);
    }

    // setup idle if specified and pull high
    if (devices[i].idle != -1) {
      pinMode(devices[i].idle, OUTPUT);
      digitalWrite(devices[i].idle, LOW);
    }

    // setup reset pin if specified (NRES - active low, deassert by default)
    if (devices[i].reset != -1) {
      pinMode(devices[i].reset, OUTPUT);
      digitalWrite(devices[i].reset, HIGH);
    }

    // setup disable pin if specified (DIS - active high, enable outputs by default)
    if (devices[i].disable != -1) {
      pinMode(devices[i].disable, OUTPUT);
      digitalWrite(devices[i].disable, LOW);
    }

    // update channel map with channel 0 and 1 configured as high side (these can't be changed)
    devices[i].output_side[OutputChannel::CHANNEL_0] = OutputType::HIGH_SIDE;
    devices[i].output_side[OutputChannel::CHANNEL_1] = OutputType::HIGH_SIDE;

    // setup channel diagnostics
    for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++)
      devices[i].diagnostics[j] = L9026ChannelDiagnostics();
  }

  // size of each transfer
  buffer_size = device_count * sizeof(uint16_t);
  tx_buffer = new uint8_t[buffer_size];
  rx_buffer = new uint8_t[buffer_size];

  // put device to sleep
  toggleSleepMode(true);

  // put device to idle
  toggleSleepMode(false);

  // get chip id for each device
  return getChipID();
}

void L9026::end() {
  delete[] tx_buffer;
  delete[] rx_buffer;
}

void L9026::packFrame(TransactionData &transaction, uint8_t *buffer) {
  uint16_t frame;
  frame = 0;

  // bit 15 is r/w flag
  frame |= (transaction.write << 15);

  // limit bits 14 - 10 to address
  frame |= (transaction.address & 0x1F) << 10;

  // mem copy data into bits 2 - 9
  if (transaction.write)
    frame |= (transaction.data & 0xFF) << 2;

  // bit 1 is odd parity (calculated from bits 2 - 15)
  frame |= (!__builtin_parity(frame)) << 1;

  // bit 0 is frame counter
  frame |= transaction.frame_counter;

  // swap frame bytes
  frame = __builtin_bswap16(frame);

  // copy frame into buffer
  memcpy(buffer, &frame, sizeof(uint16_t));
}

void L9026::unpackFrame(TransactionData &transaction, uint8_t *buffer) {
  // reset transaction data
  memset(&transaction, 0, sizeof(TransactionData));

  uint16_t frame;
  memcpy(&frame, buffer, sizeof(uint16_t));

  // Convert from big endian to little endian
  frame = __builtin_bswap16(frame);

  // extract address from bits 14 - 10
  transaction.address = (frame >> 10) & 0x1F;

  // extract data from bits 2 - 9
  transaction.data = (frame >> 2) & 0xFF;

  // extract error flag from bit 15
  transaction.last_frame_error = (frame >> 15) & 1;

  // extract frame counter from bit 0
  transaction.frame_counter = frame & 1;
}

void L9026::buildTransferBuffer(uint8_t *buffer) {
  assert(buffer != NULL);

  for (uint8_t i = 0; i < device_count; i++) {
    // reset memory
    memset(&devices[i].tx_buffer, 0, sizeof(uint16_t));
    packFrame(devices[i].transaction, devices[i].tx_buffer);
    memcpy(&buffer[DEVICE_BUFFER_OFFSET(device_count, i)], &devices[i].tx_buffer, sizeof(uint16_t));

    // debug print bytes of tx frame
    // Serial.printf("Device %d TX Frame: %02X %02X\n", i, devices[i].tx_buffer[0], devices[i].tx_buffer[1]);
  }

  // debug print buffer
  // Serial.printf("Packed buffer: %02X %02X %02X %02X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
}

void L9026::unpackTransferBuffer(uint8_t *buffer) {
  assert(buffer != NULL);

  // debug print buffer
  // Serial.printf("Unpack buffer: %02X %02X %02X %02X\n", buffer[0], buffer[1], buffer[2], buffer[3]);

  for (uint8_t i = 0; i < device_count; i++) {
    // reset memory
    memset(&devices[i].rx_buffer, 0, sizeof(uint16_t));

    // last device should be at MSB, first device at LSB
    memcpy(&devices[i].rx_buffer, &buffer[DEVICE_BUFFER_OFFSET(device_count, i)], sizeof(uint16_t));
    unpackFrame(devices[i].transaction, devices[i].rx_buffer);

    if (devices[i].transaction.last_frame_error) {
      Serial.printf("Previous frame error on device %d\n", i);
    }

    // debug print bytes of rx frame
    // Serial.printf("Device %d RX Frame: %02X %02X\n", i, devices[i].rx_buffer[0], devices[i].rx_buffer[1]);
  }
}

void L9026::transfer(bool get_response) {
  // clear buffers
  memset(tx_buffer, 0, buffer_size);
  memset(rx_buffer, 0, buffer_size);

  // take SPI interface (if callback provided)
  if (take_spi) {
    take_spi();
  }

  // start SPI transaction
  spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // we need to duplicate the transfer to read data
  if (get_response) {
    // build transfer buffer
    buildTransferBuffer(tx_buffer);

    digitalWrite(cs, LOW);
    delay(1);

    #if defined(ESP32)
    spi->transferBytes(tx_buffer, rx_buffer, buffer_size);
    #else
    spi->transfer(tx_buffer, rx_buffer, buffer_size);
    #endif
    digitalWrite(cs, HIGH);
    delay(1);

    unpackTransferBuffer(rx_buffer);

    // flip frame counters
    for (uint8_t i = 0; i < device_count; i++) {
      devices[i].transaction.frame_counter = devices[i].transaction.frame_counter ? 0 : 1;
    }
  }

  // build transfer buffer
  buildTransferBuffer(tx_buffer);

  digitalWrite(cs, LOW);
  delay(1);

  // Serial.printf("Frame counters: %d %d\n", devices[0].transaction.frame_counter, devices[1].transaction.frame_counter);
  #if defined(ESP32)
  spi->transferBytes(tx_buffer, rx_buffer, buffer_size);
  #else
  spi->transfer(tx_buffer, rx_buffer, buffer_size);
  #endif
  digitalWrite(cs, HIGH);
  
  spi->endTransaction();
  delay(1);

  // release SPI interface (if callback provided)
  if (release_spi) {
    release_spi();
  }

  // unpack transfer buffer
  unpackTransferBuffer(rx_buffer);

  // flip frame counters
  for (uint8_t i = 0; i < device_count; i++) {
    devices[i].transaction.frame_counter = devices[i].transaction.frame_counter ? 0 : 1;
  }
}

void L9026::prepareRead(uint8_t device_id, uint8_t address) {
  devices[device_id].transaction.address = address;
  devices[device_id].transaction.write = false;
  devices[device_id].transaction.data = 0;
  devices[device_id].transaction.last_frame_error = 0;

  // debug print transaction
  // Serial.printf("Read Transaction: Device %d Address: %02X Frame Counter: %d\n", device_id, devices[device_id].transaction.address, devices[device_id].transaction.frame_counter);
}

void L9026::prepareWrite(uint8_t device_id, uint8_t address, uint8_t data) {
  devices[device_id].transaction.address = address;
  devices[device_id].transaction.write = true;
  devices[device_id].transaction.data = data;
  devices[device_id].transaction.last_frame_error = 0;

  // debug print transaction
  // Serial.printf("Write Transaction: Device %d Address: %02X Frame Counter: %d\n", device_id, devices[device_id].transaction.address, devices[device_id].transaction.frame_counter);
  // Serial.printf("Write Transaction - Device %d Data: %02X\n", device_id, devices[device_id].transaction.data);
}

bool L9026::getChipID() {
  Serial.printf("Getting Chip ID for %d devices\n", device_count);
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, CHIP_ID_REG);
  }

  transfer(true);

  // set device id
  for (uint8_t i = 0; i < device_count; i++) {
    devices[i].id = devices[i].transaction.data;
    Serial.printf("Device %d ID: %02X\n", i, devices[i].id);
    if (devices[i].id == 0)
      return false;
  }

  return true;
}

// Channel [7:2], MSB refers to channel 7. Bit values: 0: (default) low side 1: high side
void L9026::configureOutputSide(uint8_t device_id, OutputChannel channel, OutputType type) {
  assert(device_id < device_count);
  assert(channel >= 2 && channel <= MAX_OUTPUT_CHANNEL);

  // bit offset in the mask is 2
  devices[device_id].registers[CFG0_REG] |= (type << channel);

  // store in map
  devices[device_id].output_side[channel] = type;
}

void L9026::configureDisableResetPins(uint8_t device_id, bool disable_enabled, bool reset_enabled) {
  assert(device_id < device_count);

  if (disable_enabled) {
    devices[device_id].registers[CFG0_REG] |= 1 << 1;
  }
  else {
    devices[device_id].registers[CFG0_REG] &= ~(1 << 1);
  }

  if (reset_enabled) {
    devices[device_id].registers[CFG0_REG] |= 1;
  }
  else {
    devices[device_id].registers[CFG0_REG] &= ~1;
  }
}

// todo: check if you can configure multiple PWM outputs simultaneously
void L9026::configureLEDPWM(uint8_t device_id, L9026LEDPWMConfiguration *config) {
  assert(device_id < device_count);

  // set frequency in the CFG_2 register
  devices[device_id].registers[CFG1_REG] |= config->frequency;

  // clear last output channel from the MAP_PWM register if it exists
  if (devices[device_id].led_pwm_config != nullptr) {
    devices[device_id].registers[MAP_PWM_REG] &= ~(1 << devices[device_id].led_pwm_config->channel);
  }

  // set new output channel in the MAP_PWM register
  devices[device_id].registers[MAP_PWM_REG] |= (1 << config->channel);

  // set new output channel in the PWM_SEL register (0 = PWM_GEN, 1 = PWM_LED)
  devices[device_id].registers[PWM_SEL_REG] |= (1 << config->channel);

  // store new configuration to unset it next time
  devices[device_id].led_pwm_config = config;
}

// todo: check if you can configure multiple PWM outputs simultaneously
void L9026::configureGeneralPWM(uint8_t device_id, L9026GeneralPWMConfiguration *config) {
  assert(device_id < device_count);

  // set frequency and adjustment in the CFG_2 register (adjustment at bit offset 2)
  devices[device_id].registers[CFG2_REG] |= config->frequency;
  devices[device_id].registers[CFG2_REG] |= (config->frequency_adjustment << 2);

  // clear last output channel from the MAP_PWM register if it exists
  if (devices[device_id].general_pwm_config != nullptr) {
    devices[device_id].registers[MAP_PWM_REG] &= ~(1 << devices[device_id].general_pwm_config->channel);
  }
  
  // set new output channel in the MAP_PWM register
  devices[device_id].registers[MAP_PWM_REG] |= (1 << config->channel);

  // set new output channel in the PWM_SEL register (0 = PWM_GEN, 1 = PWM_LED)
  devices[device_id].registers[PWM_SEL_REG] &= ~(1 << config->channel);

  // store new configuration to unset it next time
  devices[device_id].general_pwm_config = config;
}

// Channel [7:0], MSB refers to channel 7. Bit values: 0: (default) no BIM active 1: BIM active
void L9026::configureBulbInrushMode(uint8_t device_id, OutputChannel channel, bool enable) {
  assert(device_id < device_count);

  // bit offset in the mask is 0
  if (enable) {
    devices[device_id].registers[BIM_REG] |= (1 << channel);
  } else {
    devices[device_id].registers[BIM_REG] &= ~(1 << channel);
  }
}

void L9026::mapInput(uint8_t device_id, L9026PWMInput input, OutputChannel channel) {
  assert(channel >= 0 && channel <= MAX_OUTPUT_CHANNEL);

  switch (input) {
    case L9026PWMInput::IN0:
      // clear last output channel from the MAP_IN0 register if it exists
      devices[device_id].registers[MAP_IN0_REG] = 0;
      devices[device_id].registers[MAP_IN0_REG] |= (1 << channel);
      break;
    case L9026PWMInput::IN1:
      // clear last output channel from the MAP_IN1 register if it exists
      devices[device_id].registers[MAP_IN1_REG] = 0;
      devices[device_id].registers[MAP_IN1_REG] |= (1 << channel);
      break;
  }
}

void L9026::setGeneralPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle) {
  assert(device_id < device_count);

  devices[device_id].registers[PWM_GEN_DC_REG] = duty_cycle;
}

void L9026::setLEDPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle) {
  assert(device_id < device_count);

  devices[device_id].registers[PWM_LED_DC_REG] = duty_cycle;
}

void L9026::setOutput(uint8_t device_id, OutputChannel channel, bool enable) {
  assert(device_id < device_count);
  assert(channel >= 0 && channel <= MAX_OUTPUT_CHANNEL);

  // bit offset in the mask is 0
  if (enable) {
    devices[device_id].registers[PWM_SPI_REG] |= (1 << channel);
  } else {
    devices[device_id].registers[PWM_SPI_REG] &= ~(1 << channel);
  }
}

void L9026::setActiveMode(uint8_t device_id, bool enable) {
  assert(device_id < device_count);

  if (enable) {
    devices[device_id].registers[CFG1_REG] |= ENABLE_ACTIVE_MODE;
  }
  else {
    devices[device_id].registers[CFG1_REG] &= ~ENABLE_ACTIVE_MODE;
  }
}

// NRES pin control (active low - low asserts reset)
void L9026::assertReset(uint8_t device_id) {
  assert(device_id < device_count);

  if (devices[device_id].reset != -1) {
    digitalWrite(devices[device_id].reset, LOW);
  }
}

void L9026::deassertReset(uint8_t device_id) {
  assert(device_id < device_count);

  if (devices[device_id].reset != -1) {
    digitalWrite(devices[device_id].reset, HIGH);
  }
}

// DIS pin control (active high - high disables outputs)
void L9026::assertDisable(uint8_t device_id) {
  assert(device_id < device_count);

  if (devices[device_id].disable != -1) {
    digitalWrite(devices[device_id].disable, HIGH);
  }
}

void L9026::deassertDisable(uint8_t device_id) {
  assert(device_id < device_count);

  if (devices[device_id].disable != -1) {
    digitalWrite(devices[device_id].disable, LOW);
  }
}

void L9026::updateConfiguration() {
  // update all configuration registers for all devices
  for (auto i = CFG0_REG; i < DIAG_OFF_EN_REG; i++) {
    if (i != RESERVED_REG) {
      for (uint8_t j = 0; j < device_count; j++) {
        prepareWrite(j, i, devices[j].registers[i]);
      }
      transfer();
      delay(1);
    }
  }
}

void L9026::updateOutputs() {
  // updates outputs + duty cycle registers
  for (uint8_t i = 0; i < device_count; i++) {
    prepareWrite(i, PWM_SPI_REG, devices[i].registers[PWM_SPI_REG]);
    transfer();
    delay(1);

    prepareWrite(i, PWM_GEN_DC_REG, devices[i].registers[PWM_GEN_DC_REG]);
    transfer();
    delay(1);

    prepareWrite(i, PWM_LED_DC_REG, devices[i].registers[PWM_LED_DC_REG]);
    transfer();
    delay(1);
  }
}

void L9026::softReset() {
  for (uint8_t i = 0; i < device_count; i++) {
    prepareWrite(i, CFG1_REG, RESET);
  }

  transfer();
  delay(1);

  // reset status + diagnostics
  for (uint8_t i = 0; i < device_count; i++) {
    devices[i].status = L9026DeviceStatus();
    for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
      devices[i].diagnostics[j] = L9026ChannelDiagnostics();
    }
  }
}

void L9026::toggleLimpHomeMode(bool inLimpHomeMode) {
  // todo: decide what to do here
  // limp mode is where idle state is low and either in0 or in1 is high
  // IN0 controls Channel 2 and IN1 controls Channel 3 in limp home mode (overriding any other configuration)  
}

void L9026::toggleSleepMode(bool inSleepMode) {
  if (inSleepMode) {
    // set all devices to sleep mode
    for (uint8_t i = 0; i < device_count; i++) {
      digitalWrite(devices[i].idle, LOW);
      digitalWrite(devices[i].in0, LOW);
      digitalWrite(devices[i].in1, LOW);
    }
  }
  else {
    // set idle high
    for (uint8_t i = 0; i < device_count; i++) {
      digitalWrite(devices[i].idle, HIGH);
    }
  }
}

void L9026::readAllDeviceStatus() {
  // read STA_0
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, STA_0_REG);
  }
  transfer(true);
  
  for (uint8_t i = 0; i < device_count; i++) {
    // bit field 6 is disable status
    devices[i].status.disable_status = (devices[i].transaction.data >> 6) & 1;
    
    // bit field 4 is idle status
    devices[i].status.idle_status = (devices[i].transaction.data >> 4) & 1;

    // bit field 3 is IN1 status
    devices[i].status.in1_status = (devices[i].transaction.data >> 3) & 1;

    // bit field 2 is IN0 status
    devices[i].status.in0_status = (devices[i].transaction.data >> 2) & 1;

    // bit field 1 is overcurrent/overtemperature detected
    devices[i].status.overcurrent_overtemperature_detected = (devices[i].transaction.data >> 1) & 1;

    // bit field 0 is off state diagnostic failure detected
    devices[i].status.off_state_diagnostic_failure_detected = devices[i].transaction.data & 1;
  }

  // read STA_1
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, STA_1_REG);
  }
  transfer(true);

  for (uint8_t i = 0; i < device_count; i++) {
    // bit field 4 is power on reset condition detected
    devices[i].status.power_on_reset_condition_detected = (devices[i].transaction.data >> 4) & 1;

    // bit field 3 is VDDIO undervoltage detected
    devices[i].status.vddio_undervoltage_detected = (devices[i].transaction.data >> 3) & 1;

    // bit field 2 is VBATT undervoltage detected
    devices[i].status.vbatt_undervoltage_detected = (devices[i].transaction.data >> 2) & 1;

    // bit field 1-0 is operating mode
    devices[i].status.operating_mode = (L9026OperatingMode)(devices[i].transaction.data & 0x3);
  }
}

L9026DeviceStatus L9026::getDeviceStatus(uint8_t device_id) {
  return devices[device_id].status;
}

void L9026::enableOffModeDiagnostics(uint8_t device_id, bool enable, uint8_t channel_mask, bool commit) {
  // enable diagnostics on masked channels
  prepareWrite(device_id, DIAG_OFF_EN_REG, channel_mask);

  if (commit) {
    transfer();
    // wait at least 1.6ms
    delayMicroseconds(1600);
  }
}

void L9026::readDeviceOffModeOpenLoadDiagnostics(uint8_t device_id) {
  assert(device_id < device_count);

  // read off mode open load detection for single device
  prepareRead(device_id, DIAG_OPL_OFF_REG);
  transfer(true);

  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no open load detected 1: open load detected
  for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
    devices[device_id].diagnostics[j].off_state_open_load_detected = (devices[device_id].transaction.data >> j) & 1;
  }
}

void L9026::readDeviceOffModeShortedLoadDiagnostics(uint8_t device_id) {
  assert(device_id < device_count);

  // read off mode short detection for single device
  prepareRead(device_id, DIAG_SHG_REG);
  transfer(true);

  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no short detected 1: short detected
  for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
    devices[device_id].diagnostics[j].shorted_load_detected = (devices[device_id].transaction.data >> j) & 1;
  }
}

void L9026::readAllDeviceOffModeDiagnostics() {
  // enable off mode diagnostics for all devices
  for (uint8_t i = 0; i < device_count; i++) {
    enableOffModeDiagnostics(i, true, 0xFF, i == device_count - 1);
  }

  // read open load + shorted diagnostics for all devices
  for (uint8_t i = 0; i < device_count; i++) {
    readDeviceOffModeOpenLoadDiagnostics(i);
    readDeviceOffModeShortedLoadDiagnostics(i);
  }

  // disable off mode diagnostics for all devices
  for (uint8_t i = 0; i < device_count; i++) {
    enableOffModeDiagnostics(i, false, 0x00, i == device_count - 1);
  }
}

void L9026::readDeviceOnModeOvercurrentOvertemperatureDiagnostics(uint8_t device_id, bool commit) {
  assert(device_id < device_count);

  // read overcurrent/overtemperature detection for single device
  prepareRead(device_id, DIAG_OVC_OVT_REG);

  if (commit) {
    transfer(true);

    // Channel [7:0], MSB refers to channel 7. Bit values: 0: no overcurrent/overtemperature detected 1: overcurrent/overtemperature detected
    for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
      devices[device_id].diagnostics[j].overcurrent_overtemperature_detected = (devices[device_id].transaction.data >> j) & 1;
    }
  }
}

// Enable on-mode open load diagnostics for a specific channel
// Must be high-side, not in PWM mode, and no OC/OT error
void L9026::enableDeviceOnModeOpenLoadDiagnostics(uint8_t device_id, OutputChannel channel, bool commit) {
  assert(device_id < device_count);
  assert(channel >= 0 && channel <= MAX_OUTPUT_CHANNEL);

  // Validate channel is eligible for on-mode open load diagnostics
  bool configured_as_high_side = devices[device_id].output_side[channel] == OutputType::HIGH_SIDE;
  bool not_configured_for_led_pwm = devices[device_id].led_pwm_config == NULL || devices[device_id].led_pwm_config->channel != channel;
  bool not_configured_for_general_pwm = devices[device_id].general_pwm_config == NULL || devices[device_id].general_pwm_config->channel != channel;
  bool no_overcurrent_overtemperature_error = !devices[device_id].diagnostics[channel].overcurrent_overtemperature_detected;

  if (!configured_as_high_side || !not_configured_for_led_pwm || !not_configured_for_general_pwm || !no_overcurrent_overtemperature_error) {
    // Channel not eligible, write 0 to maintain daisy chain sync
    prepareWrite(device_id, DIAG_OPL_ON_REG, 0);
  } else {
    // Enable diagnostics for this channel
    prepareWrite(device_id, DIAG_OPL_ON_REG, 1 << channel);
  }

  if (commit) {
    transfer();
    // Wait 210ms for diagnostics cycle to complete
    delay(210);
  }
}

// Read on-mode open load diagnostics results
void L9026::readDeviceOnModeOpenLoadDiagnosticsResults(uint8_t device_id, bool commit) {
  assert(device_id < device_count);

  // Read on mode open load detection results
  prepareRead(device_id, DIAG_OPL_ON_REG);

  if (commit) {
    transfer(true);

    // Channel [7:0], MSB refers to channel 7. Bit values: 0: no open load detected 1: open load detected
    for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
      devices[device_id].diagnostics[j].on_state_open_load_detected = (devices[device_id].transaction.data >> j) & 1;
    }
  }
}

// note: blocking. this can take up to a minimum of 210ms x max_channels (run in a separate thread)
// Runs diagnostics in parallel across all devices in the daisy chain
void L9026::readAllDeviceOnModeDiagnostics() {
  // Read overcurrent/overtemperature for all devices (batched)
  for (uint8_t i = 0; i < device_count; i++) {
    readDeviceOnModeOvercurrentOvertemperatureDiagnostics(i, i == device_count - 1);
  }

  // Process the batched read results
  for (uint8_t i = 0; i < device_count; i++) {
    for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
      devices[i].diagnostics[j].overcurrent_overtemperature_detected = (devices[i].transaction.data >> j) & 1;
    }
  }

  // Clear overcurrent/overtemperature errors before open load testing
  clearAllDeviceOvercurrentOvertemperatureErrors();

  // Test each channel across all devices in parallel (channels 2-7)
  for (auto channel = OutputChannel::CHANNEL_2; channel < OUTPUT_CHANNELS_COUNT; channel++) {
    // Enable diagnostics for this channel on all devices simultaneously
    for (uint8_t i = 0; i < device_count; i++) {
      enableDeviceOnModeOpenLoadDiagnostics(i, channel, i == device_count - 1);
    }
  }

  // Read results for all devices (batched)
  for (uint8_t i = 0; i < device_count; i++) {
    readDeviceOnModeOpenLoadDiagnosticsResults(i, i == device_count - 1);
  }

  // Process the batched read results
  for (uint8_t i = 0; i < device_count; i++) {
    for (auto j = OutputChannel::CHANNEL_0; j < OUTPUT_CHANNELS_COUNT; j++) {
      devices[i].diagnostics[j].on_state_open_load_detected = (devices[i].transaction.data >> j) & 1;
    }
  }
}

void L9026::clearAllDeviceOvercurrentOvertemperatureErrors() {
  // clear overcurrent/overtemperature error register
  for (uint8_t i = 0; i < device_count; i++) {
    // clear all channels
    prepareWrite(i, DIAG_OVC_OVT_RLW_REG, 0xFF);
  }
  transfer();
}

L9026ChannelDiagnostics L9026::getChannelDiagnostics(uint8_t device_id, OutputChannel channel) {
  return devices[device_id].diagnostics[channel];
}