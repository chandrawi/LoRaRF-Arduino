#include <SX127x_driver.h>

// Pin setting
int8_t nssPin = 10, resetPin = 9, irqPin = 2, rxenPin = -1, txenPin = -1;

// RF frequency setting
uint32_t frequency = 915000000;

// RX gain setting
uint8_t boost = SX127X_RX_GAIN_POWER_SAVING;
uint8_t level = SX127X_RX_GAIN_AUTO;

// Define modulation parameters setting
uint8_t sf = 7;
uint8_t bw = 7;                               // 125 khz
uint8_t cr = 1;                               // 5/4

// Define packet parameters setting
uint8_t headerType = SX127X_HEADER_EXPLICIT;
uint16_t preambleLen = 12;
uint8_t crcEn = 1;

// SyncWord setting
uint8_t syncword = 0x12;

volatile bool received = false;

void checkReceiveDone() {
  received = true;
}

void settingFunction() {

  Serial.println("-- SETTING FUNCTION --");

  // Pin setting
  Serial.println("Setting pins");
  sx127x_setPins(nssPin);
  pinMode(irqPin, INPUT);
  if (txenPin != -1 && rxenPin != -1) {
    pinMode(txenPin, OUTPUT);
    pinMode(rxenPin, OUTPUT);
  }

  // Reset RF module by setting resetPin to LOW and begin SPI communication
  sx127x_reset(resetPin);
  sx127x_begin();
  uint8_t version = sx127x_readRegister(SX127X_REG_VERSION);
  if (version == 0x12 || version == 0x22) {
    Serial.println("Resetting RF module");
  } else {
    Serial.println("Something wrong, can't reset LoRa radio");
  }

  // Set modem type to LoRa and put device to standby mode
  sx127x_writeRegister(SX127X_REG_OP_MODE, SX127X_MODE_SLEEP);
  sx127x_writeRegister(SX127X_REG_OP_MODE, SX127X_LONG_RANGE_MODE);
  sx127x_writeRegister(SX127X_REG_OP_MODE, SX127X_LONG_RANGE_MODE | SX127X_MODE_STDBY);
  Serial.println("Going to standby mode");
  Serial.println("Set packet type to LoRa");

  // Set frequency
  uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
  sx127x_writeRegister(SX127X_REG_FRF_MSB, (uint8_t) (frf >> 16));
  sx127x_writeRegister(SX127X_REG_FRF_MID, (uint8_t) (frf >> 8));
  sx127x_writeRegister(SX127X_REG_FRF_LSB, (uint8_t) frf);
  Serial.print("Set frequency to ");
  Serial.print(frequency / 1000000);
  Serial.println(" MHz");

  // Set rx gain to selected gain
  Serial.print("Set RX gain to ");
  if (boost == SX127X_RX_GAIN_POWER_SAVING) Serial.println("power saving gain");
  else if (boost == SX127X_RX_GAIN_BOOSTED) Serial.println("boosted gain");
  uint8_t LnaBoostHf = boost ? 0x03 : 0x00;
  uint8_t AgcOn = level == SX127X_RX_GAIN_AUTO ? 0x01 : 0x00;
  sx127x_writeRegister(SX127X_REG_LNA, LnaBoostHf | (level << 5));
  sx127x_writeBits(SX127X_REG_MODEM_CONFIG_3, AgcOn, 2, 1);

  // Set modulation param and packet param
  Serial.println("Set modulation with predefined parameters");
  Serial.println("Set packet with predefined parameters");
  sx127x_writeBits(SX127X_REG_MODEM_CONFIG_2, sf, 4, 4);
  sx127x_writeBits(SX127X_REG_MODEM_CONFIG_1, bw, 4, 4);
  sx127x_writeBits(SX127X_REG_MODEM_CONFIG_1, cr, 1, 3);
  sx127x_writeBits(SX127X_REG_MODEM_CONFIG_1, headerType, 0, 1);
  sx127x_writeBits(SX127X_REG_MODEM_CONFIG_2, crcEn, 2, 1);
  sx127x_writeRegister(SX127X_REG_PREAMBLE_MSB, preambleLen >> 8);
  sx127x_writeRegister(SX127X_REG_PREAMBLE_LSB, preambleLen);

  // Show modulation param and packet param registers
  uint8_t reg, reg_;
  reg = sx127x_readRegister(SX127X_REG_MODEM_CONFIG_1);
  Serial.print("Modem config 1 : 0x");
  Serial.println(reg, HEX);
  reg = sx127x_readRegister(SX127X_REG_MODEM_CONFIG_2);
  Serial.print("Modem config 2 : 0x");
  Serial.println(reg, HEX);
  reg = sx127x_readRegister(SX127X_REG_PREAMBLE_MSB);
  reg_ = sx127x_readRegister(SX127X_REG_PREAMBLE_LSB);
  Serial.print("Preamble length : 0x");
  Serial.println(reg * 256 + reg_, HEX);

  // Set synchronize word
  sx127x_writeRegister(SX127X_REG_SYNC_WORD, syncword);
  reg = sx127x_readRegister(SX127X_REG_SYNC_WORD);
  Serial.print("Set syncWord to 0x");
  Serial.println(reg, HEX);
}

uint8_t receiveFunction(char* message, uint8_t &length) {

  Serial.println("\n-- RECEIVE FUNCTION --");
  uint8_t reg, reg_;

  // Activate interrupt when transmit done on DIO0
  Serial.println("Set RX done IRQ on DIO0");
  sx127x_writeRegister(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_RX_DONE);
  // Attach irqPin to DIO0
  Serial.println("Attach interrupt on IRQ pin");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkReceiveDone, RISING);

  // Set txen and rxen pin state for receiving packet
  if (txenPin != -1 && rxenPin != -1) {
    digitalWrite(txenPin, LOW);
    digitalWrite(rxenPin, HIGH);
  }

  // Receive message
  Serial.println("Receiving message...");
  sx127x_writeRegister(SX127X_REG_OP_MODE, SX127X_LORA_MODEM | SX127X_MODE_RX_CONTINUOUS);

  // Wait for RX done interrupt
  Serial.println("Wait for RX done interrupt");
  while (!received) delayMicroseconds(4);
  Serial.println("Receive done");
  // Clear transmit interrupt flag
  received = false;

  // Set mode to standby to end RX mode
  sx127x_writeBits(SX127X_REG_OP_MODE, SX127X_MODE_STDBY, 0, 3);
  Serial.println("Going to standby mode");

  // Clear the interrupt status
  uint8_t irqStat = sx127x_readRegister(SX127X_REG_IRQ_FLAGS);
  sx127x_writeRegister(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_RX_DONE);
  Serial.println("Clear IRQ status");
  if (rxenPin != -1) {
    digitalWrite(rxenPin, LOW);
  }

  // Get FIFO address of received message and configure address pointer
  reg = sx127x_readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR);
  sx127x_writeRegister(SX127X_REG_FIFO_ADDR_PTR, reg);
  Serial.print("Set FIFO address pointer to FIFO RX base address (0x");
  Serial.print(reg);
  Serial.println(")");

  // Get payload length
  length = sx127x_readRegister(SX127X_REG_RX_NB_BYTES);
  Serial.print("Get message length (");
  Serial.print(length);
  Serial.println(")");

  // Get and display packet status
  Serial.println("Get received packet status");
  float rssi = ((int16_t) sx127x_readRegister(SX127X_REG_PKT_RSSI_VALUE) - SX127X_RSSI_OFFSET_HF);
  float snr = (int8_t) sx127x_readRegister(SX127X_REG_PKT_SNR_VALUE) / 4.0;
  Serial.print("Packet status: RSSI = ");
  Serial.print(rssi);
  Serial.print(" | SNR = ");
  Serial.println(snr);

  // Read message from buffer
  Serial.print("Message in bytes : [ ");
  for (uint8_t i = 0; i < length; i++) {
    reg = sx127x_readRegister(SX127X_REG_FIFO);
    message[i] = (char) reg;
    Serial.print(reg);
    Serial.print("  ");
  }
  Serial.println("]");

  // return interrupt status
  return irqStat;
}

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Seetings for LoRa communication
  settingFunction();
}

void loop() {

  // Receive message
  char message[13];
  uint8_t length;
  uint8_t status = receiveFunction(message, length);

  // Display message if receive success or display status if error
  if (status & SX127X_IRQ_RX_DONE){
    Serial.print("Message: \'");
    for (uint8_t i=0; i< length; i++){
      Serial.print(message[i]);
    }
    Serial.println("\'");
  }
  else if(status & SX127X_IRQ_CRC_ERR){
    Serial.println("CRC error");
  }
}
