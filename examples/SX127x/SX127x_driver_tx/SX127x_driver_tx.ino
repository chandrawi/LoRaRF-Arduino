#include <SX127x_driver.h>

// Pin setting
int8_t nssPin = 10, resetPin = 9, irqPin = 2, rxenPin = -1, txenPin = -1;

// RF frequency setting
uint32_t frequency = 915000000;

// PA and TX power setting
uint8_t paConfig = 0xC0;
uint8_t txPower = 17;
uint8_t paPin = SX127X_TX_POWER_PA_BOOST;

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

volatile bool transmitted = false;

void checkTransmitDone() {
  transmitted = true;
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

  // Set tx power to selected TX power
  Serial.print("Set TX power to ");
  Serial.print(txPower, DEC);
  Serial.println(" dBm");
  uint8_t outputPower = txPower - 2;
  uint8_t paDac = txPower > 17 ? 0x07 : 0x04;
  sx127x_writeRegister(SX127X_REG_PA_DAC, paDac);
  sx127x_writeRegister(SX127X_REG_PA_CONFIG, paConfig | outputPower);

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

uint8_t transmitFunction(char* message, uint8_t length) {

  Serial.println("\n-- TRANSMIT FUNCTION --");
  uint8_t reg, reg_;

  // Configure FIFO address and address pointer for TX operation
  sx127x_writeRegister(SX127X_REG_FIFO_TX_BASE_ADDR, 0x00);
  reg = sx127x_readRegister(SX127X_REG_FIFO_TX_BASE_ADDR);
  sx127x_writeRegister(SX127X_REG_FIFO_ADDR_PTR, 0x00);
  reg_ = sx127x_readRegister(SX127X_REG_FIFO_ADDR_PTR);
  Serial.print("Set FIFO TX base address and address pointer (0x");
  Serial.print(reg, HEX);
  Serial.print(" | 0x");
  Serial.print(reg_, HEX);
  Serial.println(")");

  // Write message to FIFO
  Serial.print("Write message \'");
  Serial.print(message);
  Serial.println("\' in buffer");
  Serial.print("Message in bytes : [ ");
  for (uint8_t i = 0; i < length; i++) {
    sx127x_writeRegister(SX127X_REG_FIFO, message[i]);
    Serial.print((uint8_t) message[i]);
    Serial.print("  ");
  }
  Serial.println("]");

  // Set payload length
  sx127x_writeRegister(SX127X_REG_PAYLOAD_LENGTH, length);
  reg = sx127x_readRegister(SX127X_REG_PAYLOAD_LENGTH);
  Serial.print("Set payload length same as message length (");
  Serial.print(reg);
  Serial.println(")");

  // Activate interrupt when transmit done on DIO0
  Serial.println("Set TX done and timeout IRQ on DIO0");
  sx127x_writeRegister(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_TX_DONE);
  // Attach irqPin to DIO0
  Serial.println("Attach interrupt on IRQ pin");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkTransmitDone, RISING);

  // Set txen and rxen pin state for transmitting packet
  if (txenPin != -1 && rxenPin != -1) {
    digitalWrite(txenPin, HIGH);
    digitalWrite(rxenPin, LOW);
  }

  // Transmit message
  Serial.println("Transmitting message...");
  sx127x_writeRegister(SX127X_REG_OP_MODE, SX127X_LORA_MODEM | SX127X_MODE_TX);
  uint32_t tStart = millis(), tTrans = 0;

  // Wait for TX done interrupt and calcualte transmit time
  Serial.println("Wait for TX done interrupt");
  while (!transmitted) delayMicroseconds(4);
  tTrans = millis() - tStart;
  // Clear transmit interrupt flag
  transmitted = false;
  Serial.println("Transmit done");

  // Display transmit time
  Serial.print("Transmit time = ");
  Serial.print(tTrans);
  Serial.println(" ms");

  // Show IRQ flag and Clear interrupt
  uint8_t irqStat = sx127x_readRegister(SX127X_REG_IRQ_FLAGS);
  sx127x_writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);
  Serial.println("Clear IRQ status");
  if (txenPin != -1) {
    digitalWrite(txenPin, LOW);
  }

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

  // Message to transmit
  char message[] = "HeLoRa World";
  uint8_t length = sizeof(message);

  // Transmit message
  uint8_t status = transmitFunction(message, length);

  // Don't load RF module with continous transmit
  delay(10000);
}
