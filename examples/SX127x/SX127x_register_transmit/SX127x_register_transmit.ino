#include <SX127x.h>

SX127x LoRa;

// Frequency, modulation param, packet param, and synchronize word setting
uint32_t frequency = 915000000;
uint8_t sf = 7;
uint8_t bw = 7;                               // 125 khz
uint8_t cr = 1;                               // 5/4
uint8_t headerType = SX127X_HEADER_EXPLICIT;
uint16_t preambleLen = 12;
uint8_t crcEn = 1;
uint8_t syncword = 0x12;

// Message to tramsmit
char message[14] = "HeLoRa world!";
uint8_t payloadLength = sizeof(message) - 1;
uint8_t fifoAddress = 0;

void setup() {

  Serial.begin(38400);

  // Begin LoRa radio
  if (LoRa.begin(10, 9, 2)) {
    Serial.println("Begin LoRa radio");
  } else {
    Serial.println("Something wrong, can't begin LoRa radio");
  }

  // Set modem type to LoRa and put device to standby mode
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_MODE_SLEEP);
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_LONG_RANGE_MODE);
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_LONG_RANGE_MODE | SX127X_MODE_STDBY);
  Serial.println("Set modem type to LoRa");

  // Set frequency
  uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
  LoRa.writeRegister(SX127X_REG_FRF_MSB, (uint8_t) (frf >> 16));
  LoRa.writeRegister(SX127X_REG_FRF_MID, (uint8_t) (frf >> 8));
  LoRa.writeRegister(SX127X_REG_FRF_LSB, (uint8_t) frf);
  Serial.print("Set frequency to : ");
  Serial.print(frequency / 1000000);
  Serial.println(" MHz");

  uint8_t reg;

  // Set modulation param and packet param
  LoRa.writeBits(SX127X_REG_MODEM_CONFIG_2, sf, 4, 4);
  LoRa.writeBits(SX127X_REG_MODEM_CONFIG_1, bw, 4, 4);
  LoRa.writeBits(SX127X_REG_MODEM_CONFIG_1, cr, 1, 3);
  LoRa.writeBits(SX127X_REG_MODEM_CONFIG_1, headerType, 0, 1);
  LoRa.writeBits(SX127X_REG_MODEM_CONFIG_2, crcEn, 2, 1);
  LoRa.writeRegister(SX127X_REG_PREAMBLE_MSB, preambleLen >> 8);
  LoRa.writeRegister(SX127X_REG_PREAMBLE_LSB, preambleLen);

  // Show modulation param and packet param registers
  reg = LoRa.readRegister(SX127X_REG_MODEM_CONFIG_1);
  Serial.print("Modem config 1 : 0x");
  Serial.println(reg, HEX);
  reg = LoRa.readRegister(SX127X_REG_MODEM_CONFIG_2);
  Serial.print("Modem config 2 : 0x");
  Serial.println(reg, HEX);
  reg = LoRa.readRegister(SX127X_REG_PREAMBLE_MSB);
  Serial.print("Preamble MSB : 0x");
  Serial.println(reg, HEX);
  reg = LoRa.readRegister(SX127X_REG_PREAMBLE_LSB);
  Serial.print("Preamble LSB : 0x");
  Serial.println(reg, HEX);

  // Set synchronize word
  LoRa.writeRegister(SX127X_REG_SYNC_WORD, syncword);
  reg = LoRa.readRegister(SX127X_REG_SYNC_WORD);
  Serial.print("Synchronize word : 0x");
  Serial.println(reg, HEX);

  // Configure FIFO address and address pointer for TX operation
  LoRa.writeRegister(SX127X_REG_FIFO_TX_BASE_ADDR, fifoAddress);
  reg = LoRa.readRegister(SX127X_REG_FIFO_TX_BASE_ADDR);
  Serial.print("FIFO TX base address : 0x");
  Serial.println(reg, HEX);
  LoRa.writeRegister(SX127X_REG_FIFO_ADDR_PTR, fifoAddress);
  reg = LoRa.readRegister(SX127X_REG_FIFO_ADDR_PTR);
  Serial.print("FIFO address pointer : 0x");
  Serial.println(reg, HEX);

  // Write message to FIFO
  Serial.print("Message to transmit : ");
  Serial.println(message);
  Serial.print("Message in bytes : [ ");
  for (uint8_t i = 0; i < payloadLength; i++) {
    LoRa.writeRegister(SX127X_REG_FIFO, message[i]);
    Serial.print((uint8_t) message[i]);
    Serial.print("  ");
  }
  Serial.println("]");

  // Set payload length
  LoRa.writeRegister(SX127X_REG_PAYLOAD_LENGTH, payloadLength);
  reg = LoRa.readRegister(SX127X_REG_PAYLOAD_LENGTH);
  Serial.print("Message length : ");
  Serial.println(reg);

  // Show address pointer after write message
  reg = LoRa.readRegister(SX127X_REG_FIFO_ADDR_PTR);
  Serial.print("FIFO address pointer : ");
  Serial.println(reg);

  // Transmit message
  Serial.println("Transmitting message...");
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_LORA_MODEM | SX127X_MODE_TX);
  reg = 0x00;
  while ((reg & SX127X_IRQ_TX_DONE) == 0) {
    reg = LoRa.readRegister(SX127X_REG_IRQ_FLAGS);
    yield();
  }
  Serial.println("Transmit done");

  // Show IRQ flag and Clear interrupt
  Serial.print("IRQ flag status : 0x");
  Serial.println(reg, HEX);
  LoRa.writeRegister(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE);
  Serial.println("Clear IRQ");

}

void loop() {
  
}
