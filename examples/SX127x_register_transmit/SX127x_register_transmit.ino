#include <SX127x.h>

SX127x LoRa;

uint32_t frequency = 915000000;
uint8_t fifoAddress = 0;
const uint8_t payloadLength = 6;
uint8_t message[payloadLength] = {104, 101, 108, 111, 114, 97};

void setup() {
  Serial.begin(38400);

  LoRa.begin(10, 9, 2);

  uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
  LoRa.writeRegister(SX127X_REG_FRF_MSB, (uint8_t) (frf >> 16));
  LoRa.writeRegister(SX127X_REG_FRF_MID, (uint8_t) (frf >> 8));
  LoRa.writeRegister(SX127X_REG_FRF_LSB, (uint8_t) frf);
  Serial.print("Set frequency to : ");
  Serial.print(frequency / 1000000);
  Serial.println(" MHz");

  uint8_t reg;

  reg = LoRa.readRegister(SX127X_REG_MODEM_CONFIG_1);
  Serial.print("Modem config 1 : ");
  Serial.println(reg);

  reg = LoRa.readRegister(SX127X_REG_MODEM_CONFIG_2);
  Serial.print("Modem config 2 : ");
  Serial.println(reg);

  reg = LoRa.readRegister(SX127X_REG_MODEM_CONFIG_3);
  Serial.print("Modem config 3 : ");
  Serial.println(reg);

  reg = LoRa.readRegister(SX127X_REG_SYNC_WORD);
  Serial.print("Synchronize word : ");
  Serial.println(reg);

  LoRa.writeRegister(SX127X_REG_FIFO_TX_BASE_ADDR, fifoAddress);
  reg = LoRa.readRegister(SX127X_REG_FIFO_TX_BASE_ADDR);
  Serial.print("FIFO TX base address : ");
  Serial.println(reg);

  LoRa.writeRegister(SX127X_REG_FIFO_ADDR_PTR, fifoAddress);
  reg = LoRa.readRegister(SX127X_REG_FIFO_ADDR_PTR);
  Serial.print("FIFO address pointer : ");
  Serial.println(reg);

  Serial.print("Message to transmit : ");
  for (uint8_t i = 0; i < payloadLength; i++) {
    LoRa.writeRegister(SX127X_REG_FIFO, message[i]);
    Serial.print(message[i]);
    Serial.print("  ");
  }
  Serial.println();

  reg = LoRa.readRegister(SX127X_REG_FIFO_ADDR_PTR);
  Serial.print("FIFO address pointer : ");
  Serial.println(reg);

  LoRa.writeRegister(SX127X_REG_PAYLOAD_LENGTH, payloadLength);
  reg = LoRa.readRegister(SX127X_REG_PAYLOAD_LENGTH);
  Serial.print("Message payload : ");
  Serial.println(reg);

  Serial.println("Transmitting message...");
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_LORA_MODEM | SX127X_MODE_TX);

  while ((LoRa.readRegister(SX127X_REG_IRQ_FLAGS) & SX127X_IRQ_TX_DONE) == 0) {
    yield();
  }
  Serial.println("Transmit done");

  LoRa.writeRegister(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE);
  Serial.println("Clear IRQ");

  reg = LoRa.readRegister(SX127X_REG_OP_MODE);
  Serial.print("Device mode : ");
  Serial.println(reg);

}

void loop() {
  
}
