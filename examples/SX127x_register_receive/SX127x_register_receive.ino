#include <SX127x.h>

SX127x LoRa;

uint32_t frequency = 915000000;
uint8_t fifoAddress = 0;
uint8_t payloadLength = 6;
uint8_t message[6];

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

  Serial.println("Receiving message...");
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_LORA_MODEM | SX127X_MODE_RX_CONTINUOUS);

  while ((LoRa.readRegister(SX127X_REG_IRQ_FLAGS) & (SX127X_IRQ_RX_DONE | SX127X_IRQ_CRC_ERR)) == 0) {
    yield();
  }
  Serial.println("Receive done");

  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127X_LORA_MODEM | SX127X_MODE_STDBY);
  Serial.println("Standby mode");

  reg = LoRa.readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR);
  Serial.print("FIFO RX base address : ");
  Serial.println(reg);

  LoRa.writeRegister(SX127X_REG_FIFO_ADDR_PTR, reg);
  Serial.println("Set FIFO address pointer to FIFO RX base address");

  payloadLength = LoRa.readRegister(SX127X_REG_RX_NB_BYTES);
  Serial.print("Message payload : ");
  Serial.println(payloadLength);

  Serial.print("Received message : ");
  for (uint8_t i = 0; i < payloadLength; i++) {
    reg = LoRa.readRegister(SX127X_REG_FIFO);
    Serial.print(reg);
    Serial.print("  ");
  }
  Serial.println();

  LoRa.writeRegister(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE);
  Serial.println("Clear IRQ");

  reg = LoRa.readRegister(SX127X_REG_OP_MODE);
  Serial.print("Device mode : ");
  Serial.println(reg);

}

void loop() {
  
}
