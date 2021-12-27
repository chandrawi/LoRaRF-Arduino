#include <SX126x.h>

SX126x LoRa;

// gateway ID and node ID
uint8_t gatewayId = 0xCC;
uint8_t nodeId = 0x77;

// Message structure to transmit
struct dataObject {
  uint8_t gatewayId;
  uint8_t nodeId;
  uint16_t messageId;
  uint32_t time;
  int32_t data;
};
dataObject message;
uint8_t messageLen = sizeof(dataObject);

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, txenPin = 8, rxenPin = 7;
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }

  // Configure TCXO used in RF module
  Serial.println("Set RF module to use TCXO as clock reference");
  uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
  uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;
  LoRa.setDio3TcxoCtrl(dio3Voltage, tcxoDelay);
  
  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 915 Mhz");
  LoRa.setFrequency(915000000);

  // Set TX power
  Serial.println("Set TX power to +22 dBm");
  LoRa.setTxPower(SX126X_TX_POWER_SX1262_22);

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  uint8_t sf = 7;
  uint32_t bw = 125000;
  uint8_t cr = 5;
  LoRa.setLoRaModulation(sf, bw, cr);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  Serial.println("Set packet parameters:\n\tImplicit header type\n\tPreamble length = 12\n\tPayload Length = message length\n\tCRC on");
  uint8_t headerType = SX126X_HEADER_IMPLICIT;
  uint16_t preambleLength = 12;
  uint8_t payloadLength = messageLen;
  bool crcType = true;
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for private network (0x1424)
  Serial.println("Set syncronize word to 0x1424");
  LoRa.setSyncWord(0x1424);

  Serial.println("\n-- LoRa Node --\n");
  
  // Assign gateway Id and node Id to message object
  message.gatewayId = gatewayId;
  message.nodeId = nodeId;
  message.messageId = 0;

}

void loop() {

  // Assign data with random value and time with current time
  message.data = random(-1073741824, 1073741824);
  message.time = millis();
  message.messageId++;
  
  // Transmit message object
  // Set transmit timeout to 250 ms
  uint32_t timeout = 250;
  LoRa.beginPacket();
  LoRa.put(message);
  LoRa.endPacket(timeout);

  // Print message in serial
  Serial.print("Gateway ID    : 0x");
  if (message.gatewayId < 0x10) Serial.print("0");
  Serial.println(message.gatewayId, HEX);
  Serial.print("Node ID       : 0x");
  if (message.nodeId < 0x10) Serial.print("0");
  Serial.println(message.nodeId, HEX);
  Serial.print("Message ID    : ");
  Serial.println(message.messageId);
  Serial.print("Time          : ");
  Serial.println(message.time);
  Serial.print("Data          : ");
  Serial.println(message.data);

  // Wait until modulation process for transmitting packet finish
  LoRa.wait();

  // Print transmit time
  Serial.print("Transmit time : ");
  Serial.print(LoRa.transmitTime());
  Serial.println(" ms");
  Serial.println();
  
  // Print status timeout when transmit process terminated due to timeout
  if (LoRa.status() == SX126X_STATUS_TX_TIMEOUT) Serial.println("Transmit timeout");

  // Put RF module to sleep in a few seconds
  LoRa.sleep();
  delay(5000);
  LoRa.wake();

}
