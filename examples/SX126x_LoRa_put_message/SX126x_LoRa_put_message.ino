#include <SX126x.h>

SX126x LoRa;

// gateway ID and node ID
uint8_t deviceId = 0xCC;
uint8_t destinationId = 0x77;

// Message structure to transmit
struct dataObject {
  uint8_t deviceId;
  uint8_t destinationId;
  uint32_t time;
  uint16_t data;
  uint16_t messageId;
};
dataObject message;
uint8_t messageLen = sizeof(dataObject);

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected arduino pins
  // Set txen and rxen pin to -1 if RF module doesn't have one
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, txenPin = 8, rxenPin = 7;
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }

  // Optionally configure TCXO or XTAL used in RF module
  // Different RF module can have different clock, so make sure clock source is configured correctly
  // uncomment code below to use TCXO
  Serial.println("Set RF module to use TCXO as clock reference");
  uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
  uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;
  LoRa.setDio3TcxoCtrl(dio3Voltage, tcxoDelay);
  // uncomment code below to use XTAL
  //uint8_t xtalA = 0x12;
  //uint8_t xtalB = 0x12;
  //Serial.println("Set RF module to use XTAL as clock reference");
  //LoRa.setXtalCap(xtalA, xtalB);
  
  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 915 Mhz");
  LoRa.setFrequency(915000000);

  // Set TX power, default power for SX1262 and SX1268 are +22 dBm and for SX1261 is +14 dBm
  // This function will set PA config with optimal setting for requested TX power
  Serial.println("Set TX power to +22 dBm");
  LoRa.setTxPower(SX126X_TX_POWER_SX1262_22);                          // TX power +22 dBm for SX1262

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  // Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  uint8_t sf = 7;                                                     // LoRa spreading factor: 7
  uint8_t bw = SX126X_LORA_BW_125;                                    // Bandwidth: 125 kHz
  uint8_t cr = SX126X_LORA_CR_4_5;                                    // Coding rate: 4/5
  LoRa.setLoRaModulation(sf, bw, cr);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  // The implicit header mode used, so CR and packet parameter in receiver must be the same
  Serial.println("Set packet parameters:\n\tImplicit header type\n\tPreamble length = 12\n\tPayload Length = message length\n\tCRC on");
  uint8_t headerType = SX126X_LORA_HEADER_IMPLICIT;                   // Implicit header mode
  uint16_t preambleLength = 12;                                       // Set preamble length to 12
  uint8_t payloadLength = messageLen;                                 // Set payloadLength same as message length
  bool crcType = true;                                                // Set CRC enable
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for private network (0x1424)
  Serial.println("Set syncronize word to 0x1424");
  LoRa.setLoRaSyncWord(0x1424);

  Serial.println("\n-- LoRa NODE --\n");
  
  // Assign gateway Id and node Id to message object
  message.deviceId = deviceId;
  message.destinationId = destinationId;
  message.messageId = 0;

}

void loop() {

  // Assign data with value of analog read A0 and time with current time
  message.data = analogRead(A0);
  message.time = millis();
  message.messageId++;
  
  // Transmit message object
  // Set transmit timeout to 250 ms
  uint32_t timeout = 250;
  LoRa.beginPacket();
  LoRa.put(message);
  LoRa.endPacket(timeout);

  // Print message in serial
  Serial.print("Gateway ID   : 0x");
  if (message.deviceId < 0x10) Serial.print("0");
  Serial.println(message.deviceId, HEX);
  Serial.print("Node ID      : 0x");
  if (message.destinationId < 0x10) Serial.print("0");
  Serial.println(message.destinationId, HEX);
  Serial.print("Time         : ");
  Serial.println(message.time);
  Serial.print("Data         : ");
  Serial.println(message.data);
  Serial.print("Message ID   : ");
  Serial.println(message.messageId);

  // Wait until modulation process for transmitting packet finish
  LoRa.wait();

  // Print transmit time
  // Transmit time show the actual modulation time for transmitting LoRa packet
  Serial.print("Transmit time: ");
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
