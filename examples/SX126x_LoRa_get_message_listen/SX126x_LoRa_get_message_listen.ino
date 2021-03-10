#include <SX126x.h>

SX126x LoRa;

// gateway ID
uint8_t deviceId = 0xCC;

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

  // Set RX gain. RX gain option are power saving gain or boosted gain 
  Serial.println("Set RX gain to boosted gain");
  LoRa.setRxGain(SX126X_RX_GAIN_BOOSTED);                            // Boosted gain

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  // Transmitter must have same SF and BW setting so receiver can receive LoRa packet
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  uint8_t sf = 7;                                                     // LoRa spreading factor: 7
  uint8_t bw = SX126X_LORA_BW_125;                                    // Bandwidth: 125 kHz
  uint8_t cr = SX126X_LORA_CR_4_5;                                    // Coding rate: 4/5
  LoRa.setLoRaModulation(sf, bw, cr);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  // The implicit header mode used, so CR and packet parameter in transmitter must be the same
  Serial.println("Set packet parameters:\n\tImplicit header type\n\tPreamble length = 12\n\tPayload Length = message length\n\tCRC on");
  uint8_t headerType = SX126X_LORA_HEADER_IMPLICIT;                   // Implicit header mode
  uint16_t preambleLength = 12;                                       // Set preamble length to 12
  uint8_t payloadLength = messageLen;                                 // Set payloadLength same as message length
  uint8_t crcType = SX126X_LORA_CRC_ON;                               // Set CRC enable
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for private network (0x1424)
  Serial.println("Set syncronize word to 0x1424");
  LoRa.setLoRaSyncWord(0x1424);

  Serial.print("\nGateway ID : 0x");
  if (deviceId < 0x10) Serial.print("0");
  Serial.println(deviceId, HEX);
  Serial.println("-- LoRa GATEWAY --\n");
  
}

void loop() {

  // Set RF module to listen mode with 20 ms RX mode and 10 ms sleep
  // Some LoRa packet will not be received if sleep period too long or preamble length too short
  uint32_t rxPeriod = 20;
  uint32_t sleepPeriod = 10;
  LoRa.listen(rxPeriod, sleepPeriod);

  // Wait until modulation process for receiving packet finish
  LoRa.wait();
  
  // Get received message object
  LoRa.get(message);

  // Print received message in serial only if gateway ID is match
  if (message.deviceId == deviceId){
    Serial.print("Node ID      : 0x");
    if (message.destinationId < 0x10) Serial.print("0");
    Serial.println(message.destinationId, HEX);
    Serial.print("Time         : ");
    Serial.println(message.time);
    Serial.print("Data         : ");
    Serial.println(message.data);
    Serial.print("Message ID   : ");
    Serial.println(message.messageId);
  }
  else {
    Serial.print("Received message with wrong gateway ID (0x");
    if (message.deviceId < 0x10) Serial.print("0");
    Serial.print(message.deviceId, HEX);
    Serial.println(")");
  }

  // Print packet/signal status including RSSI, SNR, and signalRSSI
  Serial.print("Packet status: RSSI = ");
  Serial.print(LoRa.rssi());
  Serial.print(" dBm | SNR = ");
  Serial.print(LoRa.snr());
  Serial.print(" dB | signalRSSI = ");
  Serial.print(LoRa.signalRssi());
  Serial.println(" dB");
	
  // Show received status in case CRC or header error occur
  uint8_t Status = LoRa.status();
  if (Status == SX126X_STATUS_CRC_ERR) Serial.println("CRC error");
  if (Status == SX126X_STATUS_HEADER_ERR) Serial.println("Packet header error");
  Serial.println();

}
