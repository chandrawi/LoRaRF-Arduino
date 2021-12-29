#include <SX126x.h>
#include <SX127x.h>

// #define SX126X
#define SX127X

#if defined(SX126X)
SX126x LoRa;
#elif defined(SX127X)
SX127x LoRa;
#endif

// gateway ID
uint8_t gatewayId = 0xCC;

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

#if defined(SX126X)
  // Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, txenPin = 8, rxenPin = 7;
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }
  // Configure TCXO used in RF module
  Serial.println("Set RF module to use TCXO as clock reference");
  LoRa.setDio3TcxoCtrl(SX126X_DIO3_OUTPUT_1_8, SX126X_TCXO_DELAY_10);
#elif defined(SX127X)
  // Begin LoRa radio and set NSS, reset, IRQ, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, irqPin = 2, txenPin = 8, rxenPin = 7;
  if (!LoRa.begin(nssPin, resetPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }
#endif

  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 915 Mhz");
  LoRa.setFrequency(915000000);

  // Set RX gain to boosted gain
  Serial.println("Set RX gain to boosted gain");
  LoRa.setRxGain(LORA_RX_GAIN_BOOSTED);

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  uint8_t sf = 7;
  uint32_t bw = 125000;
  uint8_t cr = 5;
  LoRa.setLoRaModulation(sf, bw, cr);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  Serial.println("Set packet parameters:\n\tImplicit header type\n\tPreamble length = 12\n\tPayload Length = message length\n\tCRC on");
  uint8_t headerType = LORA_HEADER_IMPLICIT;
  uint16_t preambleLength = 12;
  uint8_t payloadLength = messageLen;
  bool crcType = true;
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for public network (0x3444)
  Serial.println("Set syncronize word to 0x3444");
  LoRa.setSyncWord(0x3444);

  Serial.print("\nGateway ID : 0x");
  if (gatewayId < 0x10) Serial.print("0");
  Serial.println(gatewayId, HEX);
  Serial.println("-- LORA GATEWAY --\n");
  
}

void loop() {

  // Set RF module to receive mode
  LoRa.request();

  // Wait incoming signal and demodulation process for receiving packet finish
  LoRa.wait();
  
  // Get received message object
  LoRa.get(message);

  // Print received message in serial only if gateway ID is match
  if (message.gatewayId == gatewayId){
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
  }
  else {
    Serial.print("Received message with wrong gateway ID (0x");
    if (message.gatewayId < 0x10) Serial.print("0");
    Serial.print(message.gatewayId, HEX);
    Serial.println(")");
  }

  // Print packet/signal status including RSSI and SNR
  Serial.print("Packet status : RSSI = ");
  Serial.print(LoRa.packetRssi());
  Serial.print(" dBm | SNR = ");
  Serial.print(LoRa.snr());
  Serial.println(" dB");
	
  // Show received status in case CRC or header error occur
  uint8_t status = LoRa.status();
  if (status == LORA_STATUS_CRC_ERR) Serial.println("CRC error");
  if (status == LORA_STATUS_HEADER_ERR) Serial.println("Packet header error");
  Serial.println();

}
