#include <SX127x.h>

SX127x LoRa;

// receive data container and length
const uint8_t maxLength = 15;
volatile uint8_t packetLength = 0;
uint8_t packetData[maxLength];

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Begin LoRa radio and set NSS, reset, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, irqPin = 2, txenPin = 8, rxenPin = 7;
  if (!LoRa.begin(nssPin, resetPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }

  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 915 Mhz");
  LoRa.setFrequency(915E6);

  // Set RX gain to boosted gain
  Serial.println("Set RX gain to boosted gain");
  LoRa.setRxGain(SX127X_RX_GAIN_BOOSTED);

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  LoRa.setSpreadingFactor(7);
  LoRa.setBandwidth(125000);
  LoRa.setCodeRate(5);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  Serial.println("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on");
  LoRa.setHeaderType(SX127X_HEADER_EXPLICIT);
  LoRa.setPreambleLength(12);
  LoRa.setPayloadLength(15);
  LoRa.setCrcEnable(true);

  // Set syncronize word
  Serial.println("Set syncronize word to 0x34");
  LoRa.setSyncWord(0x34);

  Serial.println("\n-- LORA RECEIVER CALLBACK --\n");

  // Register callback function to be called every RX done
  LoRa.onReceive(getReceiveData);

  // Begin request LoRa packet in continuous mode
  LoRa.request(SX127X_RX_CONTINUOUS);
}

void loop() {

  if (packetLength) {
    // Print received package
    Serial.write(packetData, packetLength - 1);
    Serial.print("  ");
    Serial.println(packetData[packetLength - 1]);

    // Print packet/signal status including RSSI, SNR, and signalRSSI
    Serial.print("Packet status: RSSI = ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR = ");
    Serial.print(LoRa.snr());
    Serial.println(" dB");
    Serial.println();

    // Reset receive data container length
    packetLength = 0;
  }
}

void getReceiveData() {

  // set received length
  packetLength = LoRa.available();
  if (packetLength > maxLength) {
    packetLength = maxLength;
  }
  // Store received data
  LoRa.read(packetData, packetLength);
}
