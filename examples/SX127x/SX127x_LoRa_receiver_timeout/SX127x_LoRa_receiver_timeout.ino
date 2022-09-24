#include <SX127x.h>

SX127x LoRa;

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Begin LoRa radio and set NSS, reset, txen, and rxen pin with connected arduino pins
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, irqPin = -1, txenPin = 8, rxenPin = 7;
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

  Serial.println("\n-- LORA RECEIVER TIMEOUT --\n");

}

void loop() {

  // Request for receiving new LoRa packet within 1000 ms
  LoRa.request(1000);
  // Wait for incoming LoRa packet
  LoRa.wait();

  // Only show message if receive process is done
  uint8_t status = LoRa.status();
  if (status == SX127X_STATUS_RX_DONE) {

    // Put received packet to message and counter variable
    const uint8_t msgLen = LoRa.available() - 1;
    char message[msgLen];
    LoRa.read(message, msgLen);
    uint8_t counter = LoRa.read();

    // Print received message and counter in serial
    Serial.write(message, msgLen);
    Serial.print("  ");
    Serial.println(counter);

    // Print packet / signal status
    Serial.print("RSSI: ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR: ");
    Serial.print(LoRa.snr());
    Serial.println(" dB");
    Serial.println();

  } else {

    // Show received status
    if (status == SX127X_STATUS_RX_TIMEOUT) Serial.println("Receive timeout");
    else if (status == SX127X_STATUS_CRC_ERR) Serial.println("CRC error");
    else if (status == SX127X_STATUS_HEADER_ERR) Serial.println("Packet header error");
    Serial.println();

  }

}
