#include <SX126x.h>

SX126x LoRa;

// Message to transmit
char message[] = "HeLoRa World!";
uint8_t nBytes = sizeof(message);
uint8_t counter = 0;

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Uncomment below to use non default SPI port
  //SPIClass SPI_2(PB15, PB14, PB13);
  //LoRa.setSPI(SPI_2, 16000000);

  // Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected arduino pins
  // IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = -1, txenPin = 8, rxenPin = 7;
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

  // Optionally configure DIO2 as RF switch control
  // This is usually used for a LoRa module without TXEN and RXEN pins
  //LoRa.setDio2RfSwitch(true);

  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 915 Mhz");
  LoRa.setFrequency(915000000);

  // Set TX power, default power for SX1262 and SX1268 are +22 dBm and for SX1261 is +14 dBm
  // This function will set PA config with optimal setting for requested TX power
  Serial.println("Set TX power to +17 dBm");
  LoRa.setTxPower(17, SX126X_TX_POWER_SX1262);                        // TX power +17 dBm for SX1262

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  // Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  uint8_t sf = 7;                                                     // LoRa spreading factor: 7
  uint32_t bw = 125000;                                               // Bandwidth: 125 kHz
  uint8_t cr = 5;                                                     // Coding rate: 4/5
  LoRa.setLoRaModulation(sf, bw, cr);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  // The explicit packet includes header contain CR, number of byte, and CRC type
  // Receiver can receive packet with different CR and packet parameters in explicit header mode
  Serial.println("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on");
  uint8_t headerType = SX126X_HEADER_EXPLICIT;                        // Explicit header mode
  uint16_t preambleLength = 12;                                       // Set preamble length to 12
  uint8_t payloadLength = 15;                                         // Initialize payloadLength to 15
  bool crcType = true;                                                // Set CRC enable
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for public network (0x3444)
  Serial.println("Set syncronize word to 0x3444");
  LoRa.setSyncWord(0x3444);

  Serial.println("\n-- LORA TRANSMITTER --\n");
  
}

void loop() {
  
  // Transmit message and counter
  // write() method must be placed between beginPacket() and endPacket()
  LoRa.beginPacket();
  LoRa.write(message, nBytes);
  LoRa.write(counter);
  LoRa.endPacket();

  // Print message and counter in serial
  Serial.print(message);
  Serial.print("  ");
  Serial.println(counter++);

  // Wait until modulation process for transmitting packet finish
  LoRa.wait();

  // Print transmit time
  Serial.print("Transmit time: ");
  Serial.print(LoRa.transmitTime());
  Serial.println(" ms");
  Serial.println();

  // Don't load RF module with continous transmit
  delay(5000);

}
