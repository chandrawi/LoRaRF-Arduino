#include <SX127x.h>

SX127x LoRa;

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

  // Begin LoRa radio and set NSS, reset, txen, and rxen pin with connected arduino pins
  // IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, irqPin = -1, txenPin = 8, rxenPin = 7;
  if (!LoRa.begin(nssPin, resetPin, irqPin, txenPin, rxenPin)){
    Serial.println("Something wrong, can't begin LoRa radio");
    while(1);
  }

  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 915 Mhz");
  LoRa.setFrequency(915E6);

  // Set TX power, this function will set PA config with optimal setting for requested TX power
  Serial.println("Set TX power to +17 dBm");
  LoRa.setTxPower(17, SX127X_TX_POWER_PA_BOOST);                    // TX power +17 dBm using PA boost pin

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  // Transmitter must have same SF and BW setting so receiver can receive LoRa packet
  Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
  LoRa.setSpreadingFactor(7);                                       // LoRa spreading factor: 7
  LoRa.setBandwidth(125000);                                        // Bandwidth: 125 kHz
  LoRa.setCodeRate(5);                                              // Coding rate: 4/5

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  // The explicit packet includes header contain CR, number of byte, and CRC type
  // Packet with explicit header can't be received by receiver with implicit header mode
  Serial.println("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on");
  LoRa.setHeaderType(SX127X_HEADER_EXPLICIT);                       // Explicit header mode
  LoRa.setPreambleLength(12);                                       // Set preamble length to 12
  LoRa.setPayloadLength(15);                                        // Initialize payloadLength to 15
  LoRa.setCrcEnable(true);                                          // Set CRC enable

  // Set syncronize word
  Serial.println("Set syncronize word to 0x34");
  LoRa.setSyncWord(0x34);

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
  Serial.write(message, nBytes);
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
