#include <SX126x_API.h>

SX126x_API Api;

// Pin setting
int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, rxenPin = 7, txenPin = 8;

// Clock reference setting. RF module using either TCXO or XTAL as clock reference
// uncomment code below to use XTAL
//#define SX126X_XTAL
//uint8_t xtalCap[2] = {0x12, 0x12};
// uncomment code below to use TCXO
#define SX126X_TCXO
uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;

// RF frequency setting
uint32_t rfFrequency = 915000000UL;

// RX gain setting
uint8_t gain = SX126X_POWER_SAVING_GAIN;

// Define modulation parameters setting
uint8_t sf = 7;                               // spreading factor 7
uint8_t bw = SX126X_BW_125000;                // 125 kHz
uint8_t cr = SX126X_CR_4_5;                   // 4/5 code rate
uint8_t ldro = SX126X_LDRO_OFF;               // low data rate optimize off

// Define packet parameters setting
uint16_t preambleLength = 12;                 // 12 bytes preamble
uint8_t headerType = SX126X_HEADER_EXPLICIT;  // explicit packet header
uint8_t payloadLength = 64;                   // 64 bytes payload
uint8_t crcType = SX126X_CRC_ON;              // cyclic redundancy check (CRC) on
uint8_t invertIq = SX126X_IQ_STANDARD;        // standard IQ setup

// SyncWord setting
uint8_t sw[2] = {0x34, 0x44};

volatile bool received = false;

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Seetings for LoRa communication
  settingFunction();

}

void loop() {

  // Receive message
  char message[20];
  uint8_t msgLen;
  uint32_t timeout = 5000; // 5000 ms timeout
  uint16_t status = receiveFunction(message, msgLen, timeout);

  // Display message if receive success or display status if error
  if (status & SX126X_IRQ_RX_DONE){
    Serial.print("Message: \'");
    for (uint8_t i=0; i< msgLen; i++){
      Serial.print(message[i]);
    }
    Serial.println("\'");
  }
  else if (status & SX126X_IRQ_TIMEOUT){
    Serial.println("Receive timeout");
  }
  else if(status & SX126X_IRQ_CRC_ERR){
    Serial.println("CRC error");
  }

}

void checkReceiveDone() {
  received = true;
}

void settingFunction() {

  Serial.println("-- SETTING FUNCTION --");

  // Pin setting
  Serial.println("Setting pins");
  Api.setPins(nssPin, busyPin);

  // Reset RF module by setting resetPin to LOW and begin SPI communication
  Serial.println("Resetting RF module");
  Api.reset(resetPin);
  Api.begin();

  // Optionally configure TCXO or XTAL used in RF module
#ifdef SX126X_TCXO
  Serial.println("Set RF module to use TCXO as clock reference");
  Api.setDio3AsTcxoCtrl(dio3Voltage, tcxoDelay);
#endif
#ifdef SX126X_XTAL
  Serial.println("Set RF module to use XTAL as clock reference");
  Api.writeRegister(SX126X_REG_XTA_TRIM, xtalCap, 2);
#endif

  // Set to standby mode and set packet type to LoRa
  Serial.println("Going to standby mode");
  Api.setStandby(SX126X_STANDBY_RC);
  Serial.println("Set packet type to LoRa");
  Api.setPacketType(SX126X_LORA_MODEM);

  // Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
  Serial.print("Set frequency to ");
  Serial.print(rfFrequency / 1000000);
  Serial.println(" Mhz");
  uint32_t rfFreq = ((uint64_t) rfFrequency * 33554432UL) / 32000000UL;
  Api.setRfFrequency(rfFreq);

  // Set rx gain to selected gain
  Serial.print("Set RX gain to ");
  if (gain == SX126X_POWER_SAVING_GAIN) Serial.println("power saving gain");
  else if (gain == SX126X_BOOSTED_GAIN) Serial.println("boosted gain");
  Api.writeRegister(SX126X_REG_RX_GAIN, &gain, 1);
  
  // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
  Serial.println("Set modulation with predefined parameters");
  Api.setModulationParamsLoRa(sf, bw, cr, ldro);

  // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
  Serial.println("Set packet with predefined parameters");
  Api.setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);

  // Set predefined syncronize word
  Serial.print("Set syncWord to 0x");
  Serial.println((sw[0] << 8) + sw[1], HEX);
  Api.writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sw, 2);

}

uint16_t receiveFunction(char* msg, uint8_t &len, uint32_t timeout) {

  Serial.println("\n-- RECEIVE FUNCTION --");

  // Activate interrupt when receive done on DIO1
  Serial.println("Set RX done, timeout, and CRC error IRQ on DIO1");
  uint16_t mask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR;
  Api.setDioIrqParams(mask, mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);

  // Calculate timeout (timeout duration = timeout * 15.625 us)
  uint32_t tOut = timeout * 64;
  if (timeout == SX126X_RX_CONTINUOUS) tOut = SX126X_RX_CONTINUOUS;
  // Set RF module to RX mode to receive message
  Serial.println("Receiving LoRa packet within predefined timeout");
  Api.setRx(tOut);

  // Attach irqPin to DIO1
  Serial.println("Attach interrupt on pin 2 (irqPin)");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkReceiveDone, RISING);
  // Set txen and rxen pin state for receiving packet
  digitalWrite(txenPin, LOW);
  digitalWrite(rxenPin, HIGH);

  // Wait for RX done interrupt
  Serial.println("Wait for RX done interrupt");
  while (!received) delayMicroseconds(4);
  // Clear transmit interrupt flag
  received = false;

  // Clear the interrupt status
  uint16_t irqStat;
  Api.getIrqStatus(&irqStat);
  Serial.println("Clear IRQ status");
  Api.clearIrqStatus(irqStat);
  digitalWrite(rxenPin, LOW);

  // Exit function if timeout reached
  if (irqStat & SX126X_IRQ_TIMEOUT) return irqStat;
  Serial.println("Packet received!");

  // Get last received length and buffer base address
  Serial.println("Get received length and buffer base address");
  uint8_t payloadLengthRx, rxStartBufferPointer;
  Api.getRxBufferStatus(&payloadLengthRx, &rxStartBufferPointer);
  uint8_t message[payloadLengthRx];

  // Get and display packet status
  Serial.println("Get received packet status");
  uint8_t rssiPkt, snrPkt, signalRssiPkt;
  Api.getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
  float rssi = rssiPkt / -2;
  float snr = snrPkt / 4;
  float signalRssi = signalRssiPkt / -2;
  Serial.print("Packet status: RSSI = ");
  Serial.print(rssi);
  Serial.print(" | SNR = ");
  Serial.print(snr);
  Serial.print(" | signalRSSI = ");
  Serial.println(signalRssi);

  // Read message from buffer
  Serial.println("Read message from buffer");
  Api.readBuffer(rxStartBufferPointer, message, payloadLengthRx);
  len = payloadLengthRx;
  for (uint8_t i=0; i<len; i++){
    msg[i] = (char) message[i];
  }

  // return interrupt status
  return irqStat;

}