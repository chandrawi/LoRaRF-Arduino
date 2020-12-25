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

// PA and TX power setting
uint8_t paDutyCycle = 0x02;
uint8_t hpMax = 0x03;
uint8_t deviceSel = 0x00;
uint8_t power = 0x16;

// Define modulation parameters setting
uint8_t sf = SX126X_LORA_SF_7;
uint8_t bw = SX126X_LORA_BW_125;
uint8_t cr = SX126X_LORA_CR_4_5;
uint8_t ldro = SX126X_LORA_LDRO_ON;

// Define packet parameters setting
uint16_t preambleLength = 0x0C;
uint8_t headerType = SX126X_LORA_HEADER_EXPLICIT;
uint8_t payloadLength = 64;
uint8_t crcType = SX126X_LORA_CRC_ON;
uint8_t invertIq = SX126X_LORA_IQ_STANDARD;

// SyncWord setting
uint8_t sw[2] = {0x34, 0x44};

volatile bool transmitted = false;

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Seetings for LoRa communication
  settingFunction();

}

void loop() {
  
  // Message to transmit
  char message[] = "HeLoRa World";
  uint8_t nBytes = sizeof(message);
    
  // Transmit message
  uint32_t timeout = 1000; // 1000 ms timeout
  uint16_t status = transmitFunction(message, nBytes, timeout);

  // Display status if error
  if (status & SX126X_IRQ_TIMEOUT){
    Serial.println("Receive timeout");
  }

  // Don't load RF module with continous transmit
  delay(10000);

}

void checkTransmitDone() {
  transmitted = true;
}

void settingFunction() {

  Serial.println("-- SETTING FUNCTION --");

  // Pin setting
  Serial.println("Setting pins");
  Api.setPins(nssPin, resetPin, busyPin);
  pinMode(irqPin, INPUT);
  pinMode(rxenPin, OUTPUT);
  pinMode(txenPin, OUTPUT);

  // Reset RF module by setting resetPin to LOW and begin SPI communication
  Serial.println("Resetting RF module");
  Api.reset();
  Api.begin();

  // Optionally configure TCXO or XTAL used in RF module
#ifdef SX126X_TCXO
  Serial.println("Set RF module to use TCXO as clock reference");
  Api.setDio3AsTcxoCtrl(dio3Voltage, tcxoDelay);
#endif
#ifdef SX126X_XTAL
  Serial.println("Set RF module to use XTAL as clock reference");
  Api.writeRegister(0x0911, xtalCap, 2);
#endif

  // Set to standby mode and set packet type to LoRa
  Serial.println("Going to standby mode");
  Api.setStandby(SX126X_STANDBY_RC);
  Serial.println("Set packet type to LoRa");
  Api.setPacketType(SX126X_PACKET_TYPE_LORA);

  // Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
  Serial.print("Set frequency to ");
  Serial.print(rfFrequency / 1000000);
  Serial.println(" Mhz");
  uint32_t rfFreq = rfFrequency * 33554432UL / 32000000UL;
  Api.setRfFrequency(rfFreq);

  // Set tx power to selected TX power
  Serial.print("Set TX power to ");
  Serial.print(power, DEC);
  Serial.println(" dBm");
  Api.setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01);
  Api.setTxParams(power, SX126X_PA_RAMP_200U);
  
  // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
  Serial.println("Set modulation with predefined parameters");
  Api.setModulationParamsLoRa(sf, bw, cr, ldro);

  // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
  Serial.println("Set packet with predefined parameters");
  Api.setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);

  // Set predefined syncronize word
  Serial.print("Set syncWord to 0x");
  Serial.print(sw[0], HEX);
  Serial.println(sw[1], HEX);
  Api.writeRegister(0x0740, sw, 2);

}

uint16_t transmitFunction(char* msg, uint8_t len, uint32_t timeout) {

  Serial.println("\n-- TRANSMIT FUNCTION --");

  // Set buffer base address
  Serial.println("Mark a pointer in buffer for transmit message");
  Api.setBufferBaseAddress(0x00, 0x80);

  // Write the message to buffer
  uint8_t* msgUint8 = (uint8_t*) msg;
  Serial.print("Write message \'");
  Serial.print(msg);
  Serial.println("\' in buffer");
  Api.writeBuffer(0x00, msgUint8, len);

  // Set payload length same as message length
  Serial.print("Set payload length same as message length (");
  Serial.print(len);
  Serial.println(")");
  Api.setPacketParamsLoRa(preambleLength, headerType, len, crcType, invertIq);

  // Activate interrupt when transmit done on DIO1
  Serial.println("Set TX done and timeout IRQ on DIO1");
  uint16_t mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
  Api.setDioIrqParams(mask, mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);

  // Calculate timeout (timeout duration = timeout * 15.625 us)
  uint32_t tOut = timeout * 64;
  // Set RF module to TX mode to transmit message
  Serial.println("Transmitting LoRa packet");
  Api.setTx(tOut);
  uint32_t tStart = millis(), tTrans = 0;

  // Attach irqPin to DIO1
  Serial.println("Attach interrupt on pin 2 (irqPin)");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkTransmitDone, RISING);
  digitalWrite(rxenPin, LOW);
  digitalWrite(txenPin, HIGH);

  // Wait for TX done interrupt and calcualte transmit time
  Serial.println("Wait for TX done interrupt");
  while (!transmitted) delayMicroseconds(4);
  tTrans = millis() - tStart;
  // Clear transmit interrupt flag
  transmitted = false;
  Serial.println("Packet transmitted!");

  // Display transmit time
  Serial.print("Transmit time = ");
  Serial.print(tTrans);
  Serial.println(" ms");

  // Clear the interrupt status
  uint16_t irqStat;
  Api.getIrqStatus(&irqStat);
  Serial.println("Clear IRQ status");
  Api.clearIrqStatus(irqStat);
  digitalWrite(txenPin, LOW);
  
  // return interrupt status
  return irqStat;

}