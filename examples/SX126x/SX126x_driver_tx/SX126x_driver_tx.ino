#include <SX126x_driver.h>

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

// Configure DIO2 as RF switch control or using TXEN and RXEN pin
#define SX126X_USING_TXEN_RXEN

// RF frequency setting
uint32_t rfFrequency = 915000000UL;

// PA and TX power setting
uint8_t paDutyCycle = 0x02;
uint8_t hpMax = 0x03;
uint8_t deviceSel = 0x00;
uint8_t power = 0x16;

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

volatile bool transmitted = false;

void checkTransmitDone() {
  transmitted = true;
}

void settingFunction() {

  Serial.println("-- SETTING FUNCTION --");

  // Pin setting
  Serial.println("Setting pins");
  sx126x_setPins(nssPin, busyPin);
  pinMode(irqPin, INPUT);

  // Reset RF module by setting resetPin to LOW and begin SPI communication
  Serial.println("Resetting RF module");
  sx126x_reset(resetPin);
  sx126x_begin();

  // Set to standby mode
  sx126x_setStandby(SX126X_STANDBY_RC);
  if (!sx126x_busyCheck()) {
    Serial.println("Going to standby mode");
  } else {
    Serial.println("Something wrong, can't set to standby mode");
  }

  // Optionally configure TCXO or XTAL used in RF module
#ifdef SX126X_TCXO
  Serial.println("Set RF module to use TCXO as clock reference");
  sx126x_setDio3AsTcxoCtrl(dio3Voltage, tcxoDelay);
#endif
#ifdef SX126X_XTAL
  Serial.println("Set RF module to use XTAL as clock reference");
  sx126x_writeRegister(SX126X_REG_XTA_TRIM, xtalCap, 2);
#endif

  // Optionally configure DIO2 as RF switch control
#ifdef SX126X_USING_TXEN_RXEN
  pinMode(txenPin, OUTPUT);
  pinMode(rxenPin, OUTPUT);
#else
  Serial.println("Set RF switch is controlled by DIO2");
  sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);
#endif

  // Set packet type to LoRa
  Serial.println("Set packet type to LoRa");
  sx126x_setPacketType(SX126X_LORA_MODEM);

  // Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
  Serial.print("Set frequency to ");
  Serial.print(rfFrequency / 1000000);
  Serial.println(" Mhz");
  uint32_t rfFreq = ((uint64_t) rfFrequency * 33554432UL) / 32000000UL;
  sx126x_setRfFrequency(rfFreq);

  // Set tx power to selected TX power
  Serial.print("Set TX power to ");
  Serial.print(power, DEC);
  Serial.println(" dBm");
  sx126x_setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01);
  sx126x_setTxParams(power, SX126X_PA_RAMP_200U);

  // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
  Serial.println("Set modulation with predefined parameters");
  sx126x_setModulationParamsLoRa(sf, bw, cr, ldro);

  // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
  Serial.println("Set packet with predefined parameters");
  sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);

  // Set predefined syncronize word
  Serial.print("Set syncWord to 0x");
  Serial.println((sw[0] << 8) + sw[1], HEX);
  sx126x_writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sw, 2);
}

uint16_t transmitFunction(char* message, uint8_t length, uint32_t timeout) {

  Serial.println("\n-- TRANSMIT FUNCTION --");

  // Set buffer base address
  Serial.println("Mark a pointer in buffer for transmit message");
  sx126x_setBufferBaseAddress(0x00, 0x80);

  // Write the message to buffer
  uint8_t* msgUint8 = (uint8_t*) message;
  Serial.print("Write message \'");
  Serial.print(message);
  Serial.println("\' in buffer");
  Serial.print("Message in bytes : [ ");
  sx126x_writeBuffer(0x00, msgUint8, length);
  for (uint8_t i = 0; i < length; i++) {
    Serial.print((uint8_t) message[i]);
    Serial.print("  ");
  }
  Serial.println("]");

  // Set payload length same as message length
  Serial.print("Set payload length same as message length (");
  Serial.print(length);
  Serial.println(")");
  sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);

  // Activate interrupt when transmit done on DIO1
  Serial.println("Set TX done and timeout IRQ on DIO1");
  uint16_t mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
  sx126x_setDioIrqParams(mask, mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  // Attach irqPin to DIO1
  Serial.println("Attach interrupt on IRQ pin");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkTransmitDone, RISING);
  // Set txen and rxen pin state for transmitting packet
#ifdef SX126X_USING_TXEN_RXEN
  digitalWrite(txenPin, HIGH);
  digitalWrite(rxenPin, LOW);
#endif

  // Calculate timeout (timeout duration = timeout * 15.625 us)
  uint32_t tOut = timeout * 64;
  // Set RF module to TX mode to transmit message
  Serial.println("Transmitting LoRa packet");
  sx126x_setTx(tOut);
  uint32_t tStart = millis(), tTrans = 0;

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
  sx126x_getIrqStatus(&irqStat);
  Serial.println("Clear IRQ status");
  sx126x_clearIrqStatus(irqStat);
#ifdef SX126X_USING_TXEN_RXEN
  digitalWrite(txenPin, LOW);
#endif

  // return interrupt status
  return irqStat;
}

void setup() {

  // Begin serial communication
  Serial.begin(38400);

  // Settings for LoRa communication
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
    Serial.println("Transmit timeout");
  }

  // Don't load RF module with continous transmit
  delay(10000);
}
