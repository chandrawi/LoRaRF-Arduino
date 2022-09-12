#ifndef _SX127X_DRIVER_H_
#define _SX127X_DRIVER_H_

#include <Arduino.h>
#include <SPI.h>

// SX127X register map
#define SX127X_REG_FIFO                         0x00
#define SX127X_REG_OP_MODE                      0x01
#define SX127X_REG_FRF_MSB                      0x06
#define SX127X_REG_FRF_MID                      0x07
#define SX127X_REG_FRF_LSB                      0x08
#define SX127X_REG_PA_CONFIG                    0x09
#define SX127X_REG_OCP                          0x0B
#define SX127X_REG_LNA                          0x0C
#define SX127X_REG_FIFO_ADDR_PTR                0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR            0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR            0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR         0x10
#define SX127X_REG_IRQ_FLAGS                    0x12
#define SX127X_REG_RX_NB_BYTES                  0x13
#define SX127X_REG_PKT_SNR_VALUE                0x19
#define SX127X_REG_PKT_RSSI_VALUE               0x1A
#define SX127X_REG_RSSI_VALUE                   0x1B
#define SX127X_REG_MODEM_CONFIG_1               0x1D
#define SX127X_REG_MODEM_CONFIG_2               0x1E
#define SX127X_REG_SYMB_TIMEOUT                 0x1F
#define SX127X_REG_PREAMBLE_MSB                 0x20
#define SX127X_REG_PREAMBLE_LSB                 0x21
#define SX127X_REG_PAYLOAD_LENGTH               0x22
#define SX127X_REG_MODEM_CONFIG_3               0x26
#define SX127X_REG_FREQ_ERROR_MSB               0x28
#define SX127X_REG_FREQ_ERROR_MID               0x29
#define SX127X_REG_FREQ_ERROR_LSB               0x2A
#define SX127X_REG_RSSI_WIDEBAND                0x2C
#define SX127X_REG_DETECTION_OPTIMIZE           0x31
#define SX127X_REG_INVERTIQ                     0x33
#define SX127X_REG_DETECTION_THRESHOLD          0x37
#define SX127X_REG_SYNC_WORD                    0x39
#define SX127X_REG_INVERTIQ2                    0x3B
#define SX127X_REG_DIO_MAPPING_1                0x40
#define SX127X_REG_VERSION                      0x42
#define SX127X_REG_TCXO                         0x4B
#define SX127X_REG_PA_DAC                       0x4D

// Modem options
#define SX127X_FSK_MODEM                        0x00        // GFSK packet type
#define SX127X_LORA_MODEM                       0x01        // LoRa packet type
#define SX127X_OOK_MODEM                        0x02        // OOK packet type

// Long range mode and Modulation type
#define SX127X_LONG_RANGE_MODE                  0x80        // GFSK packet type
#define SX127X_MODULATION_OOK                   0x20        // OOK packet type
#define SX127X_MODULATION_FSK                   0x00        // LoRa packet type

// Devices modes
#define SX127X_MODE_SLEEP                       0x00        // sleep
#define SX127X_MODE_STDBY                       0x01        // standby
#define SX127X_MODE_TX                          0x03        // transmit
#define SX127X_MODE_RX_CONTINUOUS               0x05        // continuous receive
#define SX127X_MODE_RX_SINGLE                   0x06        // single receive
#define SX127X_MODE_CAD                         0x07        // channel activity detection (CAD)

// Rx operation mode
#define SX127X_RX_SINGLE                        0x000000    // Rx timeout duration: no timeout (Rx single mode)
#define SX127X_RX_CONTINUOUS                    0xFFFFFF    //                      infinite (Rx continuous mode)

// TX power options
#define SX127X_TX_POWER_RFO                     0x00        // output power is limited to +14 dBm
#define SX127X_TX_POWER_PA_BOOST                0x80        // output power is limited to +20 dBm

// RX gain options
#define SX127X_RX_GAIN_POWER_SAVING             0x00        // gain used in Rx mode: power saving gain (default)
#define SX127X_RX_GAIN_BOOSTED                  0x01        //                       boosted gain
#define SX127X_RX_GAIN_AUTO                     0x00        // option enable auto gain controller (AGC)

// Header type
#define SX127X_HEADER_EXPLICIT                  0x00        // explicit header mode
#define SX127X_HEADER_IMPLICIT                  0x01        // implicit header mode

// LoRa syncword
#define SX127X_SYNCWORD_LORAWAN                 0x34        // reserved LoRaWAN syncword

// Oscillator options
#define SX127X_OSC_CRYSTAL                      0x00        // crystal oscillator with external crystal
#define SX127X_OSC_TCXO                         0x10        // external clipped sine TCXO AC-connected to XTA pin

// DIO mapping
#define SX127X_DIO0_RX_DONE                     0x00        // set DIO0 interrupt for: RX done
#define SX127X_DIO0_TX_DONE                     0x40        //                         TX done
#define SX127X_DIO0_CAD_DONE                    0x80        //                         CAD done

// IRQ flags
#define SX127X_IRQ_CAD_DETECTED                 0x01        // Valid Lora signal detected during CAD operation
#define SX127X_IRQ_FHSS_CHANGE                  0x02        // FHSS change channel interrupt
#define SX127X_IRQ_CAD_DONE                     0x04        // channel activity detection finished
#define SX127X_IRQ_TX_DONE                      0x08        // packet transmission completed
#define SX127X_IRQ_HEADER_VALID                 0x10        // valid LoRa header received
#define SX127X_IRQ_CRC_ERR                      0x20        // wrong CRC received
#define SX127X_IRQ_RX_DONE                      0x40        // packet received
#define SX127X_IRQ_RX_TIMEOUT                   0x80        // waiting packet received timeout

// Rssi offset
#define SX127X_RSSI_OFFSET_LF                   164         // low band frequency RSSI offset
#define SX127X_RSSI_OFFSET_HF                   157         // high band frequency RSSI offset
#define SX1272_RSSI_OFFSET                      139         // frequency RSSI offset for SX1272
#define SX127X_BAND_THRESHOLD                   525E6       // threshold between low and high band frequency

// TX and RX operation status
#define SX127X_STATUS_DEFAULT                   LORA_STATUS_DEFAULT
#define SX127X_STATUS_TX_WAIT                   LORA_STATUS_TX_WAIT
#define SX127X_STATUS_TX_TIMEOUT                LORA_STATUS_TX_TIMEOUT
#define SX127X_STATUS_TX_DONE                   LORA_STATUS_TX_DONE
#define SX127X_STATUS_RX_WAIT                   LORA_STATUS_RX_WAIT
#define SX127X_STATUS_RX_CONTINUOUS             LORA_STATUS_RX_CONTINUOUS
#define SX127X_STATUS_RX_TIMEOUT                LORA_STATUS_RX_TIMEOUT
#define SX127X_STATUS_RX_DONE                   LORA_STATUS_RX_DONE
#define SX127X_STATUS_HEADER_ERR                LORA_STATUS_HEADER_ERR
#define SX127X_STATUS_CRC_ERR                   LORA_STATUS_CRC_ERR
#define SX127X_STATUS_CAD_WAIT                  LORA_STATUS_CAD_WAIT
#define SX127X_STATUS_CAD_DETECTED              LORA_STATUS_CAD_DETECTED
#define SX127X_STATUS_CAD_DONE                  LORA_STATUS_CAD_DONE

// Default Hardware Configuration
#define SX127X_PIN_NSS                          10
#define SX127X_PIN_RESET                        4
#define SX127X_SPI                              SPI
#ifdef __AVR__
    #define SX127X_SPI_FREQUENCY                F_CPU / 2   // SPI speed set to half of CPU frequency for AVR processor
#else
    #define SX127X_SPI_FREQUENCY                16000000    // Maximum LoRa SPI frequency
#endif

void sx127x_setSPI(SPIClass &SpiObject, uint32_t frequency);
void sx127x_setPins(int8_t nss);
void sx127x_reset(int8_t reset);
void sx127x_begin();

// SX126x driver: Register access functions
void sx127x_writeBits(uint8_t address, uint8_t data, uint8_t position, uint8_t length);
void sx127x_writeRegister(uint8_t address, uint8_t data);
uint8_t sx127x_readRegister(uint8_t address);
uint8_t sx127x_transfer(uint8_t address, uint8_t data);

#endif