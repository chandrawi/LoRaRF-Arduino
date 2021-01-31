#ifndef _SX126X_H_
#define _SX126X_H_

#include <BaseLoRa.h>
#include <SX126x_API.h>

// Status TX and RX operation
#define SX126X_STATUS_DEFAULT                         LORA_STATUS_DEFAULT
#define SX126X_STATUS_TX_WAIT                         LORA_STATUS_TX_WAIT
#define SX126X_STATUS_TX_TIMEOUT                      LORA_STATUS_TX_TIMEOUT
#define SX126X_STATUS_TX_DONE                         LORA_STATUS_TX_DONE
#define SX126X_STATUS_RX_WAIT                         LORA_STATUS_RX_WAIT
#define SX126X_STATUS_RX_CONTINUOUS_WAIT              LORA_STATUS_RX_CONTINUOUS_WAIT
#define SX126X_STATUS_RX_TIMEOUT                      LORA_STATUS_RX_TIMEOUT
#define SX126X_STATUS_RX_DONE                         LORA_STATUS_RX_DONE
#define SX126X_STATUS_HEADER_ERR                      LORA_STATUS_HEADER_ERR
#define SX126X_STATUS_CRC_ERR                         LORA_STATUS_CRC_ERR
#define SX126X_STATUS_CAD_WAIT                        LORA_STATUS_CAD_WAIT
#define SX126X_STATUS_CAD_DETECTED                    LORA_STATUS_CAD_DETECTED
#define SX126X_STATUS_CAD_DONE                        LORA_STATUS_CAD_DONE

// Default Hardware Configuration
#define SX126X_PIN_RF_IRQ                             1
#define SX126X_PIN_IRQ                                -1

#ifdef USE_LORA_SX126X
class SX126x
#else
class SX126x : public BaseLoRa
#endif
{

    public:

        SX126x();

        bool begin();
        bool begin(int8_t nss, int8_t reset, int8_t busy, int8_t irq=-1, int8_t txen=-1, int8_t rxen=-1);
        void end();
        void reset();
        void sleep(uint8_t option=SX126X_SLEEP_WARM_START);
        void wake();
        void standby(uint8_t option=SX126X_STANDBY_RC);
        void setActive();
        void setFallbackMode(uint8_t fallbackMode);
        uint8_t getMode();

        void setSPI(SPIClass &SpiObject);
        void setPins(int8_t nss, int8_t reset, int8_t busy, int8_t irq=-1, int8_t txen=-1, int8_t rxen=-1);
        void setRfIrqPin(int8_t dioPinSelect);
        void setDio2RfSwitch(bool enable=true);
        void setDio3TcxoCtrl(uint8_t tcxoVoltage, uint32_t delayTime);
        void setXtalCap(uint8_t xtalA, uint8_t xtalB);
        void setRegulator(uint8_t regMode);
        void setCurrentProtection(uint8_t level);

        void setPacketType(uint8_t packetType=SX126X_PACKET_TYPE_LORA);
        void setFrequency(uint32_t frequency);
        void setTxPower(uint32_t txPower);
        void setRxGain(uint8_t rxGain);
        void setLoRaModulation(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro=SX126X_LORA_LDRO_ON);
        void setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, uint8_t crcType=SX126X_LORA_CRC_ON, uint8_t invertIq=SX126X_LORA_IQ_STANDARD);
        void setLoRaPayloadLength(uint8_t payloadLength);
        void setLoRaSyncWord(uint16_t sw);
        void setFskModulation(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev);
        void setFskPacket(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening);
        void setFskSyncWord(uint8_t* sw, uint8_t swLen);
        void setFskAdress(uint8_t nodeAddr, uint8_t broadcastAddr);
        void setFskCrc(uint16_t crcInit, uint16_t crcPolynom);
        void setFskWhitening(uint16_t whitening);

        void beginPacket();
        void endPacket(uint32_t timeout=SX126X_TX_MODE_SINGLE);
        void write(uint8_t data);
        void write(uint8_t* data, uint8_t length);
        void write(char* data, uint8_t length);
        template <typename T> void put(T data)
        {
            const uint8_t length = sizeof(T);
            union conv{
                T Data;
                uint8_t Binary[length];
            };
            union conv u;
            u.Data = data;
            SX126x_API::writeBuffer(_bufferIndex, u.Binary, length);
            _bufferIndex += length;
            _payloadTxRx += length;
        }

        void request(uint32_t timeout=SX126X_RX_MODE_SINGLE);
        void listen(uint32_t rxPeriod, uint32_t sleepPeriod);
        uint8_t available();
        uint8_t read();
        uint8_t read(uint8_t* data, uint8_t length);
        uint8_t read(char* data, uint8_t length);
        void flush();
        template <typename T> uint8_t get(T &data)
        {
            available();
            const uint8_t length = sizeof(T);
            union conv{
                T Data;
                uint8_t Binary[length];
            };
            union conv u;
            SX126x_API::readBuffer(_bufferIndex, u.Binary, length);
            data = u.Data;
            _bufferIndex += length;
            uint8_t len = length;
            if (_payloadTxRx > length){ _payloadTxRx -= length; }
            else { _payloadTxRx = 0; len = _payloadTxRx; }
            return len;
        }

        uint8_t status();
        void wait();
        uint32_t transmitTime();
        float dataRate();
        float rssi();
        float snr();
        float signalRssi();
        float rssiInst();
        uint16_t getError();

    protected:

        SPIClass* _spi;
        int8_t _nss, _reset, _busy;
        int8_t _irq, _txen, _rxen;
        int8_t _dio;
        uint8_t _packetType;
        uint8_t _sf, _bw, _cr, _ldro;
        uint8_t _headerType, _payloadLength, _crcType, _invertIq;
        uint16_t _preambleLength;
        static uint8_t _bufferIndex;
        static uint8_t _payloadTxRx;

    private:

        uint8_t _status;
        uint8_t _statusRxContinuous;
        static uint8_t _statusInterrupt;
        static uint32_t _transmitTime;
        static int8_t _pinToLow;

        void _irqSetup(uint16_t irqMask);
        uint16_t _waitIrq();
        static void _interruptTx();
        static void _interruptRx();
        uint8_t _getStatusInterrupt();

};

#endif