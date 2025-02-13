#ifndef STPM3X_H
#define STPM3X_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "STPM3X_define.h"

#define STPM3X_FRAME_LEN 5
#define CRC_8 (0x07)

#define _V 0
#define _I 1

typedef enum
{
    GAIN_2X = 0x00,
    GAIN_4X = 0x01,
    GAIN_8X = 0x02,
    GAIN_16X = 0x03
} Gain;

typedef struct
{
    spi_device_handle_t spi;
    int reset_pin;
    int cs_pin;
    int syn_pin;
    bool auto_latch;
    bool crc_enabled;
    float calibration[3][2];
    uint8_t _gain1; // Gain for channel 1
    uint8_t _gain2; // Gain for channel 2
} STPM3X;
void init_spi();
void STPM3X_spiTransfer(STPM3X *dev, uint8_t *tx_data, uint8_t *rx_data);
bool STPM3X_init(STPM3X *dev, int reset_pin, int cs_pin, int syn_pin);
void STPM3X_setCalibration(STPM3X *dev, float calV, float calI);
void STPM3X_setCurrentGain(STPM3X *dev, uint8_t channel, Gain gain);
uint8_t CalcCRC8(uint8_t *pBuf);
// STPM3X.h
bool STPM3X_checkGain(STPM3X *dev, uint8_t channel, uint8_t *buffer);
void STPM3X_readAll(STPM3X *dev, uint8_t channel, float *voltage, float *current, float *active, float *reactive);
float STPM3X_readVoltage(STPM3X *dev, uint8_t channel);
float STPM3X_readCurrent(STPM3X *dev, uint8_t channel);
float STPM3X_readActivePower(STPM3X *dev, uint8_t channel);
float STPM3X_readReactivePower(STPM3X *dev, uint8_t channel);
void STPM3X_sendFrameCRC(STPM3X *dev, uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB);
void STPM3X_sendFrame(STPM3X *dev, uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB);
void STPM3X_readFrame(STPM3X *dev, uint8_t address, uint8_t *rx_data);
void STPM3X_spiTransfer(STPM3X *dev, uint8_t *tx_data, uint8_t *rx_data);
void STPM3X_autoLatch(STPM3X *dev, bool enabled);
void STPM3X_CRC(STPM3X *dev, bool enabled);
inline float calcCurrent(int16_t value);
float STPM3X_readRMSCurrent(STPM3X *dev, uint8_t channel);
inline int32_t buffer15to32(uint8_t *buffer);
void Crc8Calc(uint8_t u8Data);

//uint8_t CRC_u8Checksum;

#endif
