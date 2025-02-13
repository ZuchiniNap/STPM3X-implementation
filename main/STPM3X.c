#include "STPM3X.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "STPM3X_define.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

#define DEBUG 1 // Set to 1 to enable debug logging, 0 to disable

static spi_device_handle_t spi;
static bool spi_initialized = false;
bool STPM3X_Init_STPM34(STPM3X *dev);

static const char *TAG = "STPM3X";

DSP_CR100bits_t DSP_CR100bits;
DSP_CR101bits_t DSP_CR101bits;
DSP_CR200bits_t DSP_CR200bits;
DSP_CR201bits_t DSP_CR201bits;
DSP_CR400bits_t DSP_CR400bits;
US1_REG100bits_t US1_REG100bits;

DFE_CR101bits_t DFE_CR101bits;
DFE_CR201bits_t DFE_CR201bits;
DSP_CR301bits_t DSP_CR301bits;
DSP_CR500bits_t DSP_CR500bits;

bool _autoLatch;
bool _crcEnabled;

void STPM3X_spiTransfer(STPM3X *dev, uint8_t *tx_data, uint8_t *rx_data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = STPM3X_FRAME_LEN * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    // Ensure SPI is initialized before using
    if (!spi_initialized)
    {
        init_spi();
        if (!spi_initialized) // If still not initialized, abort
        {
            ESP_LOGE(TAG, "SPI initialization failed. Cannot transmit.");
            return;
        }
    }

    esp_err_t ret = spi_device_transmit(spi, &t); // The pointer to `t` is correct here
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transmit failed with error: %d", ret);
    }

    ESP_LOGI(TAG, "SPI Pratik TX: 0x%02X 0x%02X 0x%02X 0x%02X", tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
}

void init_spi()
{
    if (spi_initialized)
    {
        ESP_LOGI(TAG, "SPI already initialized, skipping...");
        return;
    }
    ESP_LOGI(TAG, "Initializing SPI...");

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = 256,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, 
        .mode = 3,
        .spics_io_num = 5,
        .queue_size = 7,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1
        
    };

    ESP_LOGI(TAG, "Initializing SPI bus...");
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus! Error: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "Adding SPI device...");
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device! Error: %d", ret);
        return;
    }

    // Mark SPI as initialized only if everything is successful
    spi_initialized = true;
    ESP_LOGI(TAG, "SPI initialized successfully");
}

// Initialize the STPM3X device
bool STPM3X_init(STPM3X *dev, int reset_pin, int cs_pin, int syn_pin)
{
    esp_err_t ret;

    // Store pin assignments in the device struct
    dev->reset_pin = reset_pin;
    dev->cs_pin = cs_pin;
    dev->syn_pin = syn_pin;
    // dev->auto_latch = false;
    // dev->crc_enabled = true;

    // Configure GPIOs for RESET, CS, and SYN
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << reset_pin) | (1ULL << cs_pin) | (syn_pin != -1 ? (1ULL << syn_pin) : 0),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)cs_pin, 0);
    gpio_set_level((gpio_num_t)reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(35));
    gpio_set_level((gpio_num_t)reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(35));
    gpio_set_level((gpio_num_t)cs_pin, 1);

    // Toggle SYN pin 3 times if available
    if (syn_pin != -1)
    {
        for (size_t i = 0; i < 3; i++)
        {
            gpio_set_level((gpio_num_t)syn_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(2));
            gpio_set_level((gpio_num_t)syn_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level((gpio_num_t)cs_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level((gpio_num_t)cs_pin, 1);

    ESP_LOGI("STPM3X", "Initializing STPM3X...");
    STPM3X_autoLatch(dev, true);

    // Ensure SPI is initialized before proceeding
    init_spi();
    if (!spi_initialized)
    {
        ESP_LOGE("STPM3X", "SPI initialization failed!");
        return false;
    }

    dev->spi = spi; // Assign the initialized SPI handle to the device struct

    ESP_LOGI("STPM3X", "STPM3X Reset and SYN toggle completed.");

    // Initialize STPM3X registers
    if (!STPM3X_Init_STPM34(dev))
    {
        ESP_LOGE("STPM3X", "STPM3X Initialization Failed!");
        return false;
    }

    ESP_LOGI("STPM3X", "STPM3X Initialization Completed Successfully.");
    return true;
}

// STPM34 Configuration
bool STPM3X_Init_STPM34(STPM3X *dev)
{
    ESP_LOGI(TAG, "Starting STPM34 Initialization...");

    uint8_t readAdd, writeAdd, dataLSB, dataMSB;

    // Configure DSP Control Register 1 LSW
    DSP_CR100bits.ENVREF1 = 1; // enable internal Vref1 bit for CH0;
    DSP_CR100bits.TC1 = 0x02;  // set temperature compensation for CH0; Vref1=1.18v default
    readAdd = 0x00;
    writeAdd = 0x00;
    dataLSB = DSP_CR100bits.LSB;
    dataMSB = DSP_CR100bits.MSB;
    STPM3X_sendFrameCRC(dev, 0x00, 0x00, dataLSB, dataMSB);
    ESP_LOGI(TAG, "Configured DSP Control Register 1 LSW");

    // Configure DSP Control Register 1 MSW
    DSP_CR101bits.BHPFV1 = 0;  // HPF enable voltage;DC cancellation
    DSP_CR101bits.BHPFC1 = 0;  // HPF enable current;DC cancellation
    DSP_CR101bits.BLPFV1 = 1;  // LPF wideband voltage;set up fundamental mode
    DSP_CR101bits.BLPFC1 = 1;  // LPF wideband current;set up fundamental mode
    DSP_CR101bits.LPW1 = 0x04; // LED Division factor
    readAdd = 0x00;
    writeAdd = 0x01;
    dataLSB = DSP_CR101bits.LSB;
    dataMSB = DSP_CR101bits.MSB;
    STPM3X_sendFrameCRC(dev, readAdd, writeAdd, dataLSB, dataMSB);
    ESP_LOGI(TAG, "Configured DSP Control Register 1 MSW");

    // Configure DSP Control Register 2 LSW
    DSP_CR200bits.ENVREF2 = 1; // enable internal Vref1 bit for CH1;
    DSP_CR200bits.TC2 = 0x02;  // set temperature compensation for CH1;  Vref2=1.18v default
    readAdd = 0x01;
    writeAdd = 0x02;
    dataLSB = DSP_CR200bits.LSB;
    dataMSB = DSP_CR200bits.MSB;
    STPM3X_sendFrameCRC(dev, readAdd, writeAdd, dataLSB, dataMSB);
    ESP_LOGI(TAG, "Configured DSP Control Register 2 LSW");

    // Configure DSP Control Register 2 MSW
    DSP_CR201bits.BHPFV2 = 0;  // 1;//0;        //HPF enable voltage;DC cancellation
    DSP_CR201bits.BHPFC2 = 0;  // 1;//0;        //HPF enable current;DC cancellation
    DSP_CR201bits.BLPFV2 = 1;  // LPF bypassed -  wideband voltage;set up fundamental mode
    DSP_CR201bits.BLPFC2 = 1;  // LPF bypassed -  wideband current;set up fundamental mode
    DSP_CR201bits.LPW2 = 0x04; // LED Division factor
    readAdd = 0x02;
    writeAdd = 0x03;
    dataLSB = DSP_CR201bits.LSB;
    dataMSB = DSP_CR201bits.MSB;
    STPM3X_sendFrameCRC(dev, readAdd, writeAdd, dataLSB, dataMSB);
    ESP_LOGI(TAG, "Configured DSP Control Register 2 MSW");

    // Set current gain
    STPM3X_setCurrentGain(dev, 1, GAIN_8X);
    STPM3X_setCurrentGain(dev, 2, GAIN_8X);
    ESP_LOGI(TAG, "Set current gain for both channels");

    // Verify if STPM is running
    uint8_t readBuffer[4] = {0};
    STPM3X_readFrame(dev, 0x18, readBuffer);
    bool success = STPM3X_checkGain(dev, 1, readBuffer);
    ESP_LOGI(TAG, "Checked gain for channel 1: %s", success ? "Success" : "Fail");

    STPM3X_readFrame(dev, 0x1A, readBuffer);
    success &= STPM3X_checkGain(dev, 2, readBuffer);
    ESP_LOGI(TAG, "Checked gain for channel 2: %s", success ? "Success" : "Fail");

    // Enable auto latch and disable CRC
    STPM3X_autoLatch(dev, false);
    STPM3X_CRC(dev, false);
    ESP_LOGI(TAG, "Enabled auto latch and disabled CRC");

    ESP_LOGI(TAG, "STPM34 Initialization Completed: %s", success ? "Success" : "Fail");
    return success;
}

// Set calibration factors
void STPM3X_setCalibration(STPM3X *dev, float calV, float calI)
{
    for (int i = 0; i < 3; i++)
    {
        dev->calibration[i][0] = calV;
        dev->calibration[i][1] = calI;
    }
}

// Read voltage and current
void STPM3X_readAll(STPM3X *dev, uint8_t channel, float *voltage, float *current, float *active, float *reactive)
{
    uint8_t tx_data[5] = {channel == 1 ? V1_Data_Address : V2_Data_Address, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_data[5] = {0};

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);
    STPM3X_spiTransfer(dev, tx_data, rx_data);
    gpio_set_level((gpio_num_t)dev->cs_pin, 1);

    *voltage = ((rx_data[1] << 8 | rx_data[2]) * 0.000138681) * dev->calibration[channel][_V];
    *current = ((rx_data[3] << 8 | rx_data[4]) * 0.003349213) * dev->calibration[channel][_I];
}

// Read voltage
float STPM3X_readVoltage(STPM3X *dev, uint8_t channel)
{
    uint8_t tx_data[5] = {channel == 1 ? V1_Data_Address : V2_Data_Address, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_data[5] = {0};

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);
    STPM3X_spiTransfer(dev, tx_data, rx_data);
    gpio_set_level((gpio_num_t)dev->cs_pin, 1);

    return ((rx_data[1] << 8 | rx_data[2]) * 0.000138681) * dev->calibration[channel][_V];
}

// Read current
float STPM3X_readCurrent(STPM3X *dev, uint8_t channel)
{
    uint8_t tx_data[5] = {channel == 1 ? C1_Data_Address : C2_Data_Address, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_data[5] = {0};

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);
    STPM3X_spiTransfer(dev, tx_data, rx_data);
    gpio_set_level((gpio_num_t)dev->cs_pin, 1);

    return ((rx_data[1] << 8 | rx_data[2]) * 0.003349213) * dev->calibration[channel][_I];
}

float STPM3X_readRMSCurrent(STPM3X *dev, uint8_t channel)
{

    uint8_t address;
    uint8_t readBuffer[4] = {0}; // Buffer to store the read data

    // Determine the address based on the channel
    if (channel == 1)
    {
        address = C1_RMS_Data_Address;
    }
    else if (channel == 2)
    {
        address = C2_RMS_Data_Address;
    }
    else
    {
        ESP_LOGI(TAG, "Info: Channel %d out of range", channel);
        return -1;
    }

    // Ensure auto latch is enabled, if required
    if (!dev->auto_latch)
    {
        STPM3X_autoLatch(dev, true);
    }

    // Read frame from the given address
    STPM3X_readFrame(dev, address, readBuffer);

    // Debug: Print the raw data for troubleshooting
    ESP_LOGI(TAG, "Read Buffer: ");
    for (int i = 0; i < sizeof(readBuffer); i++)
    {
        ESP_LOGI(TAG, "0x%02X ", readBuffer[i]);
    }

    // Calculate the RMS current using the calibration factor
    return calcCurrent((int16_t)buffer15to32(readBuffer)) * dev->calibration[channel][_I];
}

inline float calcCurrent(int16_t raw_value)
{
    return raw_value * 0.2143;
}

inline int32_t buffer15to32(uint8_t *buffer)
{
    // Step 1: Combine bytes into a 32-bit value
    uint32_t combinedValue = 0;
    combinedValue |= ((uint32_t)buffer[3] << 16); // Shift the byte at index 3 left by 16 bits
    combinedValue |= ((uint32_t)buffer[2] << 8);  // Shift the byte at index 2 left by 8 bits
    combinedValue |= buffer[1];                   // Combine byte at index 1

    // Step 2: Shift the combined value to the right by 7 bits
    uint32_t result = combinedValue >> 7;

    return result;
    // return ((((buffer[3] << 16) | (buffer[2] << 8)) | buffer[1]) >> 7);
}
// Read active power
float STPM3X_readActivePower(STPM3X *dev, uint8_t channel)
{
    uint8_t tx_data[5] = {channel == 1 ? PH1_Active_Power_Address : PH2_Active_Power_Address, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_data[5] = {0};

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);
    STPM3X_spiTransfer(dev, tx_data, rx_data);
    gpio_set_level((gpio_num_t)dev->cs_pin, 1);

    return ((rx_data[1] << 8 | rx_data[2]) * 0.001) * dev->calibration[channel][_V] * dev->calibration[channel][_I];
}

// Read reactive power
float STPM3X_readReactivePower(STPM3X *dev, uint8_t channel)
{
    uint8_t tx_data[5] = {channel == 1 ? PH1_Active_Power_Address : PH2_Active_Power_Address, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_data[5] = {0};

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);
    STPM3X_spiTransfer(dev, tx_data, rx_data);
    gpio_set_level((gpio_num_t)dev->cs_pin, 1);

    return ((rx_data[1] << 8 | rx_data[2]) * 0.001) * dev->calibration[channel][_V] * dev->calibration[channel][_I];
}

// Set current gain
void STPM3X_setCurrentGain(STPM3X *dev, uint8_t channel, Gain gain)
{
    if (channel != 1 && channel != 2)
    {
        ESP_LOGW(TAG, "Channel %d out of range", channel);
        return;
    }

    uint8_t readAddr, writeAddr, dataLSB, dataMSB;
    uint8_t readBuffer[4] = {0};

    if (channel == 1)
    {
        readAddr = 0x18;  // Read address for channel 1 gain
        writeAddr = 0x19; // Write address for channel 1 gain
        DFE_CR101bits.LSB = readBuffer[2];
        DFE_CR101bits.MSB = readBuffer[3];
        DFE_CR101bits.GAIN1 = gain;
        dev->_gain1 = gain;
        dataLSB = DFE_CR101bits.LSB;
        dataMSB = DFE_CR101bits.MSB;
    }
    else
    {
        readAddr = 0x1A;  // Read address for channel 2 gain
        writeAddr = 0x1B; // Write address for channel 2 gain
        DFE_CR201bits.LSB = readBuffer[0];
        DFE_CR201bits.MSB = readBuffer[1];
        DFE_CR201bits.GAIN1 = gain;
        dev->_gain2 = gain;
        dataLSB = DFE_CR201bits.LSB;
        dataMSB = DFE_CR201bits.MSB;
    }

    // Write back the modified gain settings
    STPM3X_sendFrameCRC(dev, 0x00, writeAddr, dataLSB, dataMSB);
    ESP_LOGI(TAG, "Set gain for channel %d to %d", channel, gain);

    STPM3X_readFrame(dev, readAddr, readBuffer);
    ESP_LOGI(TAG, "Read Frame for channel %d: 0x%02X 0x%02X 0x%02X 0x%02X", channel, readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);
}

void STPM3X_autoLatch(STPM3X *dev, bool enabled)
{
    dev->auto_latch = enabled;
    if (_autoLatch)
    {
        DSP_CR301bits.SWAuto_Latch = 1; // Automatic measurement register latch at 7.8125 kHz
        DSP_CR301bits.SW_Latch1 = 0;
        DSP_CR301bits.SW_Latch2 = 0;
    }
    else
    {
        DSP_CR301bits.SWAuto_Latch = 0; // Automatic measurement register latch at 7.8125 kHz
        DSP_CR301bits.SW_Latch1 = 1;
        DSP_CR301bits.SW_Latch2 = 1;
    }
    if (_crcEnabled)
        STPM3X_sendFrameCRC(dev, 0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
    else
        STPM3X_sendFrameCRC(dev, 0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
}

void STPM3X_CRC(STPM3X *dev, bool enabled)
{
    if (_crcEnabled == enabled)
        return;
    // Disable CRC
    if (!enabled)
    {
        ESP_LOGI(TAG, "Info:Disable CRC");
        US1_REG100bits.CRC_EN = 0; // disable CRC polynomial
        STPM3X_sendFrameCRC(dev, 0x24, 0x24, US1_REG100bits.LSB, US1_REG100bits.MSB);
        // Enable CRC
    }
    else
    {
        ESP_LOGI(TAG, "Info:Enable CRC");
        US1_REG100bits.CRC_EN = 1; // disable CRC polynomial
        STPM3X_sendFrame(dev, 0x24, 0x24, US1_REG100bits.LSB, US1_REG100bits.MSB);
    }
    _crcEnabled = enabled;
}

bool STPM3X_checkGain(STPM3X *dev, uint8_t channel, uint8_t *buffer)
{
    // Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
    if (channel != 1 && channel != 2)
    {
        ESP_LOGI(TAG, "Info:checkCurrentGain: Channel %d out of range", channel);
        return false;
    }

    if (channel == 1)
    {
        if (DFE_CR101bits.LSB != buffer[2])
            return false;
        if (DFE_CR101bits.MSB != buffer[3])
            return false;
        if (DFE_CR101bits.GAIN1 != dev->_gain1)
            return false;
    }
    else
    {
        if (DFE_CR201bits.LSB != buffer[2])
            return false;
        if (DFE_CR201bits.MSB != buffer[3])
            return false;
        if (DFE_CR201bits.GAIN1 != dev->_gain2)
            return false;
    }
    return true;
}

uint8_t CRC_u8Checksum;

void STPM3X_readFrame(STPM3X *dev, uint8_t address, uint8_t *buffer)
{
    uint8_t tx_data[4];

    // Check if CRC is enabled and send appropriate frame
    if (_crcEnabled)
    {
        tx_data[0] = address; // Set the address
        tx_data[1] = 0xff;
        tx_data[2] = 0xff;
        tx_data[3] = 0xff; // Prepare the frame with CRC
                           //   STPM3X_spiTransfer(dev, tx_data, buffer);
    }
    else
    {
        tx_data[0] = address; // Set the address
        tx_data[1] = 0xff;
        tx_data[2] = 0xff;
        tx_data[3] = 0xff; // Prepare the frame without CRC
                           //   STPM3X_spiTransfer(dev, tx_data, buffer);
    }

    // Chip Select low
    gpio_set_level((gpio_num_t)dev->cs_pin, 0);
    // Perform the SPI transfer
    STPM3X_spiTransfer(dev, tx_data, buffer);
    // Chip Select high
    gpio_set_level((gpio_num_t)dev->cs_pin, 1);
    // Delay to ensure data is received
    vTaskDelay(pdMS_TO_TICKS(4));
}

// void STPM3X_readFrame(STPM3X *dev,uint8_t address, uint8_t *buffer) {
//     uint8_t tx_data[4] = {address, 0xFF, 0xFF, 0xFF};
//     spi_transaction_t t = {
//         .length = 8 * 4,
//         .tx_buffer = tx_data,
//         .rx_buffer = buffer,
//     };

//     // Pull CS low before transaction
//     gpio_set_level(5, 0);
//     esp_err_t ret = spi_device_transmit(spi, &t);
//     gpio_set_level(5, 1);

//     if (ret != ESP_OK) {
//         printf("SPI Read Error: %d\n", ret);
//     }
// }

void STPM3X_sendFrame(STPM3X *dev, uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB)
{

    uint8_t frame[STPM3x_FRAME_LEN];
    frame[0] = readAdd;
    frame[1] = writeAdd;
    frame[2] = dataLSB;
    frame[3] = dataMSB;

    ESP_LOGI(TAG, "Sending Frame: 0x%02X 0x%02X 0x%02X 0x%02X", frame[0], frame[1], frame[2], frame[3]);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = STPM3x_FRAME_LEN * 8; // Length in bits
    t.tx_buffer = frame;
    t.rx_buffer = NULL; // No need to receive data

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);        // Set CS low to start communication
    esp_err_t ret = spi_device_transmit(dev->spi, &t); // Use dev->spi instead of spi_handle
    if (ret != ESP_OK)
    {
        ESP_LOGE("STPM", "SPI transmit failed");
    }
    gpio_set_level((gpio_num_t)dev->cs_pin, 1); // Set CS high to end communication
}

// void STPM3X_sendFrame(STPM3X *dev, uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB)
// {
//     uint8_t frame[STPM3x_FRAME_LEN] = {readAdd, writeAdd, dataLSB, dataMSB};

//     ESP_LOGI(TAG, "Sending Frame: 0x%02X 0x%02X 0x%02X 0x%02X", frame[0], frame[1], frame[2], frame[3]);

//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = STPM3x_FRAME_LEN * 8; // Length in bits
//     t.tx_buffer = frame;
//     t.rx_buffer = NULL; // No need to receive data

//     // Start SPI transaction
//     gpio_set_level((gpio_num_t)dev->cs_pin, 0);
//     vTaskDelay(1); // Small delay for stability (similar to Arduino code)

//     esp_err_t ret = spi_device_transmit(dev->spi, &t);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
//     }

//     vTaskDelay(1); // Delay before CS high
//     gpio_set_level((gpio_num_t)dev->cs_pin, 1); // End SPI transaction
// }
void STPM3X_sendFrameCRC(STPM3X *dev, uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB)
{
    uint8_t frame[STPM3x_FRAME_LEN];
    frame[0] = readAdd;
    frame[1] = writeAdd;
    frame[2] = dataLSB;
    frame[3] = dataMSB;
    frame[4] = CalcCRC8(frame);

    ESP_LOGI(TAG, "Sending Frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", frame[0], frame[1], frame[2], frame[3], frame[4]);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * STPM3x_FRAME_LEN; // Send one byte at a time
    t.tx_buffer = frame;
    t.rx_buffer = NULL;

    gpio_set_level((gpio_num_t)dev->cs_pin, 0);        // CS LOW
    esp_err_t ret = spi_device_transmit(dev->spi, &t); // Transmit the frame
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
    }

    gpio_set_level((gpio_num_t)dev->cs_pin, 1); // CS HIGH
}

// void printRegister(uint8_t *frame, const char *regName)
// {
//     ESP_LOGI("STPM", "%s", regName);
//     for (int i = 0; i < STPM3x_FRAME_LEN; i++)
//     {
//         ESP_LOGI("STPM", "0x%02X ", frame[i]);
//     }
//     ESP_LOGI("STPM", "");
// }

// CRC8 calculation function (Implement this function based on your algorithm)
uint8_t CalcCRC8(uint8_t *pBuf)
{
    uint8_t i;
    CRC_u8Checksum = 0x00;
    for (i = 0; i < STPM3x_FRAME_LEN - 1; i++)
    {
        Crc8Calc(pBuf[i]);
    }
    return CRC_u8Checksum;
}

void Crc8Calc(uint8_t u8Data)
{
    uint8_t loc_u8Idx;
    uint8_t loc_u8Temp;
    loc_u8Idx = 0;
    while (loc_u8Idx < 8)
    {
        loc_u8Temp = u8Data ^ CRC_u8Checksum;
        CRC_u8Checksum <<= 1;
        if (loc_u8Temp & 0x80)
        {
            CRC_u8Checksum ^= CRC_8;
        }
        u8Data <<= 1;
        loc_u8Idx++;
    }
}