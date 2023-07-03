#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "logic_analyzer_ws_server.h"

#define LCD_HOST HSPI_HOST

#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 18
#define PIN_NUM_CLK 19
#define PIN_NUM_CS 21

#define BIT_PIX (257)
#define NUM_STR (32)
#define NUM_BITS (BIT_PIX * NUM_STR)
#define NUM_BYTES ((NUM_BITS / 8) + 1)
static uint8_t data[NUM_BYTES] = {0};

void fill_samples()
{
    for (int bit_i = 0, byte_i = 0, bit_mask = 1; bit_i < NUM_BITS; bit_i++)
    {
        if (bit_i % BIT_PIX == BIT_PIX - 1)
        {
            data[byte_i] |= bit_mask;
        }
        bit_mask <<= 1;
        if (bit_mask == 0x100)
        {
            bit_mask = 1;
            byte_i++;
        }
    }
}

void spi_257_32(void *p)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10000};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // Clock out at 1 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = PIN_NUM_CS,        // CS pin
        .queue_size = 1,                   // We want to be able to queue 7 transactions at a time
        .flags = SPI_DEVICE_TXBIT_LSBFIRST,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = NUM_BITS;      // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;       // Data

    fill_samples();

    while (1)
    {
        ret = spi_device_transmit(spi, &t); // Transmit!
        ESP_ERROR_CHECK(ret);               // Should have had no issues.
    }

}

void app_main(void)
{
    logic_analyzer_ws_server(); // internal Logic Analyzer // remove
    xTaskCreatePinnedToCore(spi_257_32,"spi257",4096,NULL,5,NULL,1);
}