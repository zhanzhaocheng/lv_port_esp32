/**
 * @file disp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "lvgl/lvgl.h"

#include "disp_spi.h"
#include "disp_driver.h"

#include "../lvgl_driver.h"

/*********************
 *      DEFINES
 *********************/
 #if CONFIG_LVGL_TFT_DISPLAY_SPI_HSPI == 1
 #define TFT_SPI_HOST HSPI_HOST
 #else
 #define TFT_SPI_HOST VSPI_HOST
 #endif

#define LEN_BYTES_TO_BITS(n)    (n * 8)


/**********************
 *      TYPEDEFS
 **********************/

typedef enum _spi_send_flag_t {
    SPI_SEND_QUEUED         = 0x00UL,
    SPI_SEND_POLLING        = 0x01UL,
    SPI_SEND_SYNCHRONOUS    = 0x02UL,
    SPI_SEND_SIGNAL_FLUSH   = 0x04UL
} spi_send_flag_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR spi_ready (spi_transaction_t *trans);
static void disp_spi_send_data_ex(uint8_t *data, uint16_t length, spi_send_flag_t flag);

/**********************
 *  STATIC VARIABLES
 **********************/
static spi_device_handle_t spi;
static volatile bool spi_trans_in_progress;
static volatile bool spi_color_sent;
static transaction_cb_t chained_post_cb;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void disp_spi_add_device_config(spi_host_device_t host, spi_device_interface_config_t *devcfg)
{
    chained_post_cb=devcfg->post_cb;
    devcfg->post_cb=spi_ready;
    esp_err_t ret=spi_bus_add_device(host, devcfg, &spi);
    assert(ret==ESP_OK);
}

void disp_spi_add_device(spi_host_device_t host)
{
    spi_device_interface_config_t devcfg={
#if defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ST7789
        .clock_speed_hz=24*1000*1000,           // Clock out at 24 MHz
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_HX8357
        .clock_speed_hz=26*1000*1000,           // Clock out at 26 MHz
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_SH1107
        .clock_speed_hz=8*1000*1000,            // Clock out at 8 MHz
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ILI9486
        .clock_speed_hz=24*1000*1000,           //Clock out at 24 MHz
#else
        .clock_speed_hz=40*1000*1000,           // Clock out at 40 MHz
#endif

#if defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ST7789
        .mode=2,                                // SPI mode 2
#else
	    .mode=0,				                // SPI mode 0
#endif
	    .spics_io_num=DISP_SPI_CS,              // CS pin
        .queue_size=1,
        .pre_cb=NULL,
        .post_cb=NULL,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    disp_spi_add_device_config(host, &devcfg);
}

void disp_spi_init(void)
{

    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=DISP_SPI_MOSI,
        .sclk_io_num=DISP_SPI_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
#if defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ILI9341
        .max_transfer_sz = DISP_BUF_SIZE * 2,
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ST7789
        .max_transfer_sz = DISP_BUF_SIZE * 2,
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ILI9488
        .max_transfer_sz = DISP_BUF_SIZE * 3,
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_HX8357
        .max_transfer_sz = DISP_BUF_SIZE * 2
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_ILI9486
            .max_transfer_sz = DISP_BUF_SIZE * 2,
#elif defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_SH1107
		.max_transfer_sz = DISP_BUF_SIZE * 2
#endif
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(TFT_SPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

    //Attach the LCD to the SPI bus
    disp_spi_add_device(TFT_SPI_HOST);
}

void disp_spi_send_data(uint8_t * data, uint16_t length)
{
    disp_spi_send_data_ex(data, length, SPI_SEND_QUEUED);
}

void disp_spi_send_colors(uint8_t * data, uint16_t length)
{
    disp_spi_send_data_ex(data, length, SPI_SEND_SIGNAL_FLUSH);
}


bool disp_spi_is_busy(void)
{
    return spi_trans_in_progress;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void IRAM_ATTR spi_ready (spi_transaction_t *trans)
{
    spi_send_flag_t flags = (spi_send_flag_t) trans->user;

    if (flags & SPI_SEND_SIGNAL_FLUSH) {
        lv_disp_t *disp = lv_refr_get_disp_refreshing();
        lv_disp_flush_ready(&disp->driver);
    }

    if (chained_post_cb) {
        chained_post_cb(trans);
    }
}

static void disp_spi_send_data_ex(uint8_t *data, uint16_t length,
    spi_send_flag_t flags)
{
    if (0 == length) {
        return;
    }
    
    // retrieve pending previous transaction results
    static volatile uint8_t pending = 0;
    spi_transaction_t *result = NULL;
    while (pending) {
        if (ESP_OK == spi_device_get_trans_result(spi, &result, portMAX_DELAY)) {
            pending--;
        }
    }

    spi_transaction_t t = {0};

    // If the transaction length (in bytes) is less than 4
    // we can avoid allocating memory for it.
    if (length <= 4) {
        t.length = LEN_BYTES_TO_BITS(length);
        t.flags = SPI_TRANS_USE_TXDATA;
        memcpy(t.tx_data, data, length);
    } else {
        t.length = LEN_BYTES_TO_BITS(length);
        t.tx_buffer = data;
    }

    t.user = (void *) flags;

    if (flags & SPI_SEND_POLLING) {
        spi_device_polling_transmit(spi, &t);
    } else if (flags & SPI_SEND_SYNCHRONOUS) {
        spi_device_transmit(spi, &t);
    } else {
        static spi_transaction_t queuedt;
        memcpy(&queuedt, &t, sizeof t);
        pending++;
        spi_device_queue_trans(spi, &queuedt, portMAX_DELAY);
    }

}
