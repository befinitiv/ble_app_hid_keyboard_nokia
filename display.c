#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "nrf_error.h"
#include "nrf_delay.h"


#define SPI_SCK_PIN 5u
#define SPI_MOSI_PIN 6u
#define SPI_CS_PIN 7u
#define DISP_DC_PIN 8u
#define DISP_RST_PIN 9u



uint8_t display_buffer[504];

static void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
    switch (spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
            
            break;
        
        default:
            //No implementation needed.
            break;
    }
}


static void spi_master_init()
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure SPI master.
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
   	spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K125; 
            spi_config.SPI_Pin_SCK = SPI_SCK_PIN;
            spi_config.SPI_Pin_MISO = 25;
            spi_config.SPI_Pin_MOSI = SPI_MOSI_PIN;
            spi_config.SPI_Pin_SS = SPI_CS_PIN;
    
    spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
    
    err_code = spi_master_open(SPI_MASTER_0, &spi_config);
    APP_ERROR_CHECK(err_code);

		spi_master_evt_handler_reg(SPI_MASTER_0, spi_master_event_handler);
}

void display_send_init_sequence() {
	nrf_gpio_pin_clear(DISP_DC_PIN);
	uint8_t init_seq[] = {0x21, 0x14, 0xB0, 0x20, 0x0c, 128, 64};
	memcpy(display_buffer, init_seq, sizeof(init_seq));
	spi_master_send_recv(SPI_MASTER_0, display_buffer, sizeof(init_seq), NULL, 0);
}

void display_init() {
	nrf_gpio_cfg_output(DISP_DC_PIN);
	nrf_gpio_cfg_output(DISP_RST_PIN);
	nrf_gpio_pin_clear(DISP_DC_PIN);

	nrf_gpio_pin_clear(DISP_RST_PIN);
	nrf_delay_ms(10);
	nrf_gpio_pin_set(DISP_RST_PIN);


	spi_master_init(0);

	display_send_init_sequence();
}

void display_write(uint8_t d) {
	nrf_gpio_pin_set(DISP_DC_PIN);

	uint8_t img[]={ 127, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 128, 128, 128, 128, 0, 0, 0, 0, 0, 128, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 127, 127, 127, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 128, 128, 128, 193, 127, 62, 0, 0, 129, 192, 224, 176, 152, 143, 135, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 127, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 127, 2, 60, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 0, 0, 0, 0, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 60, 38, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 63, 3, 12, 12, 3, 63, 63, 0, 28, 62, 42, 42, 46, 44, 0, 62, 62, 2, 2, 62, 60, 0, 30, 62, 32, 32, 62, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	memcpy(display_buffer, img, sizeof(display_buffer));
	spi_master_send_recv(SPI_MASTER_0, display_buffer, sizeof(display_buffer), NULL, 0);
	
	//nrf_gpio_pin_set(DISP_DC_PIN);
	//display_buffer[0] = d;
	//spi_master_send_recv(SPI_MASTER_0, display_buffer, 1, NULL, 0);

}

