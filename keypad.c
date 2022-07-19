#include "nordic_common.h"
#include "nrf_gpio.h"
#include "app_gpiote.h"
#include "nrf_delay.h"


#include "keypad.h"


#define R0_PIN 10
#define R1_PIN 12
#define R2_PIN 11
#define R3_PIN 14
#define R4_PIN 13
#define C1_PIN 16
#define C2_PIN 15
#define C3_PIN 20
#define C4_PIN 18


static app_gpiote_user_id_t           m_gpiote_user_id;


uint32_t check_row(uint32_t col_pin) {
	nrf_gpio_cfg_output(col_pin);
	nrf_gpio_pin_set(col_pin);


	uint32_t row_val = 0;
	if(nrf_gpio_pin_read(R0_PIN)) {
		row_val = 0;
	}
	if(nrf_gpio_pin_read(R1_PIN)) {
		row_val = 1;
	}
	if(nrf_gpio_pin_read(R2_PIN)) {
		row_val = 2;
	}
	if(nrf_gpio_pin_read(R3_PIN)) {
		row_val = 3;
	}
	if(nrf_gpio_pin_read(R4_PIN)) {
		row_val = 4;
	}

	nrf_gpio_cfg_input(col_pin, NRF_GPIO_PIN_PULLDOWN);
	
	return row_val;
}


void (*callback)(nokia_key);

//index: column, row

nokia_key nokia_key_map[4][5] = {
{KNONE, KC, K1, K2, K3},
{KNONE, KMENU, K4, K5, K6},
{KDOWN, KNONE, K7, K8, K9},
{KNONE, KUP, KSTAR, K0, KPOUND}};


static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
		nrf_gpio_cfg_input(R0_PIN, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(R1_PIN, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(R2_PIN, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(R3_PIN, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(R4_PIN, NRF_GPIO_PIN_PULLDOWN);

		uint32_t row_val;
		nokia_key nk = KNONE;
	app_gpiote_user_disable(m_gpiote_user_id);

		if(event_pins_low_to_high & (1 << C1_PIN)) {
			row_val = check_row(C1_PIN);
			nk = nokia_key_map[0][row_val];
		}
		if(event_pins_low_to_high & (1 << C2_PIN)) {
			row_val = check_row(C2_PIN);
			nk = nokia_key_map[1][row_val];
		}
		if(event_pins_low_to_high & (1 << C3_PIN)) {
			row_val = check_row(C3_PIN);
			nk = nokia_key_map[2][row_val];
		}
		if(event_pins_low_to_high & (1 << C4_PIN)) {
			row_val = check_row(C4_PIN);
			nk = nokia_key_map[3][row_val];
		}

			

	nrf_gpio_cfg_output(R0_PIN);
	nrf_gpio_cfg_output(R1_PIN);
	nrf_gpio_cfg_output(R2_PIN);
	nrf_gpio_cfg_output(R3_PIN);
	nrf_gpio_cfg_output(R4_PIN);

	nrf_gpio_pin_set(R0_PIN);
	nrf_gpio_pin_set(R1_PIN);
	nrf_gpio_pin_set(R2_PIN);
	nrf_gpio_pin_set(R3_PIN);
	nrf_gpio_pin_set(R4_PIN);
	
	
	app_gpiote_user_enable(m_gpiote_user_id);

	callback(nk);
}


void keypad_init(void (*cb)(nokia_key)) {
	APP_GPIOTE_INIT(1);

	nrf_gpio_cfg_input(C1_PIN, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(C2_PIN, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(C3_PIN, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(C4_PIN, NRF_GPIO_PIN_PULLDOWN);
	
	uint32_t pins_transition_mask = (1 << C1_PIN) | (1 << C2_PIN) | (1 << C3_PIN) | (1 << C4_PIN);

	// Register button module as a GPIOTE user.
	app_gpiote_user_register(&m_gpiote_user_id, pins_transition_mask, 0, gpiote_event_handler);

	app_gpiote_user_enable(m_gpiote_user_id);
	
	nrf_gpio_cfg_output(R0_PIN);
	nrf_gpio_cfg_output(R1_PIN);
	nrf_gpio_cfg_output(R2_PIN);
	nrf_gpio_cfg_output(R3_PIN);
	nrf_gpio_cfg_output(R4_PIN);

	nrf_gpio_pin_set(R0_PIN);
	nrf_gpio_pin_set(R1_PIN);
	nrf_gpio_pin_set(R2_PIN);
	nrf_gpio_pin_set(R3_PIN);
	nrf_gpio_pin_set(R4_PIN);

	callback = cb;
	}



