#include "nordic_common.h"
#include "nrf_gpio.h"
#include "app_gpiote.h"
#include "nrf_delay.h"


#include "keyboard.h"
#include "scancodes.h"

uint8_t row_pins[] = {13, 14, 15, 16, 17, 19};
uint8_t col_pins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

static app_gpiote_user_id_t           m_gpiote_user_id;



int row_col_to_scancode[sizeof(row_pins)][sizeof(col_pins)] = {
{KEY_ESC, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12},
{KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_MINUS, KEY_EQUAL, KEY_BACKSPACE},
{KEY_TAB, KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Z, KEY_U, KEY_I, KEY_O, KEY_P, KEY_LEFTBRACE, KEY_RIGHTBRACE},
{KEY_CAPSLOCK, KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K, KEY_L, KEY_SEMICOLON, KEY_APOSTROPHE, KEY_BACKSLASH},
{KEY_MOD_LSHIFT, KEY_Y, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_DOT, KEY_COMMA, KEY_SLASH, 0, KEY_ENTER},
{KEY_MOD_LCTRL, KEY_MOD_LMETA, KEY_MOD_LALT, 0, KEY_SPACE, 0, 0, 0, 0, KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN}
};



static uint32_t check_col(uint32_t row_pin_idx) {
	nrf_gpio_cfg_output(row_pins[row_pin_idx]);
	nrf_gpio_pin_clear(row_pins[row_pin_idx]);

	int i;
	int scancode = 0;
	for(i=0; i<sizeof(col_pins); ++i) {
		if(!nrf_gpio_pin_read(col_pins[i]))
			scancode = row_col_to_scancode[row_pin_idx][i];
	}

	nrf_gpio_cfg_input(row_pins[row_pin_idx], NRF_GPIO_PIN_PULLDOWN);
	
	return scancode;
}


static void (*callback)(int);


static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
	int i;
	for(i=0; i<sizeof(col_pins); ++i)
		nrf_gpio_cfg_input(col_pins[i], NRF_GPIO_PIN_PULLUP);
		

	int scancode;
	app_gpiote_user_disable(m_gpiote_user_id);

	for(i=0; i<sizeof(row_pins); ++i) {
		if(event_pins_low_to_high & (1 << row_pins[i]))
			scancode= check_col(i);
	}
			
	for(i=0; i<sizeof(col_pins); ++i) {
		nrf_gpio_cfg_output(col_pins[i]);
		nrf_gpio_pin_set(col_pins[i]);
	}

	app_gpiote_user_enable(m_gpiote_user_id);

	callback(scancode);
}


void keyboard_init(void (*cb)(int)) {
	APP_GPIOTE_INIT(1);

	int32_t pins_transition_mask = 0;
	int i;
	for(i=0; i<sizeof(row_pins); ++i) {
		nrf_gpio_cfg_input(row_pins[i], NRF_GPIO_PIN_PULLDOWN);
		pins_transition_mask |= (1 << row_pins[i]);
	}	

	// Register button module as a GPIOTE user.
	app_gpiote_user_register(&m_gpiote_user_id, pins_transition_mask, 0, gpiote_event_handler);

	app_gpiote_user_enable(m_gpiote_user_id);
	
	for(i=0; i<sizeof(col_pins); ++i) {
		nrf_gpio_cfg_output(col_pins[i]);
		nrf_gpio_pin_set(col_pins[i]);
	}

	callback = cb;
	}



