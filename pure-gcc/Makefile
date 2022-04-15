# List all source files the application uses.
APPLICATION_SRCS = $(notdir $(wildcard ../*.c))
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += ble_bas.c
APPLICATION_SRCS += app_scheduler.c 
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += app_gpiote.c
APPLICATION_SRCS += nrf_delay.c
APPLICATION_SRCS += ble_hids.c
APPLICATION_SRCS += device_manager_peripheral.c
APPLICATION_SRCS += pstorage.c
APPLICATION_SRCS += spi_master.c

PROJECT_NAME = ble_app_beacon

DEVICE = NRF51
BOARD = BOARD_PCA10000
#SEGGER_SERIAL =

USE_SOFTDEVICE = s110

SDK_PATH = ../../../nrf_sdk/
TEMPLATE_PATH = ../../../nrf51-pure-gcc-setup/template/

CFLAGS = -O0 -g3 -DSPI_MASTER_0_ENABLE

GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
