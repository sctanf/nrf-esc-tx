#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <zephyr/settings/settings.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "battery.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/init.h>
#include <hal/nrf_gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

LOG_MODULE_REGISTER(main, 4);

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define RBT_CNT_ID 1
#define PAIRED_ID 2

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);

// this was randomly generated
uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};
uint8_t base_addr_0[4] = {0,0,0,0};
uint8_t base_addr_1[4] = {0,0,0,0};
uint8_t addr_prefix[8] = {0,0,0,0,0,0,0,0};
uint8_t paired_addr[8] = {0,0,0,0,0,0,0,0};

uint8_t reset_mode = -1;

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);

int tickrate = 5;

uint8_t fan_batt;
uint8_t batt;
uint8_t batt_v;
uint32_t batt_pptt;

bool main_running = false;

unsigned int last_pot[8] = {0,0,0,0,0,0,0,0};
unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
int8_t last_pot_i = 0;
int8_t last_batt_pptt_i = 0;
bool system_off_main = false;
bool send_data = false;
int64_t idle_start_time = 0;
bool pot_idle = false;
int64_t last_data_sent = 0;
#define pot_idle_time_dur 10000

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			if (paired_addr[0] == 0x00) {
				if (rx_payload.length == 8) {
					for (int i = 0; i < 8; i++) {
						paired_addr[i] = rx_payload.data[i];
					}
				}
			} else {
				if (rx_payload.length == 4) {
					if (rx_payload.data[0] == rx_payload.data[1]) {
						fan_batt = rx_payload.data[0];
						last_data_sent = k_uptime_get();
					}
				}
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	sys_notify_init_spinwait(&clk_cli.notify);
	err = onoff_request(clk_mgr, &clk_cli);
	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
	} while (err);
	return 0;
}

int esb_initialize(void)
{
	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	// config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
	config.bitrate = ESB_BITRATE_1MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 8;
	// config.retransmit_delay = 600;
	//config.retransmit_count = 0;
	//config.tx_mode = ESB_TXMODE_MANUAL;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	esb_init(&config);
	esb_set_base_address_0(base_addr_0);
	esb_set_base_address_1(base_addr_1);
	esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	return 0;
}

int esb_initialize_rx(void)
{
	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.bitrate = ESB_BITRATE_1MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 8;
	// config.retransmit_delay = 600;
	// config.retransmit_count = 3;
	// config.tx_mode = ESB_TXMODE_AUTO;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	esb_init(&config);
	esb_set_base_address_0(base_addr_0);
	esb_set_base_address_1(base_addr_1);
	esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	return 0;
}

void configure_system_off(void){
	// Configure dock interrupt
	// Set system off
	sys_poweroff();
}

void power_check(void) {
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	if (batt_pptt == 0) {
		gpio_pin_set_dt(&led, 0); // Turn off LED
		//configure_system_off();
	}
	LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
}

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

const struct device *pot_adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER_NODE));
int16_t raw;

#include <zephyr/init.h>

int32_t reset_reason;
static int early_init()
{
	reset_reason = NRF_POWER->RESETREAS;
//	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	return 0;
}

SYS_INIT(early_init, POST_KERNEL, 0);

int main(void)
{
//	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;
	bool booting_from_shutdown = false;

//	power_check(); // check the battery first before continuing (4ms delta to read from ADC)

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 1);

	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);

	struct adc_sequence asp = {
		.channels = BIT(0),
		.buffer = &raw,
		.buffer_size = sizeof(raw),
		.oversampling = 4,
		.calibrate = true,
	};
	asp.resolution = 14;
	struct adc_channel_cfg accp = {
		.gain = ADC_GAIN_1_6,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	};
	accp.input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0;
	adc_channel_setup(pot_adc, &accp);

	if (reset_reason & 0x01) { // Count pin resets
		nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		reset_mode = reboot_counter;
		reboot_counter++;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 0;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	}
	gpio_pin_set_dt(&led, 0);

	if (reset_mode == 1) { // Reset mode pairing reset
		LOG_INF("Enter pairing reset");
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Clear paired address
		reset_mode = 0; // Clear reset mode
	} else {
		// Read paired address from NVS
		nvs_read(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr));
	}

	clocks_start();

	if (paired_addr[0] == 0x00) { // No dongle paired
		for (int i = 0; i < 4; i++) {
			base_addr_0[i] = discovery_base_addr_0[i];
			base_addr_1[i] = discovery_base_addr_1[i];
		}
		for (int i = 0; i < 8; i++) {
			addr_prefix[i] = discovery_addr_prefix[i];
		}
		esb_initialize();
		tx_payload_pair.noack = false;
		uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
		uint8_t check = addr & 255;
		if (check == 0) check = 8;
		LOG_INF("Check Code: %02X", paired_addr[0]);
		tx_payload_pair.data[0] = check; // Use int from device address to make sure packet is for this device
		for (int i = 0; i < 6; i++) {
			tx_payload_pair.data[i+2] = (addr >> (8 * i)) & 0xFF;
		}
		while (paired_addr[0] != check) {
			if (paired_addr[0] != 0x00) {
				LOG_INF("Incorrect check code: %02X", paired_addr[0]);
				paired_addr[0] = 0x00; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // Still fails after a while
			esb_start_tx();
			gpio_pin_set_dt(&led, 1);
			k_msleep(100);
			gpio_pin_set_dt(&led, 0);
			k_msleep(900);
			power_check();
		}
		LOG_INF("Paired");
		nvs_write(&fs, PAIRED_ID, &paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_disable();
	}

	// Recreate dongle address
	uint8_t buf2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	for (int i = 0; i < 4; i++) {
		buf2[i] = paired_addr[i+2];
		buf2[i+4] = paired_addr[i+2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++) {
		buf2[i+8] = paired_addr[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (buf2[i] == 0x00 || buf2[i] == 0x55 || buf2[i] == 0xAA) {
			buf2[i] += 8;
		};
	}
	for (int i = 0; i < 4; i++) {
		base_addr_0[i] = buf2[i];
		base_addr_1[i] = buf2[i+4];
	}
	for (int i = 0; i < 8; i++) {
		addr_prefix[i] = buf2[i+8];
	}
	
	esb_initialize();

	tx_payload.noack = false;

	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();

		int batt_mV;
		batt_pptt = read_batt_mV(&batt_mV);
		last_batt_pptt[last_batt_pptt_i] = batt_pptt;
		last_batt_pptt_i++;
		last_batt_pptt_i %= 15;
		for (uint8_t i = 0; i < 15; i++) {  // Average battery readings across 16 samples
			if (last_batt_pptt[i] == 10001) {
				batt_pptt += batt_pptt / (i + 1);
			} else {
				batt_pptt += last_batt_pptt[i];
			}
		}
		batt_pptt /= 16;
		if (batt_pptt + 49 < last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt + 49;} // Lower bound -100pptt
		else if (batt_pptt - 49 > last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt - 49;} // Upper bound +0pptt
		else {batt_pptt = last_batt_pptt[15];} // Effectively 100-10000 -> 1-100%
		int batt = (int)((float)batt_pptt/100.0);

		adc_channel_setup(pot_adc, &accp);
		uint64_t pot_total = 0;
		for (int i = 0; i < 4; i++) {
			adc_read(pot_adc, &asp);
			int32_t val = raw;
			adc_raw_to_millivolts(adc_ref_internal(pot_adc),
					      accp.gain,
					      asp.resolution,
					      &val);
			//pot_total += raw;
			pot_total += val;
		}
		last_pot[last_pot_i] = pot_total;
		last_pot_i++;
		last_pot_i %= 7;
		for (uint8_t i = 0; i < 7; i++) {  // Average pot readings across 8 samples
			if (last_pot[i] == 0) {
				pot_total += pot_total / (i + 1);
			} else {
				pot_total += last_pot[i];
			}
		}
		float pot_total_out = 3300.0 * 10.0 / ((float)pot_total / 32.0) - 10.0;
		if (pot_total_out > 200) pot_total_out = 0; // but the measurement should already be zero, aka off (check from adc!!)
		else {
			pot_total_out /= 190;
			pot_total_out *= 0.93;
			pot_total_out += 0.07;
			pot_total_out *= 32767;
			if (pot_total_out < 0.07) pot_total_out = 0.07;
			if (pot_total_out > 32767) pot_total_out = 32767;
		}
		raw = pot_total_out;

		float idle_val = 0.05 * 32768.0;
		float idle_exit_val = 0.05 * 32768.0;
		if ((pot_total_out > -idle_val && pot_total_out < idle_val) && pot_idle == false)
		{
			pot_idle = true;
			idle_start_time = k_uptime_get();
		}
		if ((pot_total_out < -idle_exit_val || pot_total_out > idle_exit_val) && pot_idle == true)
		{
			pot_idle = false;
		}

		if (pot_idle) // TODO use this pin to turn on/off device, if pulled to vcc turn on, if ~0v turn off
		{
//			raw = 0;
			tickrate = 50;
			gpio_pin_set_dt(&led, 0);
		}
		else
		{
			tickrate = 5;
			gpio_pin_set_dt(&led, 1);
		}

		asp.calibrate = false;
		tx_payload.data[0] = (raw >> 8) & 0xff;
		tx_payload.data[1] = raw & 0xff;
		tx_payload.data[2] = (raw >> 8) & 0xff;
		tx_payload.data[3] = raw & 0xff;
//		tx_payload.data[4] = batt;
//		tx_payload.data[5] = batt;
		esb_flush_tx();
		esb_write_payload(&tx_payload); // Add transmission to queue
		esb_start_tx();

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		if (time_delta > tickrate)
		{
			k_yield();
		}
		else
		{
			k_msleep(tickrate - time_delta);
		}
	}
}
// Transmitter, has OLED, transmits pwm and receives battery and rpm and data from pump