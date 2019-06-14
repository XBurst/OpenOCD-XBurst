/***************************************************************************
 *   Copyright (C) 2017 by Zhou Yanjie, zhou_yan_jie@zoho.com               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"

#include <sys/mman.h>

#define GPIO_BASE	(0x10010000) /* GPIO controller */

#define PZINTS		(*(pio_base+0x714/4))
#define PZINTC		(*(pio_base+0x718/4))
#define PZMSKS		(*(pio_base+0x724/4))
#define PZMSKC		(*(pio_base+0x728/4))
#define PZPAT1S		(*(pio_base+0x734/4))
#define PZPAT1C		(*(pio_base+0x738/4))
#define PZPAT0S		(*(pio_base+0x744/4))
#define PZPAT0C		(*(pio_base+0x748/4))
#define PZGID2LD	(*(pio_base+0x7f0/4))

#define PxPIN(n)	(*(pio_base+(n*0x100+0x00)/4))
#define PxPAT0S(n)	(*(pio_base+(n*0x100+0x44)/4))
#define PxPAT0C(n)	(*(pio_base+(n*0x100+0x48)/4))

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static bb_value_t x1000_read(void);
static int x1000_write(int tck, int tms, int tdi);
static int x1000_reset(int trst, int srst);
static int x1000_led(int on);

static int x1000_init(void);
static int x1000_quit(void);

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;
static int led_gpio = -1;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static bb_value_t x1000_read(void)
{
	return !!(PxPIN(tdo_gpio/32) & 1<<tdo_gpio);
}

static int x1000_write(int tck, int tms, int tdi)
{
	PxPAT0S(tms_gpio/32) = tms<<tms_gpio;
	PxPAT0C(tms_gpio/32) = !tms<<tms_gpio;

	PxPAT0S(tdi_gpio/32) = tdi<<tdi_gpio;
	PxPAT0C(tdi_gpio/32) = !tdi<<tdi_gpio;

	PxPAT0S(tck_gpio/32) = tck<<tck_gpio;
	PxPAT0C(tck_gpio/32) = !tck<<tck_gpio;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int x1000_reset(int trst, int srst)
{
	if (trst_gpio > 0) {
		PxPAT0S(trst_gpio/32) = !trst<<trst_gpio;
		PxPAT0C(trst_gpio/32) = trst<<trst_gpio;
	}

	if (srst_gpio > 0) {
		PxPAT0S(srst_gpio/32) = !srst<<srst_gpio;
		PxPAT0C(srst_gpio/32) = srst<<srst_gpio;
	}
	return ERROR_OK;
}

static int x1000_led(int on)
{
	PxPAT0S(led_gpio/32) = on<<led_gpio;
	PxPAT0C(led_gpio/32) = !on<<led_gpio;
	return ERROR_OK;
}

static int x1000_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int x1000_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int x1000_speed(int speed)
{
	jtag_delay = speed;
	printf("jtag_delay = %08X\n",jtag_delay);
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio <= 101;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD_CTX,
			"GPIO config: tck = %d, tms = %d, tdi = %d, tdi = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD_CTX, "X1000 num: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD_CTX, "X1000 num: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD_CTX, "X1000 num: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD_CTX, "X1000 num: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD_CTX, "X1000 num: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD_CTX, "X1000 num: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_led)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], led_gpio);

	command_print(CMD_CTX, "X1000 num: led = %d", led_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}
	return ERROR_OK;
}

static const struct command_registration x1000_command_handlers[] = {
	{
		.name = "x1000_jtag_nums",
		.handler = &x1000_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "(tck tms tdi tdo)* ",
	},
	{
		.name = "x1000_tck_num",
		.handler = &x1000_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
	},
	{
		.name = "x1000_tms_num",
		.handler = &x1000_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
	},
	{
		.name = "x1000_tdo_num",
		.handler = &x1000_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
	},
	{
		.name = "x1000_tdi_num",
		.handler = &x1000_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
	},
	{
		.name = "x1000_srst_num",
		.handler = &x1000_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
	},
	{
		.name = "x1000_trst_num",
		.handler = &x1000_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
	},
	{
		.name = "x1000_led_num",
		.handler = &x1000_handle_jtag_gpionum_led,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for led.",
	},
	{
		.name = "x1000_speed_coeffs",
		.handler = &x1000_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
	},
	COMMAND_REGISTRATION_DONE
};

static struct bitbang_interface x1000_bitbang = {
	.read = x1000_read,
	.write = x1000_write,
	.reset = x1000_reset,
	.blink = x1000_led,
};

struct jtag_interface x1000_interface = {
	.name = "x1000",
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
	.transports = jtag_only,
	.speed = x1000_speed,
	.khz = x1000_khz,
	.speed_div = x1000_speed_div,
	.commands = x1000_command_handlers,
	.init = x1000_init,
	.quit = x1000_quit,
};

static int x1000_init(void)
{
	bitbang_interface = &x1000_bitbang;

	if (!is_gpio_valid(tdo_gpio) || !is_gpio_valid(tdi_gpio) ||
		!is_gpio_valid(tck_gpio) || !is_gpio_valid(tms_gpio) ||
		(trst_gpio != -1 && !is_gpio_valid(trst_gpio)) ||
		(srst_gpio != -1 && !is_gpio_valid(srst_gpio)) ||
		(led_gpio != -1 && !is_gpio_valid(led_gpio)))
		return ERROR_JTAG_INIT_FAILED;

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		perror("open");
		return ERROR_JTAG_INIT_FAILED;
	}

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, GPIO_BASE);

	if (pio_base == MAP_FAILED) {
		perror("mmap");
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	PZINTC = 1<<tdo_gpio;
	PZMSKS = 1<<tdo_gpio;
	PZPAT1S = 1<<tdo_gpio;
	PZGID2LD = tdo_gpio/32;

	PZINTC = 1<<tdi_gpio;
	PZMSKS = 1<<tdi_gpio;
	PZPAT1C = 1<<tdi_gpio;
	PZPAT0C = 1<<tdi_gpio;
	PZGID2LD = tdi_gpio/32;

	PZINTC = 1<<tck_gpio;
	PZMSKS = 1<<tck_gpio;
	PZPAT1C = 1<<tck_gpio;
	PZPAT0C = 1<<tck_gpio;
	PZGID2LD = tck_gpio/32;

	PZINTC = 1<<tms_gpio;
	PZMSKS = 1<<tms_gpio;
	PZPAT1C = 1<<tms_gpio;
	PZPAT0S = 1<<tms_gpio;
	PZGID2LD = tms_gpio/32;

	if (trst_gpio != -1) {
		PZINTC = 1<<trst_gpio;
		PZMSKS = 1<<trst_gpio;
		PZPAT1C = 1<<trst_gpio;
		PZPAT0S = 1<<trst_gpio;
		PZGID2LD = trst_gpio/32;
	}
	if (srst_gpio != -1) {
		PZINTC = 1<<srst_gpio;
		PZMSKS = 1<<srst_gpio;
		PZPAT1C = 1<<srst_gpio;
		PZPAT0S = 1<<srst_gpio;
		PZGID2LD = srst_gpio/32;
	}
	if (led_gpio != -1) {
		PZINTC = 1<<led_gpio;
		PZMSKS = 1<<led_gpio;
		PZPAT1C = 1<<led_gpio;
		PZPAT0C = 1<<led_gpio;
		PZGID2LD = led_gpio/32;
	}

	LOG_INFO("GPIO JTAG bitbang driver");

	return ERROR_OK;
}

static int x1000_quit(void)
{
	PZINTC = 1<<tdo_gpio;
	PZMSKS = 1<<tdo_gpio;
	PZPAT1S = 1<<tdo_gpio;
	PZGID2LD = tdo_gpio/32;

	PZINTC = 1<<tdi_gpio;
	PZMSKS = 1<<tdi_gpio;
	PZPAT1C = 1<<tdi_gpio;
	PZPAT0C = 1<<tdi_gpio;
	PZGID2LD = tdi_gpio/32;

	PZINTC = 1<<tck_gpio;
	PZMSKS = 1<<tck_gpio;
	PZPAT1C = 1<<tck_gpio;
	PZPAT0C = 1<<tck_gpio;
	PZGID2LD = tck_gpio/32;

	PZINTC = 1<<tms_gpio;
	PZMSKS = 1<<tms_gpio;
	PZPAT1C = 1<<tms_gpio;
	PZPAT0C = 1<<tms_gpio;
	PZGID2LD = tms_gpio/32;

	if (trst_gpio != -1) {
		PZINTC = 1<<trst_gpio;
		PZMSKS = 1<<trst_gpio;
		PZPAT1C = 1<<trst_gpio;
		PZPAT0C = 1<<trst_gpio;
		PZGID2LD = trst_gpio/32;
	}
	if (srst_gpio != -1) {
		PZINTC = 1<<srst_gpio;
		PZMSKS = 1<<srst_gpio;
		PZPAT1C = 1<<srst_gpio;
		PZPAT0C = 1<<srst_gpio;
		PZGID2LD = srst_gpio/32;
	}
	if (led_gpio != -1) {
		PZINTC = 1<<led_gpio;
		PZMSKS = 1<<led_gpio;
		PZPAT1C = 1<<led_gpio;
		PZPAT0C = 1<<led_gpio;
		PZGID2LD = led_gpio/32;
	}

	return ERROR_OK;
}

