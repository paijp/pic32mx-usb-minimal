/*
 * kb0007.c - PIC32MX270F256B USB HID Boot Keyboard minimal demo
 *
 * A single-file, no-interrupt, no-library USB HID keyboard for
 * PIC32MX270F256B. Enumerates as a HID boot keyboard on Linux and
 * sends the letter 'a' once every ~1 second.
 *
 * Copyright (c) 2026 paijp
 *
 * Developed in collaboration with Anthropic's Claude (Claude Opus 4.6).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ---------------------------------------------------------------------------
 *
 * This code was written from scratch but draws on the following reference
 * implementations for the PIC32MX USB peripheral, in particular the
 * requirement to use full ping-pong buffering and the correct ordering of
 * module power-on during initialisation:
 *
 *   Microchip Libraries for Applications (MLA) USB stack
 *     https://github.com/MicrochipTech/mla_usb
 *     Apache License 2.0
 *
 *   signal11/m-stack (USB stack for PIC MCUs)
 *     https://github.com/signal11/m-stack
 *     Apache License 2.0 / GPL-3.0 / LGPL-3.0 (triple-licensed)
 *
 * Neither project's source code is copied here; only the architectural
 * knowledge was reused. USB vendor/product IDs 0x04D8/0x003F are
 * Microchip's published HID sample values.
 */

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#pragma config FNOSC    = PRIPLL
#pragma config POSCMOD  = XT
#pragma config FPLLIDIV = DIV_1
#pragma config FPLLMUL  = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FPBDIV   = DIV_1
#pragma config UPLLEN   = ON
#pragma config UPLLIDIV = DIV_1
#pragma config FUSBIDIO = OFF
#pragma config FVBUSONIO= OFF
#pragma config FWDTEN   = OFF
#pragma config JTAGEN   = OFF
#pragma config FSOSCEN  = OFF
#pragma config IESO     = OFF
#pragma config FCKSM    = CSDCMD
#pragma config OSCIOFNC = OFF
#pragma config ICESEL   = ICS_PGx1

/* ============================================================================
 *                             USB HID driver
 * ==========================================================================*/

typedef struct {
	volatile uint32_t stat;
	volatile uint32_t addr;
} bdt_t;

#define BDT_UOWN     0x0080
#define BDT_DATA01   0x0040
#define BDT_DTSEN    0x0008
#define BDT_BSTALL   0x0004
#define BDT_BC_SHIFT 16
#define BDT_BC_MASK  0x03FF0000

#define PID_MASK     0x003C
#define PID_SETUP    (0x0D << 2)

static bdt_t bdt[8] __attribute__((aligned(512)));

#define BD_EP0_OUT_E 0
#define BD_EP0_OUT_O 1
#define BD_EP0_IN_E  2
#define BD_EP0_IN_O  3
#define BD_EP1_IN_E  6
#define BD_EP1_IN_O  7

static uint8_t ep0_out_buf[2][64];
static uint8_t ep0_in_buf[2][64];
static uint8_t ep1_in_buf[2][8];

static const uint8_t dev_desc[] = {
	18, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 64,
	0xD8, 0x04, 0x3F, 0x00, 0x01, 0x00, 1, 2, 0, 1
};

#define CFG_TOTAL (9 + 9 + 9 + 7)
static const uint8_t cfg_desc[CFG_TOTAL] = {
	9, 0x02, CFG_TOTAL, 0x00, 1, 1, 0, 0xA0, 50,
	9, 0x04, 0, 0, 1, 0x03, 0x01, 0x01, 0,
	9, 0x21, 0x11, 0x01, 0x00, 1, 0x22, 63, 0x00,
	7, 0x05, 0x81, 0x03, 0x08, 0x00, 10
};

static const uint8_t hid_report_desc[] = {
	0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
	0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
	0x95, 0x05, 0x75, 0x01, 0x05, 0x08,
	0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
	0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
	0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
	0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
	0xC0
};

static const uint8_t str0[] = { 4, 0x03, 0x09, 0x04 };
static const uint8_t str1[] = { 2 + 2 * 2, 0x03, 'k', 0, 'b', 0 };
static const uint8_t str2[] = {
	2 + 6 * 2, 0x03,
	'k', 0, 'b', 0, '0', 0, '0', 0, '0', 0, '7', 0
};

typedef enum {
	CTRL_IDLE,
	CTRL_DATA_IN
} ctrl_state_t;

static ctrl_state_t ctrl_state;
static const uint8_t *ctrl_src;
static uint16_t ctrl_left;
static uint8_t ctrl_in_d1;
static uint8_t ep0_in_ppbi;
static uint8_t usb_addr_pending;
static uint8_t usb_config;
static uint8_t ep1_in_ppbi;

#define VA2PA(x)  ((uint32_t)(x) & 0x1FFFFFFF)

static void ep0_out_arm(uint8_t ppbi)
{
	uint8_t idx = (ppbi == 0) ? BD_EP0_OUT_E : BD_EP0_OUT_O;

	bdt[idx].addr = VA2PA(ep0_out_buf[ppbi]);
	bdt[idx].stat = ((uint32_t)64 << BDT_BC_SHIFT) | BDT_UOWN;
}

static void ep0_in_send(const void *data, uint16_t len)
{
	uint8_t idx = (ep0_in_ppbi == 0) ? BD_EP0_IN_E : BD_EP0_IN_O;
	uint8_t *buf = ep0_in_buf[ep0_in_ppbi];

	if (len > 64) {
		len = 64;
	}
	if (len) {
		memcpy(buf, data, len);
	}
	bdt[idx].addr = VA2PA(buf);
	bdt[idx].stat = ((uint32_t)len << BDT_BC_SHIFT) |
	                BDT_UOWN | BDT_DTSEN |
	                (ctrl_in_d1 ? BDT_DATA01 : 0);
	ctrl_in_d1 ^= 1;
	ep0_in_ppbi ^= 1;
}

static void ep0_in_zlp(void)
{
	uint8_t idx = (ep0_in_ppbi == 0) ? BD_EP0_IN_E : BD_EP0_IN_O;

	bdt[idx].addr = VA2PA(ep0_in_buf[ep0_in_ppbi]);
	bdt[idx].stat = (0u << BDT_BC_SHIFT) |
	                BDT_UOWN | BDT_DTSEN | BDT_DATA01;
	ep0_in_ppbi ^= 1;
}

static void ep0_in_continue(void)
{
	uint16_t chunk = ctrl_left;

	if (chunk > 64) {
		chunk = 64;
	}
	ep0_in_send(ctrl_src, chunk);
	ctrl_src += chunk;
	ctrl_left -= chunk;
	if (chunk == 0) {
		ctrl_state = CTRL_IDLE;
	}
}

static void handle_setup(const uint8_t *s)
{
	uint8_t bmRequestType = s[0];
	uint8_t bRequest = s[1];
	uint16_t wValue = (uint16_t)s[2] | ((uint16_t)s[3] << 8);
	uint16_t wIndex = (uint16_t)s[4] | ((uint16_t)s[5] << 8);
	uint16_t wLength = (uint16_t)s[6] | ((uint16_t)s[7] << 8);

	(void)wIndex;

	ctrl_in_d1 = 1;
	ctrl_state = CTRL_IDLE;

	if ((bmRequestType & 0x60) == 0) {
		switch (bRequest) {
		case 0x05:
			usb_addr_pending = (uint8_t)wValue;
			ep0_in_zlp();
			goto out;

		case 0x06:
		{
			uint8_t type = (uint8_t)(wValue >> 8);
			uint8_t index = (uint8_t)(wValue & 0xFF);
			const uint8_t *p = 0;
			uint16_t n = 0;

			if (type == 0x01) {
				p = dev_desc;
				n = sizeof dev_desc;
			} else if (type == 0x02) {
				p = cfg_desc;
				n = sizeof cfg_desc;
			} else if (type == 0x22) {
				p = hid_report_desc;
				n = sizeof hid_report_desc;
			} else if (type == 0x03) {
				if (index == 0) {
					p = str0;
					n = sizeof str0;
				} else if (index == 1) {
					p = str1;
					n = sizeof str1;
				} else if (index == 2) {
					p = str2;
					n = sizeof str2;
				}
			}

			if (p) {
				if (n > wLength) {
					n = wLength;
				}
				ctrl_src = p;
				ctrl_left = n;
				ctrl_state = CTRL_DATA_IN;
				ep0_in_continue();
				goto out;
			}
			break;
		}

		case 0x09:
			usb_config = (uint8_t)wValue;
			U1EP1 = 0x15;
			ep1_in_ppbi = 0;
			ep0_in_zlp();
			goto out;

		case 0x00:
			ep0_in_buf[ep0_in_ppbi][0] = 0;
			ep0_in_buf[ep0_in_ppbi][1] = 0;
			ctrl_src = ep0_in_buf[ep0_in_ppbi];
			ctrl_left = 2;
			ctrl_state = CTRL_DATA_IN;
			ep0_in_continue();
			goto out;

		case 0x08:
			ep0_in_buf[ep0_in_ppbi][0] = usb_config;
			ctrl_src = ep0_in_buf[ep0_in_ppbi];
			ctrl_left = 1;
			ctrl_state = CTRL_DATA_IN;
			ep0_in_continue();
			goto out;
		}
	} else if ((bmRequestType & 0x60) == 0x20) {
		/* HID class request: minimal accept (SET_IDLE etc.) */
		if (bmRequestType & 0x80) {
			ep0_in_buf[ep0_in_ppbi][0] = 0;
			ep0_in_buf[ep0_in_ppbi][1] = 0;
			ctrl_src = ep0_in_buf[ep0_in_ppbi];
			ctrl_left = (wLength > 2) ? 2 : wLength;
			ctrl_state = CTRL_DATA_IN;
			ep0_in_continue();
		} else {
			ep0_in_zlp();
		}
		goto out;
	}

	{
		uint8_t in_idx = (ep0_in_ppbi == 0) ? BD_EP0_IN_E : BD_EP0_IN_O;

		bdt[in_idx].stat = BDT_UOWN | BDT_BSTALL;
	}

out:
	U1CONbits.PKTDIS = 0;
}

static void handle_trn(void)
{
	uint8_t stat = U1STAT;
	uint8_t ep = (stat >> 4) & 0x0F;
	uint8_t dir = (stat >> 3) & 0x01;
	uint8_t ppbi = (stat >> 2) & 0x01;
	uint8_t idx = (ep << 2) | (dir << 1) | ppbi;
	bdt_t *b = &bdt[idx];
	uint8_t pid = (uint8_t)(b->stat & PID_MASK);

	U1IR = 0x08;

	if (ep == 0) {
		if (pid == PID_SETUP) {
			handle_setup(ep0_out_buf[ppbi]);
			ep0_out_arm(ppbi);
		} else if (dir == 1) {
			if (usb_addr_pending) {
				U1ADDR = usb_addr_pending;
				usb_addr_pending = 0;
			}
			if (ctrl_state == CTRL_DATA_IN && ctrl_left > 0) {
				ep0_in_continue();
			}
		} else {
			ep0_out_arm(ppbi);
		}
	}
	/* EP1 IN completion: nothing to do */
}

static void usb_bus_reset(void)
{
	U1ADDR = 0;
	usb_addr_pending = 0;
	usb_config = 0;
	ctrl_state = CTRL_IDLE;
	ctrl_in_d1 = 1;
	ep0_in_ppbi = 0;
	ep1_in_ppbi = 0;

	U1IR = 0xFF;
	U1EIR = 0xFF;

	U1CONbits.PPBRST = 1;
	U1CONbits.PPBRST = 0;

	U1EP0 = 0x0D;
	U1EP1 = 0;

	memset(bdt, 0, sizeof bdt);
	ep0_out_arm(0);
	ep0_out_arm(1);
}

static void usb_init_hw(void)
{
	uint32_t pa;

	U1CNFG1 = 0x02;

	U1CONbits.PPBRST = 1;
	U1CONbits.PPBRST = 0;

	U1IE = 0;
	U1EIE = 0;

	U1CONbits.USBEN = 1;

	U1IR = 0x08;
	U1IR = 0x08;
	U1IR = 0x08;
	U1IR = 0x08;
	U1IR = 0xFF;
	U1EIR = 0xFF;

	pa = VA2PA(bdt);
	U1BDTP1 = (uint8_t)((pa >> 8) & 0xFE);
	U1BDTP2 = (uint8_t)(pa >> 16);
	U1BDTP3 = (uint8_t)(pa >> 24);

	U1EP0 = 0;
	U1EP1 = 0;
	U1EP2 = 0;
	U1EP3 = 0;
	U1EP4 = 0;
	U1EP5 = 0;
	U1EP6 = 0;
	U1EP7 = 0;
	U1EP8 = 0;
	U1EP9 = 0;
	U1EP10 = 0;
	U1EP11 = 0;
	U1EP12 = 0;
	U1EP13 = 0;
	U1EP14 = 0;
	U1EP15 = 0;

	U1EP0 = 0x0D;

	U1ADDR = 0;
	usb_addr_pending = 0;
	usb_config = 0;
	ctrl_state = CTRL_IDLE;
	ctrl_in_d1 = 1;
	ep0_in_ppbi = 0;
	ep1_in_ppbi = 0;

	memset(bdt, 0, sizeof bdt);
	ep0_out_arm(0);
	ep0_out_arm(1);

	U1PWRCbits.USBPWR = 1;
}

/* ============================================================================
 *                          KB API implementation
 * ==========================================================================*/

static void kb_init(void)
{
	ANSELA = 0;
	ANSELB = 0;
	usb_init_hw();
}

static void kb_poll(void)
{
	if (U1IR & 0x01) {
		usb_bus_reset();
		U1IR = 0x01;
	}
	if (U1IR & 0x08) {
		handle_trn();
	}
	if (U1IR & 0x10) {
		U1IR = 0x10;
	}
	if (U1IR & 0x80) {
		U1IR = 0x80;
	}
	if (U1IR & 0x02) {
		U1EIR = 0xFF;
		U1IR = 0x02;
	}
}

static uint8_t kb_is_configured(void)
{
	return usb_config ? 1 : 0;
}

/*
 * kb_send_key:
 *   If the next EP1 IN BD is free, arm it with an 8-byte HID boot
 *   keyboard report and return 1.  Returns 0 if the device is not
 *   configured or the BD is still in flight.
 */
static uint8_t kb_send_key(uint8_t modifier, uint8_t code)
{
	uint8_t idx;
	uint8_t *buf;
	uint32_t s;

	if (!usb_config) {
		return 0;
	}

	idx = (ep1_in_ppbi == 0) ? BD_EP1_IN_E : BD_EP1_IN_O;
	if (bdt[idx].stat & BDT_UOWN) {
		return 0;
	}

	buf = ep1_in_buf[ep1_in_ppbi];
	buf[0] = modifier;
	buf[1] = 0;
	buf[2] = code;
	buf[3] = 0;
	buf[4] = 0;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = 0;

	bdt[idx].addr = VA2PA(buf);
	s = ((uint32_t)8 << BDT_BC_SHIFT) | BDT_UOWN | BDT_DTSEN;
	if (ep1_in_ppbi) {
		s |= BDT_DATA01;
	}
	bdt[idx].stat = s;

	ep1_in_ppbi ^= 1;
	return 1;
}

/* ============================================================================
 *                             Application
 * ==========================================================================*/

int main(void)
{
	uint32_t tick = 0;
	uint8_t phase = 0;

	kb_init();

	for (;;) {
		kb_poll();

		if (kb_is_configured()) {
			tick++;
			if (tick >= 200000UL) {
				tick = 0;
				if (phase == 0) {
					if (kb_send_key(0, 0x04)) {
						phase = 1;
					}
				} else {
					if (kb_send_key(0, 0x00)) {
						phase = 0;
					}
				}
			}
		}
	}
	return 0;
}
