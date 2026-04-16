/*
 * usbhost0015.c - PIC32MX270F256B USB host minimal demo
 *
 * A single-file, no-interrupt, no-library USB host for
 * PIC32MX270F256B.  Enumerates an attached full-speed USB printer
 * (class 0x07) or a low-speed HID boot keyboard (class 0x03) and
 * dispatches to a per-class handler.  Any other device is accepted
 * onto the bus but left idle until detach.
 *
 * Clocking: 4 MHz external crystal
 *   SYSCLK: 4 MHz / FPLLIDIV(1) * FPLLMUL(20) / FPLLODIV(2) = 40 MHz
 *   USB   : 4 MHz / UPLLIDIV(1) * 12                        = 48 MHz
 *
 * Design notes:
 *   - Only four BDT entries are used (IN/OUT x EVEN/ODD).  In host
 *     mode the endpoint is selected per token via U1TOK, so the BDT
 *     is not indexed by endpoint number.
 *   - U1EP0 and U1ADDR are re-applied before every token.
 *   - RETRYDIS is cleared for control transfers (hardware retries)
 *     and set for bulk/interrupt (software retry on NAK).
 *   - All busy-wait loops call usb_polltask (if non-NULL) so that
 *     the application can perform background work while USB is
 *     waiting for hardware events.
 *
 * Build: XC32 (PIC32MX270F256B / 28-pin SOIC or DIP)
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 * -------------------------------------------------------------------
 *
 * This code was written from scratch but draws on the following
 * reference implementations for the PIC32MX USB peripheral, in
 * particular the register initialisation order, bus reset timing,
 * ping-pong management, and the requirement to re-apply
 * U1EP0/U1ADDR before each token issued in host mode:
 *
 *   Microchip Libraries for Applications (MLA) USB stack
 *     https://github.com/MicrochipTech/mla_usb
 *     Apache License 2.0
 *
 *   signal11/m-stack (USB stack for PIC MCUs)
 *     https://github.com/signal11/m-stack
 *     Apache License 2.0 / GPL-3.0 / LGPL-3.0 (triple-licensed)
 *
 * Neither project's source code is copied here; only the
 * architectural knowledge was reused.
 */
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>

/* ============================================================
 * Configuration bits
 * ============================================================ */
#pragma config FPLLIDIV = DIV_1
#pragma config FPLLMUL  = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FNOSC    = PRIPLL
#pragma config POSCMOD  = XT
#pragma config FSOSCEN  = OFF
#pragma config UPLLIDIV = DIV_1
#pragma config UPLLEN   = ON
#pragma config FPBDIV   = DIV_1
#pragma config FWDTEN   = OFF
#pragma config JTAGEN   = OFF

#define SYS_CLK_HZ       40000000UL
#define CORE_TICK_PER_MS  (SYS_CLK_HZ / 2 / 1000)

/* ============================================================
 * Constants
 * ============================================================ */
#define USB_PID_SETUP  0xD
#define USB_PID_IN     0x9
#define USB_PID_OUT    0x1

#define KVA_TO_PA(v)  ((uint32_t)(v) & 0x1FFFFFFF)

/* ============================================================
 * BDT
 * MLA style: 4 entries (IN Even, IN Odd, OUT Even, OUT Odd).
 * The endpoint number is selected via U1TOK, not the BDT index.
 * ============================================================ */
typedef struct {
	uint32_t stat;
	uint32_t adr;
} BdtEntry;

#define BDT_UOWN   (1u << 7)
#define BDT_DATA1  (1u << 6)
#define BDT_DTS    (1u << 3)
#define BDT_BC(n)  (((uint32_t)(n) & 0x3FF) << 16)

#define BDT_IN_EVEN    0
#define BDT_IN_ODD     1
#define BDT_OUT_EVEN   2
#define BDT_OUT_ODD    3
#define BDT_SIZE       4

static BdtEntry __attribute__((aligned(512))) g_bdt[BDT_SIZE];

/* ============================================================
 * Buffers
 * ============================================================ */
static uint8_t g_ep0_rx_buf[64];
static uint8_t g_ep0_tx_buf[64];
static uint8_t g_bulk_buf[64];
static uint8_t g_hid_buf[8];

/* ============================================================
 * State
 * ============================================================ */
typedef enum {
	USB_DEV_UNKNOWN,
	USB_DEV_PRINTER,
	USB_DEV_KEYBOARD
} UsbDevType;

typedef enum {
	USB_OK = 0,
	USB_ERR_NAK_TIMEOUT,
	USB_ERR_STALL,
	USB_ERR_TIMEOUT
} UsbResult;

#define BULK_RETRY_MAX  100

static UsbDevType g_dev_type     = USB_DEV_UNKNOWN;
static uint8_t    g_dev_addr     = 0;
static uint8_t    g_bulk_ep      = 0;
static uint8_t    g_hid_ep       = 0;
static uint8_t    g_bulk_toggle  = 0;
static uint8_t    g_hid_toggle   = 0;
static uint8_t    g_ep0_toggle   = 0;
static uint8_t    g_is_low_speed = 0;
static uint8_t    g_ep0_max_pkt  = 8;

/* Ping-pong tracking (MLA bfPingPongIn/bfPingPongOut equivalent) */
static uint8_t g_pp_in  = 0;  /* 0 = EVEN next, 1 = ODD next */
static uint8_t g_pp_out = 0;

/* ============================================================
 * Background task hook
 *
 * If non-NULL this function is called repeatedly inside every
 * busy-wait loop (delay, attach wait, token completion wait,
 * detach wait).  The callee must return promptly.
 * ============================================================ */
void (*usb_polltask)(void);

/* ============================================================
 * Utility
 * ============================================================ */
static void poll_call(void)
{
	if (usb_polltask) {
		usb_polltask();
	}
}

int usb_is_detached(void)
{
	return U1IRbits.DETACHIF != 0;
}

static void my_memset(uint8_t *dst, uint8_t val, uint16_t len)
{
	while (len--) {
		*dst++ = val;
	}
}

static void my_memcpy(uint8_t *dst, const uint8_t *src, uint16_t len)
{
	while (len--) {
		*dst++ = *src++;
	}
}

static void delay_init_ms(uint32_t ms)
{
	uint32_t start = _CP0_GET_COUNT();
	uint32_t ticks = ms * CORE_TICK_PER_MS;

	while ((_CP0_GET_COUNT() - start) < ticks) {
		poll_call();
	}
}

static void delay_usbms(uint32_t ms)
{
	while (ms--) {
		U1OTGIR = _U1OTGIR_T1MSECIF_MASK;
		while (!(U1OTGIR & _U1OTGIR_T1MSECIF_MASK)) {
			poll_call();
		}
	}
}

/* ============================================================
 * Ping-pong BDT selection
 * ============================================================ */
static uint8_t pick_bdt_in(void)
{
	uint8_t idx = g_pp_in ? BDT_IN_ODD : BDT_IN_EVEN;
	g_pp_in ^= 1;
	return idx;
}

static uint8_t pick_bdt_out(void)
{
	uint8_t idx = g_pp_out ? BDT_OUT_ODD : BDT_OUT_EVEN;
	g_pp_out ^= 1;
	return idx;
}

static void reset_ping_pong(void)
{
	g_pp_in  = 0;
	g_pp_out = 0;
}

/*
 * U1EP0 bit 6 is RETRYDIS (Retry Disable).
 * Define it manually in case the xc.h version lacks the macro.
 */
#ifndef _U1EP0_RETRYDIS_MASK
#define EP_RETRYDIS   0x40
#else
#define EP_RETRYDIS   _U1EP0_RETRYDIS_MASK
#endif

/* ============================================================
 * Transfer primitives
 * ============================================================ */

/*
 * token_send - MLA _USB_SendToken equivalent.
 *
 * Re-applies U1EP0 and U1ADDR before every token so that
 * low-speed / full-speed switching and any stale register state
 * are handled reliably.
 *
 * pid:        USB_PID_SETUP / IN / OUT
 * ep:         endpoint number (0..15)
 * is_control: 1 enables SETUP (clears EPCONDIS), 0 for bulk/int
 */
static void token_send(uint8_t pid, uint8_t ep, uint8_t is_control)
{
	uint8_t ep_val;
	uint8_t addr_val;

	/*
	 * Control: RETRYDIS=0 (HW auto-retry, NAK absorbed by SIE)
	 * Int/Bulk: RETRYDIS=1 (NAK surfaces immediately for SW retry)
	 */
	ep_val = _U1EP0_EPRXEN_MASK
	       | _U1EP0_EPTXEN_MASK
	       | _U1EP0_EPHSHK_MASK;
	if (!is_control) {
		ep_val |= _U1EP0_EPCONDIS_MASK;
		ep_val |= EP_RETRYDIS;
	}
	if (g_is_low_speed) {
		ep_val |= _U1EP0_LSPD_MASK;
	}
	U1EP0 = ep_val;

	/* U1ADDR: set LSEN for low-speed devices */
	addr_val = g_dev_addr;
	if (g_is_low_speed) {
		addr_val |= 0x80;
	}
	U1ADDR = addr_val;

	/* Issue the token */
	U1TOK = (pid << 4) | (ep & 0x0F);
}

static UsbResult wait_trn(void)
{
	uint32_t timeout = 500000;

	while (!U1IRbits.TRNIF) {
		if (--timeout == 0) {
			return USB_ERR_TIMEOUT;
		}
		if (U1IRbits.DETACHIF) {
			return USB_ERR_TIMEOUT;
		}
		poll_call();
	}
	U1IR = _U1IR_TRNIF_MASK;
	__asm__("nop");
	__asm__("nop");
	return USB_OK;
}

static UsbResult check_pid(uint8_t bdt_idx)
{
	uint8_t pid = (g_bdt[bdt_idx].stat >> 2) & 0x0F;

	if (pid == 0x0A) {
		return USB_ERR_NAK_TIMEOUT;
	}
	if (pid == 0x0E) {
		return USB_ERR_STALL;
	}
	return USB_OK;
}

/* ============================================================
 * USB initialisation
 * ============================================================ */
void usb_init(void)
{
	uint32_t pa;

	my_memset((uint8_t *)g_bdt, 0, sizeof(g_bdt));
	reset_ping_pong();

	/* Disable GPIO / analogue on D+/D- pins (RB10/RB11) */
	ANSELB &= ~((1 << 10) | (1 << 11));
	CNPUB   = 0;   /* clear any stale weak pull-ups */
	CNPDB   = 0;

	/* Disable all USB interrupts and clear flags */
	U1IE    = 0;
	U1IR    = 0xFF;
	U1OTGIE = 0;
	U1OTGIR = 0x7D;
	U1EIE   = 0;
	U1EIR   = 0xFF;

	/* BDT base address */
	pa = KVA_TO_PA((uint32_t)g_bdt);
	U1BDTP1 = (pa >> 8)  & 0xFF;
	U1BDTP2 = (pa >> 16) & 0xFF;
	U1BDTP3 = (pa >> 24) & 0xFF;

	/* HOSTEN + PPBRST sequence */
	U1CON = _U1CON_HOSTEN_MASK;
	U1CON = _U1CON_HOSTEN_MASK | _U1CON_PPBRST_MASK;
	U1CON = _U1CON_HOSTEN_MASK;

	/* D+/D- pull-downs + VBUS on (separate write) */
	U1OTGCON = _U1OTGCON_DPPULDWN_MASK | _U1OTGCON_DMPULDWN_MASK;
	U1OTGCON |= _U1OTGCON_VBUSON_MASK;

	/* Full ping-pong */
	U1CNFG1 = 0x02;

	U1ADDR = 0;
	U1EP0  = _U1EP0_EPCONDIS_MASK
	       | _U1EP0_EPRXEN_MASK
	       | _U1EP0_EPTXEN_MASK
	       | _U1EP0_EPHSHK_MASK;
	/* RETRYDIS is set per-transfer inside token_send() */
	U1SOF = 0x4A;

	/* Power on the USB module last */
	U1PWRCbits.USBPWR = 1;
	delay_init_ms(10);
}

/* ============================================================
 * Attach wait and bus reset
 * ============================================================ */
void usb_wait_attach_and_reset(void)
{
	while (!U1IRbits.ATTACHIF) {
		poll_call();
	}
	U1IR = _U1IR_ATTACHIF_MASK;

	delay_usbms(200);

	if (!U1CONbits.JSTATE) {
		g_is_low_speed = 1;
	} else {
		g_is_low_speed = 0;
	}

	/* Re-reset ping-pong (both HW and SW) */
	U1CONbits.PPBRST = 1;
	U1CONbits.PPBRST = 0;
	reset_ping_pong();

	/* Bus reset: assert for 50 ms */
	U1CONbits.USBRST = 1;
	delay_usbms(50);
	U1CONbits.USBRST = 0;

	/* MLA order: enable SOF immediately after reset release */
	U1CONbits.SOFEN = 1;

	/* Reset recovery: low-speed devices need ~100 ms */
	delay_usbms(100);

	g_dev_addr  = 0;
	g_ep0_toggle = 0;

	U1IR = _U1IR_DETACHIF_MASK;
}

/* ============================================================
 * Control transfer primitives (ping-pong aware)
 * ============================================================ */
static void ctrl_setup(uint8_t *pkt8)
{
	uint8_t idx = pick_bdt_out();

	my_memcpy(g_ep0_tx_buf, pkt8, 8);

	g_bdt[idx].adr  = KVA_TO_PA((uint32_t)g_ep0_tx_buf);
	g_bdt[idx].stat = BDT_UOWN | BDT_DTS | BDT_BC(8);

	token_send(USB_PID_SETUP, 0, 1);
	wait_trn();
	g_ep0_toggle = 1;
}

static uint16_t ctrl_in(uint8_t *data, uint16_t max_len)
{
	uint16_t total = 0;
	uint16_t chunk_len;
	uint16_t rx_len;
	uint8_t  idx;

	while (total < max_len) {
		chunk_len = max_len - total;
		if (chunk_len > g_ep0_max_pkt) {
			chunk_len = g_ep0_max_pkt;
		}

		idx = pick_bdt_in();
		g_bdt[idx].adr  = KVA_TO_PA((uint32_t)g_ep0_rx_buf);
		g_bdt[idx].stat = BDT_UOWN | BDT_DTS
		                | (g_ep0_toggle ? BDT_DATA1 : 0)
		                | BDT_BC(chunk_len);

		token_send(USB_PID_IN, 0, 1);
		if (wait_trn() != USB_OK) {
			break;
		}

		rx_len = (g_bdt[idx].stat >> 16) & 0x3FF;
		my_memcpy(data + total, g_ep0_rx_buf, rx_len);
		total += rx_len;
		g_ep0_toggle ^= 1;

		/* Short packet terminates the transfer */
		if (rx_len < chunk_len) {
			break;
		}
	}
	return total;
}

static void ctrl_out_zlp(void)
{
	uint8_t idx = pick_bdt_out();

	g_bdt[idx].adr  = KVA_TO_PA((uint32_t)g_ep0_tx_buf);
	g_bdt[idx].stat = BDT_UOWN | BDT_DTS | BDT_DATA1 | BDT_BC(0);

	token_send(USB_PID_OUT, 0, 1);
	wait_trn();
}

static void ctrl_in_zlp(void)
{
	uint8_t idx = pick_bdt_in();

	g_bdt[idx].adr  = KVA_TO_PA((uint32_t)g_ep0_rx_buf);
	g_bdt[idx].stat = BDT_UOWN | BDT_DTS | BDT_DATA1 | BDT_BC(0);

	token_send(USB_PID_IN, 0, 1);
	wait_trn();
}

/* ============================================================
 * Standard requests
 * ============================================================ */
static uint16_t usb_get_descriptor(uint8_t type, uint8_t idx,
                                   uint8_t *buf, uint16_t len)
{
	uint8_t  setup[8];
	uint16_t rx_len;

	setup[0] = 0x80; setup[1] = 0x06;
	setup[2] = idx;  setup[3] = type;
	setup[4] = 0x00; setup[5] = 0x00;
	setup[6] = (uint8_t)(len & 0xFF);
	setup[7] = (uint8_t)(len >> 8);

	ctrl_setup(setup);
	rx_len = ctrl_in(buf, len);
	ctrl_out_zlp();
	return rx_len;
}

static void usb_set_address(uint8_t addr)
{
	uint8_t setup[8];

	setup[0] = 0x00; setup[1] = 0x05;
	setup[2] = addr; setup[3] = 0x00;
	setup[4] = 0x00; setup[5] = 0x00;
	setup[6] = 0x00; setup[7] = 0x00;

	ctrl_setup(setup);
	ctrl_in_zlp();
	delay_usbms(2);

	g_dev_addr = addr;
}

static void usb_set_configuration(uint8_t cfg_val)
{
	uint8_t setup[8];

	setup[0] = 0x00; setup[1] = 0x09;
	setup[2] = cfg_val; setup[3] = 0x00;
	setup[4] = 0x00; setup[5] = 0x00;
	setup[6] = 0x00; setup[7] = 0x00;

	ctrl_setup(setup);
	ctrl_in_zlp();
}

static void usb_set_interface(uint8_t if_num, uint8_t alt_num)
{
	uint8_t setup[8];

	setup[0] = 0x01; setup[1] = 0x0B;
	setup[2] = alt_num; setup[3] = 0x00;
	setup[4] = if_num;  setup[5] = 0x00;
	setup[6] = 0x00; setup[7] = 0x00;

	ctrl_setup(setup);
	ctrl_in_zlp();
}

/* ============================================================
 * Configuration descriptor parser
 * ============================================================ */
static UsbDevType parse_config_desc(uint8_t *buf, uint16_t len)
{
	UsbDevType dev_type = USB_DEV_UNKNOWN;
	uint16_t i = 0;
	uint8_t  desc_len;
	uint8_t  desc_type;
	uint8_t  cls;
	uint8_t  subcls;
	uint8_t  proto;
	uint8_t  alt_num;
	uint8_t  ep_addr;
	uint8_t  attr;
	uint8_t  in_target = 0;

	g_bulk_ep = 0;
	g_hid_ep  = 0;

	while (i < len) {
		desc_len  = buf[i];
		desc_type = buf[i + 1];

		if (desc_type == 0x04) {  /* Interface descriptor */
			alt_num = buf[i + 3];
			cls     = buf[i + 5];
			subcls  = buf[i + 6];
			proto   = buf[i + 7];

			/*
			 * Accept only the first matching interface
			 * at alternate setting 0.
			 */
			if (alt_num == 0 && dev_type == USB_DEV_UNKNOWN) {
				if (cls == 0x07) {
					dev_type  = USB_DEV_PRINTER;
					in_target = 1;
				} else if (cls == 0x03
				           && subcls == 0x01
				           && proto == 0x01) {
					dev_type  = USB_DEV_KEYBOARD;
					in_target = 1;
				} else {
					in_target = 0;
				}
			} else {
				in_target = 0;
			}
		}

		if (desc_type == 0x05 && in_target) {  /* Endpoint */
			ep_addr = buf[i + 2];
			attr    = buf[i + 3];

			if (dev_type == USB_DEV_PRINTER) {
				if ((ep_addr & 0x80) == 0x00
				    && (attr & 0x03) == 0x02) {
					if (g_bulk_ep == 0) {
						g_bulk_ep = ep_addr & 0x0F;
					}
				}
			} else if (dev_type == USB_DEV_KEYBOARD) {
				if ((ep_addr & 0x80) == 0x80
				    && (attr & 0x03) == 0x03) {
					if (g_hid_ep == 0) {
						g_hid_ep = ep_addr & 0x0F;
					}
				}
			}
		}

		if (desc_len == 0) {
			break;
		}
		i += desc_len;
	}
	return dev_type;
}

static void hid_set_boot_protocol(void)
{
	uint8_t setup[8];

	setup[0] = 0x21; setup[1] = 0x0B;
	setup[2] = 0x00; setup[3] = 0x00;
	setup[4] = 0x00; setup[5] = 0x00;
	setup[6] = 0x00; setup[7] = 0x00;

	ctrl_setup(setup);
	ctrl_in_zlp();
}

/* ============================================================
 * Enumeration
 * ============================================================ */
void usb_enumerate(void)
{
	uint8_t  buf[256];
	uint16_t total_len;

	my_memset(buf, 0, sizeof(buf));
	g_ep0_max_pkt = 8;

	usb_get_descriptor(0x01, 0, buf, 8);

	g_ep0_max_pkt = buf[7];
	if (g_ep0_max_pkt == 0 || g_ep0_max_pkt > 64) {
		g_ep0_max_pkt = 8;
	}

	usb_set_address(0x01);

	usb_get_descriptor(0x01, 0, buf, 18);

	usb_get_descriptor(0x02, 0, buf, 9);

	total_len = buf[2] | ((uint16_t)buf[3] << 8);
	usb_get_descriptor(0x02, 0, buf, total_len);

	g_dev_type = parse_config_desc(buf, total_len);

	usb_set_configuration(buf[5]);

	if (g_dev_type == USB_DEV_PRINTER) {
		usb_set_interface(0, 0);

		/*
		 * Enable the EP1 register for bulk transfers.
		 * The endpoint number is still selected via U1TOK,
		 * but the EPn register must also be enabled in host
		 * mode.
		 */
		U1EP1 = _U1EP1_EPTXEN_MASK
		      | _U1EP1_EPRXEN_MASK
		      | _U1EP1_EPHSHK_MASK;
		g_bulk_toggle = 0;
	} else if (g_dev_type == USB_DEV_KEYBOARD) {
		hid_set_boot_protocol();
		U1EP1 = _U1EP1_EPRXEN_MASK | _U1EP1_EPHSHK_MASK;
		g_hid_toggle = 0;
	}
}

/* ============================================================
 * Post-detach full reset
 * ============================================================ */
static void usb_reset_state(void)
{
	g_dev_type     = USB_DEV_UNKNOWN;
	g_dev_addr     = 0;
	g_bulk_ep      = 0;
	g_hid_ep       = 0;
	g_bulk_toggle  = 0;
	g_hid_toggle   = 0;
	g_ep0_toggle   = 0;
	g_is_low_speed = 0;
	g_ep0_max_pkt  = 8;

	U1IE    = 0;
	U1OTGIE = 0;
	U1EIE   = 0;
	U1CON   = 0;
	U1PWRCbits.USBPWR = 0;
	delay_init_ms(2);

	my_memset((uint8_t *)g_bdt, 0, sizeof(g_bdt));

	usb_init();
}

/* ============================================================
 * Bulk OUT transfer (shares the BDT with EP0; endpoint number
 * is selected via U1TOK)
 * ============================================================ */
UsbResult usb_bulk_write(uint8_t *data, uint16_t len)
{
	uint16_t  offset = 0;
	uint16_t  chunk_len;
	uint8_t   retry_count;
	uint8_t   idx;
	UsbResult result;

	while (offset < len) {
		chunk_len = len - offset;
		if (chunk_len > 64) {
			chunk_len = 64;
		}
		retry_count = 0;

		my_memcpy(g_bulk_buf, data + offset, chunk_len);

		do {
			idx = pick_bdt_out();
			g_bdt[idx].adr  = KVA_TO_PA((uint32_t)g_bulk_buf);
			g_bdt[idx].stat = BDT_UOWN | BDT_DTS
			                | (g_bulk_toggle ? BDT_DATA1 : 0)
			                | BDT_BC(chunk_len);

			token_send(USB_PID_OUT, g_bulk_ep, 0);

			result = wait_trn();
			if (result == USB_ERR_TIMEOUT) {
				return USB_ERR_TIMEOUT;
			}

			result = check_pid(idx);

			if (result == USB_ERR_STALL) {
				return USB_ERR_STALL;
			}

			if (result == USB_ERR_NAK_TIMEOUT) {
				retry_count++;
				if (retry_count >= BULK_RETRY_MAX) {
					return USB_ERR_NAK_TIMEOUT;
				}
				delay_usbms(1);
				if (U1IRbits.DETACHIF) {
					return USB_ERR_TIMEOUT;
				}
			}
		} while (result == USB_ERR_NAK_TIMEOUT);

		g_bulk_toggle ^= 1;
		offset += chunk_len;
	}
	return USB_OK;
}

/* ============================================================
 * Interrupt IN transfer
 * ============================================================ */
typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} HidKbReport;

static int usb_interrupt_in(void)
{
	uint32_t timeout;
	uint8_t  pid;
	uint8_t  idx;

	idx = pick_bdt_in();
	g_bdt[idx].adr  = KVA_TO_PA((uint32_t)g_hid_buf);
	g_bdt[idx].stat = BDT_UOWN | BDT_DTS
	                | (g_hid_toggle ? BDT_DATA1 : 0)
	                | BDT_BC(8);

	token_send(USB_PID_IN, g_hid_ep, 0);

	/*
	 * RETRYDIS is set, so a NAK raises TRNIF immediately.
	 * A short timeout is sufficient.
	 */
	timeout = 10000;
	while (!U1IRbits.TRNIF) {
		if (--timeout == 0) {
			return -1;
		}
		if (U1IRbits.DETACHIF) {
			return -1;
		}
		poll_call();
	}
	U1IR = _U1IR_TRNIF_MASK;

	pid = (g_bdt[idx].stat >> 2) & 0x0F;
	if (pid == 0x0A) {
		return 0;  /* NAK - no data available */
	}

	g_hid_toggle ^= 1;
	return 1;
}

/* ============================================================
 * Keycode to ASCII (US layout)
 * ============================================================ */
static const char keycode_to_ascii[58] = {
	0,    0,    0,    0,   'a', 'b', 'c', 'd',
	'e',  'f',  'g',  'h', 'i', 'j', 'k', 'l',
	'm',  'n',  'o',  'p', 'q', 'r', 's', 't',
	'u',  'v',  'w',  'x', 'y', 'z', '1', '2',
	'3',  '4',  '5',  '6', '7', '8', '9', '0',
	'\n', 0,   '\b', '\t', ' ', '-', '=', '[',
	']',  '\\', 0,    ';', '\'', '`', ',', '.',
	'/',  0,
};

static char keycode_to_char(uint8_t keycode, uint8_t modifier)
{
	char c;

	if (keycode >= sizeof(keycode_to_ascii)) {
		return 0;
	}
	c = keycode_to_ascii[keycode];
	if (c >= 'a' && c <= 'z') {
		if (modifier & 0x22) {
			c -= 0x20;
		}
	}
	return c;
}

/* ============================================================
 * Main
 * ============================================================ */
int main(void)
{
	HidKbReport  prev;
	HidKbReport *rep;
	int          ret;
	int          i;
	int          j;
	int          already;
	uint8_t      kc;
	char         c;

	usb_init();

	while (1) {
		usb_wait_attach_and_reset();

		usb_enumerate();

		if (g_dev_type == USB_DEV_PRINTER) {
			/*
			 * Application hook: send data to the printer, e.g.
			 *   usb_bulk_write((uint8_t *)print_data,
			 *                  sizeof(print_data));
			 */
			while (!usb_is_detached()) {
				poll_call();
			}
		} else if (g_dev_type == USB_DEV_KEYBOARD) {
			prev.modifier = 0;
			prev.reserved = 0;
			for (i = 0; i < 6; i++) {
				prev.keycode[i] = 0;
			}

			while (!usb_is_detached()) {
				ret = usb_interrupt_in();
				delay_usbms(10);

				if (ret <= 0) {
					continue;
				}

				rep = (HidKbReport *)g_hid_buf;

				for (i = 0; i < 6; i++) {
					kc = rep->keycode[i];
					if (kc == 0) {
						continue;
					}

					already = 0;
					for (j = 0; j < 6; j++) {
						if (prev.keycode[j] == kc) {
							already = 1;
							break;
						}
					}
					if (already) {
						continue;
					}

					c = keycode_to_char(kc, rep->modifier);
					if (c) {
						/*
						 * Application hook: forward
						 * to UART, ring buffer, etc.
						 */
						(void)c;
					}
				}
				prev = *rep;
			}
		} else {
			/* Unknown class: idle until detach */
			while (!usb_is_detached()) {
				poll_call();
			}
		}

		usb_reset_state();
		delay_init_ms(200);
	}

	return 0;
}
