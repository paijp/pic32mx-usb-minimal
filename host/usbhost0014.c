/*
 * usbhost0014.c - PIC32MX270F256B USB host minimal demo
 *
 * A single-file, no-interrupt, no-library USB host for
 * PIC32MX270F256B. Enumerates an attached full-speed USB printer
 * (class 0x07) or a low-speed HID boot keyboard (class 0x03) and
 * dispatches to a per-class handler. Any other device is accepted
 * onto the bus but left idle until detach.
 *
 * Clocking: 4 MHz external crystal
 *   SYSCLK: 4 MHz / FPLLIDIV(2) * FPLLMUL(20) / FPLLODIV(1) = 40 MHz
 *   USB   : 4 MHz / UPLLIDIV(1) * 12                        = 48 MHz
 *
 * Design notes:
 *   - Only four BDT entries are used (IN/OUT x EVEN/ODD). In host
 *     mode the endpoint is selected per token via U1TOK, so the BDT
 *     is not indexed by endpoint number.
 *   - U1EP0 and U1ADDR are re-applied before every token.
 *   - RETRYDIS is cleared for control transfers (hardware retries)
 *     and set for bulk/interrupt (software retry on NAK).
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ---------------------------------------------------------------------------
 *
 * This code was written from scratch but draws on the following reference
 * implementations for the PIC32MX USB peripheral, in particular the
 * register initialisation order, bus reset timing, ping-pong management,
 * and the requirement to re-apply U1EP0/U1ADDR before each token issued
 * in host mode:
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
 * knowledge was reused.
 */
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>

/* ============================================================
 * コンフィグ
 * ============================================================ */
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL  = MUL_20
#pragma config FPLLODIV = DIV_1
#pragma config FNOSC    = PRIPLL
#pragma config POSCMOD  = XT
#pragma config FSOSCEN  = OFF
#pragma config UPLLIDIV = DIV_1
#pragma config UPLLEN   = ON
#pragma config FPBDIV   = DIV_1
#pragma config FWDTEN   = OFF
#pragma config JTAGEN   = OFF

#define SYS_CLK_HZ        40000000UL
#define CORE_TICK_PER_MS  (SYS_CLK_HZ / 2 / 1000)

/* ============================================================
 * 定数
 * ============================================================ */
#define USB_PID_SETUP  0xD
#define USB_PID_IN     0x9
#define USB_PID_OUT    0x1

#define KVA_TO_PA(v)  ((uint32_t)(v) & 0x1FFFFFFF)

/* ============================================================
 * BDT
 * MLA 方式: 4 エントリ (IN Even, IN Odd, OUT Even, OUT Odd)
 * エンドポイント番号は U1TOK で指定するので BDT には持たない
 * ============================================================ */
typedef struct {
	uint32_t STAT;
	uint32_t ADR;
} BDT_ENTRY;

#define BDT_UOWN   (1u << 7)
#define BDT_DATA1  (1u << 6)
#define BDT_DTS    (1u << 3)
#define BDT_BC(n)  (((uint32_t)(n) & 0x3FF) << 16)

#define BDT_IN_EVEN    0
#define BDT_IN_ODD     1
#define BDT_OUT_EVEN   2
#define BDT_OUT_ODD    3
#define BDT_SIZE       4

static BDT_ENTRY __attribute__((aligned(512))) g_bdt[BDT_SIZE];

/* ============================================================
 * バッファ
 * ============================================================ */
static uint8_t g_ep0RxBuf[64];
static uint8_t g_ep0TxBuf[64];
static uint8_t g_bulkBuf[64];
static uint8_t g_hidBuf[8];

/* ============================================================
 * 状態
 * ============================================================ */
typedef enum {
	USB_DEV_UNKNOWN,
	USB_DEV_PRINTER,
	USB_DEV_KEYBOARD
} USB_DEV_TYPE;

typedef enum {
	USB_OK = 0,
	USB_ERR_NAK_TIMEOUT,
	USB_ERR_STALL,
	USB_ERR_TIMEOUT
} USB_RESULT;

#define BULK_RETRY_MAX  100

static USB_DEV_TYPE g_devType    = USB_DEV_UNKNOWN;
static uint8_t      g_devAddr    = 0;   /* 現在のデバイスアドレス */
static uint8_t      g_bulkEP     = 0;
static uint8_t      g_hidEP      = 0;
static uint8_t      g_bulkToggle = 0;
static uint8_t      g_hidToggle  = 0;
static uint8_t      g_ep0Toggle  = 0;
static uint8_t      g_isLowSpeed = 0;
static uint8_t      g_ep0MaxPkt  = 8;   /* EP0 max packet size: 初期は安全側 8 */

/* Ping-Pong 管理 (MLA bfPingPongIn/bfPingPongOut 相当) */
static uint8_t g_ppIn  = 0;   /* 0=EVEN が次, 1=ODD が次 */
static uint8_t g_ppOut = 0;

/* ============================================================
 * ユーティリティ
 * ============================================================ */
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
		;
	}
}

static void delay_usbms(uint32_t ms)
{
	while (ms--) {
		U1OTGIR = _U1OTGIR_T1MSECIF_MASK;
		while (!(U1OTGIR & _U1OTGIR_T1MSECIF_MASK)) {
			;
		}
	}
}

/* ============================================================
 * Ping-Pong 用 BDT 選択
 * ============================================================ */
static uint8_t pickBDT_IN(void)
{
	uint8_t idx = g_ppIn ? BDT_IN_ODD : BDT_IN_EVEN;
	g_ppIn ^= 1;
	return idx;
}

static uint8_t pickBDT_OUT(void)
{
	uint8_t idx = g_ppOut ? BDT_OUT_ODD : BDT_OUT_EVEN;
	g_ppOut ^= 1;
	return idx;
}

static void resetPingPong(void)
{
	g_ppIn  = 0;
	g_ppOut = 0;
}

/*
 * U1EP0 の bit6 は RETRYDIS (Retry Disable)
 * xc.h 版によっては _U1EP0_RETRYDIS_MASK が定義されている
 * ない場合に備えて直接値で定義 */
#ifndef _U1EP0_RETRYDIS_MASK
#define EP_RETRYDIS   0x40
#else
#define EP_RETRYDIS   _U1EP0_RETRYDIS_MASK
#endif

/* ============================================================
 * 転送プリミティブ
 * ============================================================ */

/*
 * token_send - MLA _USB_SendToken 相当
 *
 * トークン送出のたびに U1EP0 と U1ADDR を再設定してから U1TOK を書く。
 * これにより、Low-Speed/Full-Speed の切り替えや、途中で壊れた設定から
 * 確実に復帰できる。
 *
 * pid: USB_PID_SETUP/IN/OUT
 * ep : エンドポイント番号 (0..15)
 * isControl: 1 なら EPCONDIS を落として SETUP 許可
 */
static void token_send(uint8_t pid, uint8_t ep, uint8_t isControl)
{
	uint8_t epVal;
	uint8_t addrVal;

	/* Control: RETRYDIS=0 (HW 自動リトライ、NAK は SIE が吸収)
	 * Int/Bulk: RETRYDIS=1 (NAK 即時検出でソフトリトライ) */
	epVal = _U1EP0_EPRXEN_MASK
	      | _U1EP0_EPTXEN_MASK
	      | _U1EP0_EPHSHK_MASK;
	if (!isControl) {
		epVal |= _U1EP0_EPCONDIS_MASK;
		epVal |= EP_RETRYDIS;
	}
	if (g_isLowSpeed) {
		epVal |= _U1EP0_LSPD_MASK;
	}
	U1EP0 = epVal;

	/* U1ADDR: Low-Speed なら LSEN=1 + アドレス */
	addrVal = g_devAddr;
	if (g_isLowSpeed) {
		addrVal |= 0x80;
	}
	U1ADDR = addrVal;

	/* トークン送出 */
	U1TOK = (pid << 4) | (ep & 0x0F);
}

static USB_RESULT waitTRN(void)
{
	uint32_t timeout = 500000;

	while (!U1IRbits.TRNIF) {
		if (--timeout == 0) {
			return USB_ERR_TIMEOUT;
		}
		if (U1IRbits.DETACHIF) {
			return USB_ERR_TIMEOUT;
		}
	}
	U1IR = _U1IR_TRNIF_MASK;
	__asm__("nop");
	__asm__("nop");
	return USB_OK;
}

static USB_RESULT checkPID(uint8_t bdtIdx)
{
	uint8_t pid = (g_bdt[bdtIdx].STAT >> 2) & 0x0F;

	if (pid == 0x0A) {
		return USB_ERR_NAK_TIMEOUT;
	}
	if (pid == 0x0E) {
		return USB_ERR_STALL;
	}
	return USB_OK;
}

/* ============================================================
 * USB 初期化
 * ============================================================ */
void USB_Init(void)
{
	uint32_t pa;

	my_memset((uint8_t *)g_bdt, 0, sizeof(g_bdt));
	resetPingPong();

	/* D+/D- ピン (RB10/RB11) の GPIO/アナログ機能を無効化 */
	ANSELB &= ~((1 << 10) | (1 << 11));
	CNPUB   = 0;   /* ブートローダーが残した弱プルアップ対策 */
	CNPDB   = 0;

	/* 割り込み無効化 & クリア */
	U1IE    = 0;
	U1IR    = 0xFF;
	U1OTGIE = 0;
	U1OTGIR = 0x7D;
	U1EIE   = 0;
	U1EIR   = 0xFF;

	/* BDT アドレス */
	pa = KVA_TO_PA((uint32_t)g_bdt);
	U1BDTP1 = (pa >> 8)  & 0xFF;
	U1BDTP2 = (pa >> 16) & 0xFF;
	U1BDTP3 = (pa >> 24) & 0xFF;

	/* HOSTEN + PPBRST シーケンス */
	U1CON = _U1CON_HOSTEN_MASK;
	U1CON = _U1CON_HOSTEN_MASK | _U1CON_PPBRST_MASK;
	U1CON = _U1CON_HOSTEN_MASK;

	/* D+/D- プルダウン + VBUS ON (別代入) */
	U1OTGCON = _U1OTGCON_DPPULDWN_MASK | _U1OTGCON_DMPULDWN_MASK;
	U1OTGCON |= _U1OTGCON_VBUSON_MASK;

	/* Full Ping-Pong */
	U1CNFG1 = 0x02;

	U1ADDR = 0;
	U1EP0  = _U1EP0_EPCONDIS_MASK
	       | _U1EP0_EPRXEN_MASK
	       | _U1EP0_EPTXEN_MASK
	       | _U1EP0_EPHSHK_MASK;
	/* RETRYDIS は token_send で Control/Int/Bulk ごとに設定 */
	U1SOF  = 0x4A;

	/* USB モジュール ON (最後) */
	U1PWRCbits.USBPWR = 1;
	delay_init_ms(10);
}

/* ============================================================
 * Attach 待ち & バスリセット
 * ============================================================ */
void USB_WaitAttachAndReset(void)
{
	while (!U1IRbits.ATTACHIF) {
		;
	}
	U1IR = _U1IR_ATTACHIF_MASK;

	delay_usbms(200);

	if (!U1CONbits.JSTATE) {
		g_isLowSpeed = 1;
	} else {
		g_isLowSpeed = 0;
	}
	/* LSPD/LSEN は token_send が毎回設定する */

	/* Ping-Pong 再リセット (HW と SW 両方) */
	U1CONbits.PPBRST = 1;
	U1CONbits.PPBRST = 0;
	resetPingPong();

	/* バスリセット: 50ms アサート */
	U1CONbits.USBRST = 1;
	delay_usbms(50);
	U1CONbits.USBRST = 0;

	/* MLA 順: USBRST 解除直後に SOFEN=1 (ハブ代替の keep-alive) */
	U1CONbits.SOFEN = 1;

	/* Reset recovery: Low-Speed デバイスでは 100ms 必要 */
	delay_usbms(100);

	/* アドレスリセット (LSEN は g_isLowSpeed に応じて保持) */
	g_devAddr = 0;
	g_ep0Toggle = 0;

	U1IR = _U1IR_DETACHIF_MASK;
}

/* ============================================================
 * Control 転送プリミティブ (Ping-Pong 管理対応)
 * ============================================================ */
static void ctrl_setup(uint8_t *pkt8)
{
	uint8_t idx = pickBDT_OUT();

	my_memcpy(g_ep0TxBuf, pkt8, 8);

	g_bdt[idx].ADR  = KVA_TO_PA((uint32_t)g_ep0TxBuf);
	g_bdt[idx].STAT = BDT_UOWN | BDT_DTS | BDT_BC(8);

	token_send(USB_PID_SETUP, 0, 1);   /* isControl=1 */
	waitTRN();
	g_ep0Toggle = 1;
}

static uint16_t ctrl_in(uint8_t *data, uint16_t maxLen)
{
	uint16_t total    = 0;
	uint16_t chunkLen;
	uint16_t rxLen;
	uint8_t  idx;

	while (total < maxLen) {
		/* chunkLen は EP0 max packet size 以下 */
		chunkLen = maxLen - total;
		if (chunkLen > g_ep0MaxPkt) {
			chunkLen = g_ep0MaxPkt;
		}

		idx = pickBDT_IN();
		g_bdt[idx].ADR  = KVA_TO_PA((uint32_t)g_ep0RxBuf);
		g_bdt[idx].STAT = BDT_UOWN | BDT_DTS
		                | (g_ep0Toggle ? BDT_DATA1 : 0)
		                | BDT_BC(chunkLen);

		token_send(USB_PID_IN, 0, 1);
		if (waitTRN() != USB_OK) {
			break;
		}

		rxLen = (g_bdt[idx].STAT >> 16) & 0x3FF;
		my_memcpy(data + total, g_ep0RxBuf, rxLen);
		total += rxLen;
		g_ep0Toggle ^= 1;

		/* Short packet (要求より少ない) で終了 */
		if (rxLen < chunkLen) {
			break;
		}
	}
	return total;
}

static void ctrl_out_zlp(void)
{
	uint8_t idx = pickBDT_OUT();

	g_bdt[idx].ADR  = KVA_TO_PA((uint32_t)g_ep0TxBuf);
	g_bdt[idx].STAT = BDT_UOWN | BDT_DTS | BDT_DATA1 | BDT_BC(0);

	token_send(USB_PID_OUT, 0, 1);
	waitTRN();
}

static void ctrl_in_zlp(void)
{
	uint8_t idx = pickBDT_IN();

	g_bdt[idx].ADR  = KVA_TO_PA((uint32_t)g_ep0RxBuf);
	g_bdt[idx].STAT = BDT_UOWN | BDT_DTS | BDT_DATA1 | BDT_BC(0);

	token_send(USB_PID_IN, 0, 1);
	waitTRN();
}

/* ============================================================
 * 標準リクエスト
 * ============================================================ */
static uint16_t USB_GetDescriptor(uint8_t type, uint8_t idx,
                                   uint8_t *buf, uint16_t len)
{
	uint8_t  setup[8];
	uint16_t rxLen;

	setup[0] = 0x80; setup[1] = 0x06;
	setup[2] = idx;  setup[3] = type;
	setup[4] = 0x00; setup[5] = 0x00;
	setup[6] = (uint8_t)(len & 0xFF);
	setup[7] = (uint8_t)(len >> 8);

	ctrl_setup(setup);
	rxLen = ctrl_in(buf, len);
	ctrl_out_zlp();
	return rxLen;
}

static void USB_SetAddress(uint8_t addr)
{
	uint8_t setup[8] = {
		0x00, 0x05, addr, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	ctrl_setup(setup);
	ctrl_in_zlp();
	delay_usbms(2);

	/* 以降の token_send で使われる */
	g_devAddr = addr;
}

static void USB_SetConfiguration(uint8_t cfgVal)
{
	uint8_t setup[8] = {
		0x00, 0x09, cfgVal, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	ctrl_setup(setup);
	ctrl_in_zlp();
}

static void USB_SetInterface(uint8_t ifNum, uint8_t altNum)
{
	uint8_t setup[8] = {
		0x01, 0x0B, altNum, 0x00, ifNum, 0x00, 0x00, 0x00
	};

	ctrl_setup(setup);
	ctrl_in_zlp();
}

/* ============================================================
 * ConfigDescriptor 解析
 * ============================================================ */
static USB_DEV_TYPE parseConfigDesc(uint8_t *buf, uint16_t len)
{
	USB_DEV_TYPE devType = USB_DEV_UNKNOWN;
	uint16_t i = 0;
	uint8_t  descLen, descType;
	uint8_t  cls, subcls, proto;
	uint8_t  altNum;
	uint8_t  epAddr, attr;
	uint8_t  inTarget = 0;

	g_bulkEP = 0;
	g_hidEP  = 0;

	while (i < len) {
		descLen  = buf[i];
		descType = buf[i + 1];

		if (descType == 0x04) {   /* Interface */
			altNum = buf[i + 3];
			cls    = buf[i + 5];
			subcls = buf[i + 6];
			proto  = buf[i + 7];

			/* 最初に見つけた対象 Interface (alt=0) のみ採用 */
			if (altNum == 0 && devType == USB_DEV_UNKNOWN) {
				if (cls == 0x07) {
					devType = USB_DEV_PRINTER;
					inTarget = 1;
				} else if (cls == 0x03 && subcls == 0x01 && proto == 0x01) {
					devType = USB_DEV_KEYBOARD;
					inTarget = 1;
				} else {
					inTarget = 0;
				}
			} else {
				inTarget = 0;
			}
		}

		if (descType == 0x05 && inTarget) {  /* Endpoint */
			epAddr = buf[i + 2];
			attr   = buf[i + 3];

			if (devType == USB_DEV_PRINTER) {
				if ((epAddr & 0x80) == 0x00 && (attr & 0x03) == 0x02) {
					if (g_bulkEP == 0) {
						g_bulkEP = epAddr & 0x0F;
					}
				}
			} else if (devType == USB_DEV_KEYBOARD) {
				if ((epAddr & 0x80) == 0x80 && (attr & 0x03) == 0x03) {
					if (g_hidEP == 0) {
						g_hidEP = epAddr & 0x0F;
					}
				}
			}
		}

		if (descLen == 0) {
			break;
		}
		i += descLen;
	}
	return devType;
}

static void HID_SetBootProtocol(void)
{
	uint8_t setup[8] = { 0x21, 0x0B, 0, 0, 0, 0, 0, 0 };

	ctrl_setup(setup);
	ctrl_in_zlp();
}

/* ============================================================
 * Enumeration
 * ============================================================ */
void USB_Enumerate(void)
{
	uint8_t  buf[256];
	uint16_t totalLen;

	my_memset(buf, 0, sizeof(buf));
	g_ep0MaxPkt = 8;   /* 最初の GET_DESC(dev, 8) は 8 バイトで実行 */

	USB_GetDescriptor(0x01, 0, buf, 8);

	/* buf[7] = bMaxPacketSize0 */
	g_ep0MaxPkt = buf[7];
	if (g_ep0MaxPkt == 0 || g_ep0MaxPkt > 64) {
		g_ep0MaxPkt = 8;
	}

	USB_SetAddress(0x01);

	USB_GetDescriptor(0x01, 0, buf, 18);

	USB_GetDescriptor(0x02, 0, buf, 9);

	totalLen = buf[2] | ((uint16_t)buf[3] << 8);
	USB_GetDescriptor(0x02, 0, buf, totalLen);

	g_devType = parseConfigDesc(buf, totalLen);

	USB_SetConfiguration(buf[5]);

	if (g_devType == USB_DEV_PRINTER) {
		USB_SetInterface(0, 0);

		/* EP1 レジスタを Bulk 用に設定 (U1TOK で EP 番号指定するが、
		 * ホストモードでは EPn レジスタも有効化が必要) */
		U1EP1 = _U1EP1_EPTXEN_MASK
		      | _U1EP1_EPRXEN_MASK
		      | _U1EP1_EPHSHK_MASK;
		g_bulkToggle = 0;
	} else if (g_devType == USB_DEV_KEYBOARD) {
		HID_SetBootProtocol();
		U1EP1 = _U1EP1_EPRXEN_MASK | _U1EP1_EPHSHK_MASK;
		g_hidToggle = 0;
	}
}

/* ============================================================
 * Detach 後リセット
 * ============================================================ */
static void USB_ResetState(void)
{
	g_devType    = USB_DEV_UNKNOWN;
	g_devAddr    = 0;
	g_bulkEP     = 0;
	g_hidEP      = 0;
	g_bulkToggle = 0;
	g_hidToggle  = 0;
	g_ep0Toggle  = 0;
	g_isLowSpeed = 0;
	g_ep0MaxPkt  = 8;

	U1IE    = 0;
	U1OTGIE = 0;
	U1EIE   = 0;
	U1CON   = 0;
	U1PWRCbits.USBPWR = 0;
	delay_init_ms(2);

	my_memset((uint8_t *)g_bdt, 0, sizeof(g_bdt));

	USB_Init();
}


/* ============================================================
 * Bulk OUT 転送 (EP0 と同じ BDT を共用、EP 番号は U1TOK で指定)
 * ============================================================ */
USB_RESULT USB_BulkWrite(uint8_t *data, uint16_t len)
{
	uint16_t   offset     = 0;
	uint16_t   chunkLen;
	uint8_t    retryCount;
	uint8_t    idx;
	USB_RESULT result;

	while (offset < len) {
		chunkLen = len - offset;
		if (chunkLen > 64) {
			chunkLen = 64;
		}
		retryCount = 0;

		my_memcpy(g_bulkBuf, data + offset, chunkLen);

		do {
			idx = pickBDT_OUT();
			g_bdt[idx].ADR  = KVA_TO_PA((uint32_t)g_bulkBuf);
			g_bdt[idx].STAT = BDT_UOWN | BDT_DTS
			                | (g_bulkToggle ? BDT_DATA1 : 0)
			                | BDT_BC(chunkLen);

			token_send(USB_PID_OUT, g_bulkEP, 0);

			result = waitTRN();
			if (result == USB_ERR_TIMEOUT) {
				return USB_ERR_TIMEOUT;
			}

			result = checkPID(idx);

			if (result == USB_ERR_STALL) {
				return USB_ERR_STALL;
			}

			if (result == USB_ERR_NAK_TIMEOUT) {
				retryCount++;
				if (retryCount >= BULK_RETRY_MAX) {
					return USB_ERR_NAK_TIMEOUT;
				}
				delay_usbms(1);
				if (U1IRbits.DETACHIF) {
					return USB_ERR_TIMEOUT;
				}
			}

		} while (result == USB_ERR_NAK_TIMEOUT);

		g_bulkToggle ^= 1;
		offset += chunkLen;
	}
	return USB_OK;
}

/* ============================================================
 * Interrupt IN 転送
 * ============================================================ */
typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} HID_KB_REPORT;

static int USB_InterruptIN(void)
{
	uint32_t timeout;
	uint8_t  pid;
	uint8_t  idx;

	idx = pickBDT_IN();
	g_bdt[idx].ADR  = KVA_TO_PA((uint32_t)g_hidBuf);
	g_bdt[idx].STAT = BDT_UOWN | BDT_DTS
	                | (g_hidToggle ? BDT_DATA1 : 0)
	                | BDT_BC(8);

	token_send(USB_PID_IN, g_hidEP, 0);

	/* RETRYDIS が効いていれば NAK でも TRNIF がすぐ立つので、
	 * タイムアウトは短くてよい */
	timeout = 10000;
	while (!U1IRbits.TRNIF) {
		if (--timeout == 0) {
			return -1;
		}
		if (U1IRbits.DETACHIF) {
			return -1;
		}
	}
	U1IR = _U1IR_TRNIF_MASK;

	pid = (g_bdt[idx].STAT >> 2) & 0x0F;
	if (pid == 0x0A) {
		return 0;
	}

	g_hidToggle ^= 1;
	return 1;
}

/* ============================================================
 * キーコード → ASCII (US配列)
 * ============================================================ */
static const char keycode2ascii[58] = {
	0,    0,    0,    0,   'a', 'b', 'c', 'd',
	'e',  'f',  'g',  'h', 'i', 'j', 'k', 'l',
	'm',  'n',  'o',  'p', 'q', 'r', 's', 't',
	'u',  'v',  'w',  'x', 'y', 'z', '1', '2',
	'3',  '4',  '5',  '6', '7', '8', '9', '0',
	'\n', 0,   '\b', '\t', ' ', '-', '=', '[',
	']',  '\\', 0,    ';', '\'', '`', ',', '.',
	'/',  0,
};

static char KeycodeToAscii(uint8_t keycode, uint8_t modifier)
{
	char c;

	if (keycode >= sizeof(keycode2ascii)) {
		return 0;
	}
	c = keycode2ascii[keycode];
	if (c >= 'a' && c <= 'z') {
		if (modifier & 0x22) {
			c -= 0x20;
		}
	}
	return c;
}

/* ============================================================
 * メイン
 * ============================================================ */
int main(void)
{
	HID_KB_REPORT   prev;
	HID_KB_REPORT  *rep;
	int             ret, i, j, already;
	uint8_t         kc;
	char            c;


	USB_Init();

	while (1) {
		USB_WaitAttachAndReset();

		USB_Enumerate();

		if (g_devType == USB_DEV_PRINTER) {
			while (!U1IRbits.DETACHIF) {
				;
			}
		} else if (g_devType == USB_DEV_KEYBOARD) {
			prev.modifier = 0;
			prev.reserved = 0;
			for (i = 0; i < 6; i++) {
				prev.keycode[i] = 0;
			}

			while (!U1IRbits.DETACHIF) {
				ret = USB_InterruptIN();
				delay_usbms(10);   /* HID polling interval */

				if (ret <= 0) {
					continue;
				}

				rep = (HID_KB_REPORT *)g_hidBuf;

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

					c = KeycodeToAscii(kc, rep->modifier);
					if (c) {
						/* ここでアプリ処理 (例: UART送信や
						 * キューへの登録など) を行う */
						(void)c;
					}
				}
				prev = *rep;
			}
		} else {
			/* Printer でも Keyboard でもない: 何もせず Detach 待ち */
			while (!U1IRbits.DETACHIF) {
				;
			}
		}

		USB_ResetState();
		delay_init_ms(200);
	}

	return 0;
}
