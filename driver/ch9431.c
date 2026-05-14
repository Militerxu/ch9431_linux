/*
 * CAN bus driver for CH9431 CAN Controller CH9431 with SPI Interface
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * System required:
 * Kernel version beyond 4.0.x
 *
 * V1.0 - initial version.
 * V1.1 - add support for kernel 3.11.x.
 * V1.2 - re-implement spi TX/RX data transfer logic for bulk transfers.
 * V1.3	- cancel interrupt delay to prevent duplicate reception.
 * V1.3.1 - change irq trigger type from edge to level-low.
 *        - fixup receive 0 len CAN frame issue.
 * V1.3.2 - fix issue where chip fails to enter config mode
 * 			after CAN interface down/up.
 * V1.4 - ddd read bulk mode to improve receive efficiency
 * V1.4.1 - optimize transmit flow to improve sending efficiency
 * V1.4.2 - add SPI bulk read support for improved receive throughput
 */

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#define DRVNAME_CH9431 "ch9431"
#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC \
	"CAN bus driver for CH9431 CAN Controller with SPI Interface"
#define VERSION_DESC "V1.4.2 On 2026.04"

#define CH9431_DEBUG 0
#define CH9431_DBG_FLOW "CH9431_FLOW"
#define CH9431_DBG_IRQ "CH9431_IRQ"
#define CH9431_DBG_TX "CH9431_TX"
#define CH9431_DBG_RX "CH9431_RX"
#define CH9431_DBG_REG "CH9431_REG"
#define CH9431_DBG_POWER "CH9431_POWER"
#define CH9431_DBG_RATELIMIT_MS 1000

#define USE_IRQ_FROM_DTS
#define GPIO_NUMBER 0
//#undef USE_IRQ_FROM_DTS

#define REG_LABEL(REG) { #REG, REG }

#define CH9431_CLK_FREQ 20000000
#define CH9431_TXBD_CMD_LEN 1
#define CH9431_TX_BUF_NUM 3
#define CH9431_TX_MASK ((1 << CH9431_TX_BUF_NUM) - 1)
#define CAN_FRAME_HEADER_LEN 5
#define CH9431_FRAME_SIZE 13
#define CH9431_MAX_FRAMES 5
#define CH9431_BULK_SIZE (1 + CH9431_FRAME_SIZE * CH9431_MAX_FRAMES)
#define CAN_FRAME_TX_CMD_LEN 6
#define CAN_FRAME_MAX_DATA_LEN 8
#define SPI_TRANSFER_BUF_LEN \
	(CAN_FRAME_TX_CMD_LEN + CAN_FRAME_MAX_DATA_LEN)
#define CH9431_SPI_TX_BUF_LEN (SPI_TRANSFER_BUF_LEN + CH9431_TXBD_CMD_LEN)
#define CH9431_SPI_RX_BUF_LEN CH9431_BULK_SIZE

/* SPI Delay */
#define WAIT_DATA_US (1)
#define BULK_WAIT_DATA_US (2)
#define CLR_INTR_US (5)
#define OST_DELAY_MS (5)
#define RST_DELAY_MS (16)

/* CH9431 SPI commands */
#define CMD_CAN_WRITE 0x02
#define CMD_CAN_READ 0x03
#define CMD_CAN_BIT_MODIFY 0x05
#define CMD_CAN_RD_STATUS 0xA0
#define CMD_CAN_RX_STATUS 0xB0
#define CMD_CAN_RESET 0xC0
#define CMD_CAN_RTS 0x80
#define CMD_CAN_LOAD_TX 0X40
#define CMD_CAN_RD_RX_BUFF 0x90
#define CMD_CAN_TX_SEL 0X4F
#define CMD_CAN_RX0_5PACK(n) (((n) == 0) ? 0x20 : 0x21)
#define INSTRUCTION_QREAD_RXBD(n) (((n) == 0) ? 0x22 : 0x23)
#define INSTRUCTION_READ_RXBS(n) (((n) == 0) ? 0x90 : 0x94)
#define INSTRUCTION_READ_RXBD(n) (((n) == 0) ? 0x92 : 0x96)
#define INSTRUCTION_LOAD_TXBS(n) (0x40 + 2 * (n))
#define INSTRUCTION_LOAD_TXBD(n) (0x41 + 2 * (n))
#define INSTRUCTION_RTS(n) (0x80 | ((n) & 0x07))

/* CH9431 configuration registers */
#define CH9431_CTRL 0x0F
#define CH9431_CTRL_REQOP_MASK 0xE0
#define CH9431_CTRL_REQOP_NORMAL 0x00
#define CH9431_CTRL_REQOP_CONFIG 0x20
#define CH9431_CTRL_REQOP_LISTEN 0x40
#define CH9431_CTRL_REQOP_LOOPBACK 0x60
#define CH9431_CTRL_REQOP_SLEEP_LIGHT 0x80
#define CH9431_CTRL_REQOP_SLEEP_DEEP 0xA0
#define CH9431_CTRL_ABAT (1 << 4)
#define CH9431_CTRL_OSM (1 << 3)
#define CH9431_CTRL_CLKEN (1 << 2)
#define CH9431_CTRL_CLKPRE_20 0x00
#define CH9431_CTRL_CLKPRE_10 0x01
#define CH9431_CTRL_CLKPRE_08 0x02
#define CH9431_CTRL_CLKPRE_04 0x03
#define CH9431_STAT 0x0E
#define CH9431_STAT_OPMOD_NORMAL 0x00
#define CH9431_STAT_OPMOD_CONFIG 0x20
#define CH9431_STAT_OPMOD_LISTEN 0x40
#define CH9431_STAT_OPMOD_LOOPBACK 0x60
#define CH9431_STAT_OPMOD_UPDATE 0xC0
#define CH9431_STAT_ICOD_NONE 0x00
#define CH9431_STAT_ICOD_ERROR 0x02
#define CH9431_STAT_ICOD_WAKEUP 0x04
#define CH9431_STAT_ICOD_TXB0 0x06
#define CH9431_STAT_ICOD_TXB1 0x08
#define CH9431_STAT_ICOD_TXB2 0x0A
#define CH9431_STAT_ICOD_RXB0 0x0C
#define CH9431_STAT_ICOD_RXB1 0x0E
#define RXIPCTRL 0x0C
#define TXRTSCTRL 0x0D

#define TEC 0x1C
#define REC 0x1D

#define BTIMER3 0x28
#define BTIMER2 0x29
#define BTIMER2_SJW_SHIFT 4
#define BTIMER1 0x2A

#define CH9431_INTE 0x2B
#define CH9431_INTE_MERRE 0x80
#define CH9431_INTE_WAKIE 0x40
#define CH9431_INTE_ERRIE 0x20
#define CH9431_INTE_TX2IE 0x10
#define CH9431_INTE_TX1IE 0x08
#define CH9431_INTE_TX0IE 0x04
#define CH9431_INTE_RX1IE 0x02
#define CH9431_INTE_RX0IE 0x01
#define CH9431_INTF 0x2C
#define CH9431_INTF_MERRF 0x80
#define CH9431_INTF_WAKIF 0x40
#define CH9431_INTF_ERRIF 0x20
#define CH9431_INTF_TX2IF 0x10
#define CH9431_INTF_TX1IF 0x08
#define CH9431_INTF_TX0IF 0x04
#define CH9431_INTF_RX1IF 0x02
#define CH9431_INTF_RX0IF 0x01
#define CH9431_INTF_RX (CH9431_INTF_RX0IF | CH9431_INTF_RX1IF)
#define CH9431_INTF_TX \
	(CH9431_INTF_TX0IF | CH9431_INTF_TX1IF | CH9431_INTF_TX2IF)
#define CH9431_INTF_ERR (CH9431_INTF_ERRIF)
#define CH9431_EFLAG 0x2D
#define CH9431_EFLAG_RX1OVR (1 << 7)
#define CH9431_EFLAG_RX0OVR (1 << 6)
#define CH9431_EFLAG_TXBO (1 << 5)
#define CH9431_EFLAG_TXEP (1 << 4)
#define CH9431_EFLAG_RXEP (1 << 3)
#define CH9431_EFLAG_TXWAR (1 << 2)
#define CH9431_EFLAG_RXWAR (1 << 1)
#define CH9431_EFLAG_EWARN (1 << 0)

/*  CH9431 receive filters */
#define RXFAID(n) (((n) * 4) + 0x00)

#define RXF0SIDL 0x00
#define RXF0SIDH 0x01
#define RXF0EIDL 0x02
#define RXF0EIDH 0x03
#define RXFASIDL 0x23
#define RXFASIDH 0x48
#define RXFBSIDH 0x58
#define RXFAEIDL 0xD1
#define RXFAEIDH 0x48

#define RXF1SIDL 0x04
#define RXF1SIDH 0x05
#define RXF1EIDL 0x06
#define RXF1EIDH 0x07

#define RXF2SIDL 0x08
#define RXF2SIDH 0x09
#define RXF2EIDL 0x0A
#define RXF2EIDH 0x0B

#define RXFBID(n) (((n - 3) * 4) + 0x10)
#define RXF3SIDL 0x10
#define RXF3SIDH 0x11
#define RXF3EIDL 0x12
#define RXF3EIDH 0x13

#define RXF4SIDL 0x14
#define RXF4SIDH 0x15
#define RXF4EIDL 0x16
#define RXF4EIDH 0x17

#define RXF5SIDL 0x18
#define RXF5SIDH 0x19
#define RXF5EIDL 0x1A
#define RXF5EIDH 0x1B

/* Receive masks */
#define RXMID(n) (((n) * 4) + 0x20)
#define RXM0SIDL 0x20
#define RXM0SIDH 0x21
#define RXM0EIDL 0x22
#define RXM0EIDH 0x23

#define RXM1SIDL 0x24
#define RXM1SIDH 0x25
#define RXM1EIDL 0x26
#define RXM1EIDH 0x27

/* CH9431 TX buffer 0 */
#define TXBCTRL(n) (((n) * 0x10) + 0x30 + TXBCTRL_OFF)
#define TXB0CTRL 0x30
#define TXBS_LOAD_CMD 0
#define TXBCTRL_OFF 0
#define TXBSIDL_OFF 1
#define TXBSIDH_OFF 2
#define RXBSIDH_EXIDE 0x10
#define TXBEIDL_OFF 3
#define TXBEIDH_OFF 4
#define TXBDLC_OFF 5
#define RXBDLC_RTR 0x40
#define RXBDLC_LEN_MASK 0x0F
#define TXBD_LOAD_CMD 6
#define TXBDAT_OFF 7

#define TXB0SIDL 0x31
#define TXB0SIDH 0x32
#define TXB0EIDL 0x33
#define TXB0EIDH 0x34

#define TXB0DLC 0x35
#define TXBDAT(n) (((n) * 0x10) + 0x30 + TXBD_LOAD_CMD)
#define TXB0D0 0x36
#define TXB0D1 0x37
#define TXB0D2 0x38
#define TXB0D3 0x39
#define TXB0D4 0x3A
#define TXB0D5 0x3B
#define TXB0D6 0x3C
#define TXB0D7 0x3D

/* CH9431 TX buffer 1 */
#define TXB1CTRL 0x40

#define TXB1SIDL 0x41
#define TXB1SIDH 0x42
#define TXB1EIDL 0x43
#define TXB1EIDH 0x44

#define TXB1DLC 0x45
#define TXB1D0 0x46
#define TXB1D1 0x47
#define TXB1D2 0x48
#define TXB1D3 0x49
#define TXB1D4 0x4A
#define TXB1D5 0x4B
#define TXB1D6 0x4C
#define TXB1D7 0x4D

/* CH9431 TX buffer 2 */
#define TXB2CTRL 0x50

#define TXB2SIDL 0x51
#define TXB2SIDH 0x52
#define TXB2EIDL 0x53
#define TXB2EIDH 0x54

#define TXB2DLC 0x55
#define TXB2D0 0x56
#define TXB2D1 0x57
#define TXB2D2 0x58
#define TXB2D3 0x59
#define TXB2D4 0x5A
#define TXB2D5 0x5B
#define TXB2D6 0x5C
#define TXB2D7 0x5D

/* CH9431 RX buffer 0 */
#define RXBCTRL(n) (((n) * 0x10) + 0x60 + RXBCTRL_OFF)
#define RXB0CTRL 0x60
#define RXBCTRL_BUKT (1 << 2)
#define RXBSIDL_OFF 0
#define RXBSIDH_OFF 1
#define RXBSIDH_EXIDE 0x10
#define RXBEIDL_OFF 2
#define RXBEIDH_OFF 3
#define RXBDLC_OFF 4
#define RXBDLC_RTR 0x40
#define RXBDLC_LEN_MASK 0x0F
#define RXBDAT_OFF 5
#define RXBDAT(n) (((n) * 0x10) + 0x60 + RXBDAT_OFF)
#define RXB0SIDL 0x61
#define RXB0SIDH 0x62
#define RXB0EIDL 0x63
#define RXB0EIDH 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66

/* CH9431 RX buffer 1 */
#define RXB1CTRL 0x70

#define RXB1SIDL 0x71
#define RXB1SIDH 0x72
#define RXB1EIDL 0x73
#define RXB1EIDH 0x74

#define RXB1DLC 0x75
#define RXB1D0 0x76
#define RXB1D1 0x77
#define RXB1D2 0x78
#define RXB1D3 0x79
#define RXB1D4 0x7A
#define RXB1D5 0x7B
#define RXB1D6 0x7C
#define RXB1D7 0x7D

static const struct can_bittiming_const ch9431_bittiming_const = {
	.name = DRVNAME_CH9431,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

struct reg_label {
	char *name;
	unsigned int reg;
};

static struct reg_label reg_labels[] = {
	REG_LABEL(CH9431_CTRL), REG_LABEL(CH9431_STAT),
	REG_LABEL(BTIMER1),	REG_LABEL(BTIMER2),
	REG_LABEL(BTIMER3),	REG_LABEL(CH9431_INTE),
	REG_LABEL(CH9431_INTF), REG_LABEL(CH9431_EFLAG),
	REG_LABEL(TXB0CTRL),	REG_LABEL(TXB0DLC),
	REG_LABEL(TXB0D0),	REG_LABEL(TXB1CTRL),
	REG_LABEL(TXB1DLC),	REG_LABEL(TXB1D0),
	REG_LABEL(TXB2CTRL),	REG_LABEL(TXB2DLC),
	REG_LABEL(TXB2D0),	REG_LABEL(RXB0CTRL),
	REG_LABEL(RXB1CTRL),	REG_LABEL(RXF0SIDL),
	REG_LABEL(RXF0SIDH),	REG_LABEL(RXF0EIDL),
	REG_LABEL(RXF0EIDH),	REG_LABEL(RXF2SIDL),
	REG_LABEL(RXF2SIDH),	REG_LABEL(RXF2EIDL),
	REG_LABEL(RXF2EIDH),	REG_LABEL(RXF3SIDL),
	REG_LABEL(RXF3SIDH),	REG_LABEL(RXF3EIDL),
	REG_LABEL(RXF3EIDH),
};

struct ch9431_priv;

struct ch9431_tx_agg {
	struct ch9431_priv *dev;
	struct work_struct work;
	struct sk_buff *skb;
	int tx_buff_id;
};

struct ch9431_ops {
	int (*dev_setup)(struct ch9431_priv *ch9431);
	irqreturn_t (*irq_handler)(int irq, void *pw);
	netdev_tx_t (*start_xmit)(struct sk_buff *skb,
				  struct net_device *ndev);
	void (*hw_rx)(struct ch9431_priv *ch9431, int rx_buf_idx);
	void (*tx_clean)(struct net_device *ndev);
};

struct ch9431_priv {
	struct can_priv can;
	struct net_device *ndev;
	struct spi_device *spi;
	struct mutex reg_lock;
	struct mutex ops_lock;

	u8 *spi_tx_buf;
	u8 *spi_rx_buf;

	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct restart_work;
	const struct ch9431_ops *dev_ops;
	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
#define AFTER_SUSPEND_POWER 4
#define AFTER_SUSPEND_RESTART 8
	int restart_tx;
	int gpio_irq_num;
	struct sk_buff *tx_skb;
	/* tx_pending reserves a hardware slot; tx_busy means echo skb exists. */
	bool tx_busy[CH9431_TX_BUF_NUM];
	u8 tx_pending;
	bool rx0_flag;
	struct regulator *power;
	struct regulator *transceiver;
	spinlock_t tx_lock;
	char link_name[32];
	u8 fw_version[8];
	bool sysfs_created;
};

#if CH9431_DEBUG
#define ch9431_dbg(_ch9431, _tag, _fmt, ...)                          \
	do {                                                          \
		struct ch9431_priv *__ch9431 = (_ch9431);             \
		if (__ch9431 && __ch9431->spi)                        \
			dev_info(&__ch9431->spi->dev, "%s %s: " _fmt, \
				 _tag, __func__, ##__VA_ARGS__);      \
	} while (0)
#define ch9431_dbg_rl(_ch9431, _tag, _fmt, ...)                       \
	do {                                                          \
		static unsigned long __next;                          \
		struct ch9431_priv *__ch9431 = (_ch9431);             \
		if (__ch9431 && __ch9431->spi &&                      \
		    time_after(jiffies, __next)) {                    \
			__next = jiffies +                            \
				 msecs_to_jiffies(                    \
					 CH9431_DBG_RATELIMIT_MS);    \
			dev_info(&__ch9431->spi->dev, "%s %s: " _fmt, \
				 _tag, __func__, ##__VA_ARGS__);      \
		}                                                     \
	} while (0)
#define ch9431_flow_dbg(_ch9431, _fmt, ...) \
	ch9431_dbg(_ch9431, CH9431_DBG_FLOW, _fmt, ##__VA_ARGS__)
#define ch9431_irq_dbg(_ch9431, _fmt, ...) \
	ch9431_dbg_rl(_ch9431, CH9431_DBG_IRQ, _fmt, ##__VA_ARGS__)
#define ch9431_tx_dbg(_ch9431, _fmt, ...) \
	ch9431_dbg(_ch9431, CH9431_DBG_TX, _fmt, ##__VA_ARGS__)
#define ch9431_rx_dbg(_ch9431, _fmt, ...) \
	ch9431_dbg_rl(_ch9431, CH9431_DBG_RX, _fmt, ##__VA_ARGS__)
#define ch9431_reg_dbg(_ch9431, _fmt, ...) \
	ch9431_dbg(_ch9431, CH9431_DBG_REG, _fmt, ##__VA_ARGS__)
#define ch9431_power_dbg(_ch9431, _fmt, ...) \
	ch9431_dbg(_ch9431, CH9431_DBG_POWER, _fmt, ##__VA_ARGS__)
#else
#define ch9431_flow_dbg(_ch9431, _fmt, ...) \
	do {                                \
	} while (0)
#define ch9431_irq_dbg(_ch9431, _fmt, ...) \
	do {                               \
	} while (0)
#define ch9431_tx_dbg(_ch9431, _fmt, ...) \
	do {                              \
	} while (0)
#define ch9431_rx_dbg(_ch9431, _fmt, ...) \
	do {                              \
	} while (0)
#define ch9431_reg_dbg(_ch9431, _fmt, ...) \
	do {                               \
	} while (0)
#define ch9431_power_dbg(_ch9431, _fmt, ...) \
	do {                                 \
	} while (0)
#endif

static int ch9431_v1_setup(struct ch9431_priv *ch9431);
static int ch9431_v2_setup(struct ch9431_priv *ch9431);
static irqreturn_t ch9431_v1_rx_threaded_irq(int irq, void *pw);
static irqreturn_t ch9431_v2_rx_threaded_irq(int irq, void *pw);
static netdev_tx_t ch9431_v1_start_xmit(struct sk_buff *skb,
					struct net_device *ndev);
static netdev_tx_t ch9431_v2_start_xmit(struct sk_buff *skb,
					struct net_device *ndev);
static void ch9431_v1_hw_rx(struct ch9431_priv *ch9431, int rx_buf_idx);
static void ch9431_v2_hw_rx(struct ch9431_priv *ch9431, int rx_buf_idx);
static void ch9431_v1_clean(struct net_device *ndev);
static void ch9431_v2_clean(struct net_device *ndev);

static const struct ch9431_ops ch9431_v1_ops = {
	.dev_setup = ch9431_v1_setup,
	.irq_handler = ch9431_v1_rx_threaded_irq,
	.start_xmit = ch9431_v1_start_xmit,
	.hw_rx = ch9431_v1_hw_rx,
	.tx_clean = ch9431_v1_clean,
};

static const struct ch9431_ops ch9431_v2_ops = {
	.dev_setup = ch9431_v2_setup,
	.irq_handler = ch9431_v2_rx_threaded_irq,
	.start_xmit = ch9431_v2_start_xmit,
	.hw_rx = ch9431_v2_hw_rx,
	.tx_clean = ch9431_v2_clean,
};

static void ch9431_v1_clean(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	if (ch9431->tx_skb || ch9431->tx_busy[0])
		ndev->stats.tx_errors++;
	dev_kfree_skb(ch9431->tx_skb);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0))
	if (ch9431->tx_busy[0])
		can_free_echo_skb(ch9431->ndev, 0, NULL);
#else
	if (ch9431->tx_busy[0])
		can_free_echo_skb(ch9431->ndev, 0);
#endif
	ch9431->tx_skb = NULL;
	ch9431->tx_busy[0] = false;
}

static void ch9431_v2_clean(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ch9431->tx_lock, flags);

	for (i = 0; i < CH9431_TX_BUF_NUM; i++) {
		if (ch9431->tx_busy[i]) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0))
			can_free_echo_skb(ndev, i, NULL);
#else
			can_free_echo_skb(ndev, i);
#endif
			ch9431->tx_busy[i] = false;
		}
	}

	ch9431->tx_pending = 0;

	spin_unlock_irqrestore(&ch9431->tx_lock, flags);
}

static int ch9431_v2_reserve_tx_slot(struct ch9431_priv *ch9431)
{
	struct net_device *ndev = ch9431->ndev;
	unsigned long flags;
	int tx_buff_id = -1;
	int i;

	spin_lock_irqsave(&ch9431->tx_lock, flags);

	for (i = 0; i < CH9431_TX_BUF_NUM; i++) {
		if (ch9431->tx_busy[i] || (ch9431->tx_pending & BIT(i)))
			continue;

		ch9431->tx_pending |= BIT(i);
		tx_buff_id = i;
		break;
	}

	if (ch9431->tx_pending == CH9431_TX_MASK || tx_buff_id < 0)
		netif_stop_queue(ndev);

	spin_unlock_irqrestore(&ch9431->tx_lock, flags);

	ch9431_tx_dbg(ch9431, "reserve slot=%d pending=0x%x stopped=%d\n",
		      tx_buff_id, ch9431->tx_pending,
		      netif_queue_stopped(ndev));

	return tx_buff_id;
}

static void ch9431_v2_release_tx_slot(struct ch9431_priv *ch9431,
				      int tx_buff_id)
{
	struct net_device *ndev = ch9431->ndev;
	unsigned long flags;
	bool wake_queue = false;

	if (tx_buff_id < 0 || tx_buff_id >= CH9431_TX_BUF_NUM)
		return;

	spin_lock_irqsave(&ch9431->tx_lock, flags);

	ch9431->tx_busy[tx_buff_id] = false;
	ch9431->tx_pending &= ~BIT(tx_buff_id);
	if (!ch9431->force_quit &&
	    ch9431->can.state != CAN_STATE_BUS_OFF &&
	    ch9431->can.state != CAN_STATE_STOPPED &&
	    netif_queue_stopped(ndev) &&
	    ch9431->tx_pending != CH9431_TX_MASK)
		wake_queue = true;

	spin_unlock_irqrestore(&ch9431->tx_lock, flags);

	if (wake_queue)
		netif_wake_queue(ndev);

	ch9431_tx_dbg(ch9431,
		      "release slot=%d pending=0x%x wake=%d stopped=%d\n",
		      tx_buff_id, ch9431->tx_pending, wake_queue,
		      netif_queue_stopped(ndev));
}

static int ch9431_spi_read_cmd(struct ch9431_priv *ch9431, const u8 *cmd,
			       int cmd_len, u8 *buf, int len, int delay)
{
	struct spi_transfer xfer[2] = {};
	struct spi_message m;
	int ret;

	if (cmd_len > CH9431_SPI_TX_BUF_LEN || len > CH9431_SPI_RX_BUF_LEN)
		return -EMSGSIZE;

	mutex_lock(&ch9431->reg_lock);

	memcpy(ch9431->spi_tx_buf, cmd, cmd_len);

	xfer[0].tx_buf = ch9431->spi_tx_buf;
	xfer[0].len = cmd_len;
	xfer[0].cs_change = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
	xfer[0].delay.value = delay;
#else
	xfer[0].delay_usecs = delay;
#endif

	xfer[1].rx_buf = ch9431->spi_rx_buf;
	xfer[1].len = len;

	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);

	ret = spi_sync(ch9431->spi, &m);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, spi transfer failed: ret = %d\n", __func__,
			ret);
	else
		memcpy(buf, ch9431->spi_rx_buf, len);

	mutex_unlock(&ch9431->reg_lock);

	return ret;
}

static int ch9431_spi_write(struct ch9431_priv *ch9431, const u8 *buf,
			    int len)
{
	int ret;

	if (len > CH9431_SPI_TX_BUF_LEN)
		return -EMSGSIZE;

	mutex_lock(&ch9431->reg_lock);

	memcpy(ch9431->spi_tx_buf, buf, len);
	ret = spi_write(ch9431->spi, ch9431->spi_tx_buf, len);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, spi write failed: ret = %d\n", __func__, ret);

	mutex_unlock(&ch9431->reg_lock);

	return ret;
}

static int ch9431_spi_trans(struct ch9431_priv *ch9431, u8 *buf, u8 reg,
			    int len, int delay)
{
	return ch9431_spi_read_cmd(ch9431, &reg, 1, buf, len, delay);
}

static int ch9431_read_reg_raw(struct ch9431_priv *ch9431, u8 reg, u8 *val)
{
	u8 cmd[2];
	u8 data = 0;
	int ret;

	cmd[0] = CMD_CAN_READ;
	cmd[1] = reg;

	ret = ch9431_spi_read_cmd(ch9431, cmd, sizeof(cmd), &data, 1,
				  WAIT_DATA_US);
	if (!ret)
		*val = data;

	return ret;
}

static u8 ch9431_read_reg(struct ch9431_priv *ch9431, u8 reg)
{
	u8 val = 0;

	ch9431_read_reg_raw(ch9431, reg, &val);

	return val;
}

static int ch9431_read_2regs(struct ch9431_priv *ch9431, u8 reg, u8 *v1,
			     u8 *v2)
{
	int ret;

	ret = ch9431_read_reg_raw(ch9431, reg, v1);
	if (ret)
		return ret;

	return ch9431_read_reg_raw(ch9431, reg + 1, v2);
}

static int ch9431_read_mem(struct ch9431_priv *ch9431, int rx_buf_idx,
			   u8 *buff, int len)
{
	return ch9431_spi_trans(ch9431, buff,
				INSTRUCTION_READ_RXBD(rx_buf_idx), len,
				WAIT_DATA_US);
}

static int ch9431_read_bulk(struct ch9431_priv *ch9431, int rx_buf_idx,
			    u8 *buff, int len)
{
	int ret;

	ret = ch9431_spi_trans(ch9431, buff, CMD_CAN_RX0_5PACK(rx_buf_idx),
			       len, BULK_WAIT_DATA_US);
	usleep_range(2, 3);

	return ret;
}

static int ch9431_v1_hw_rx_frame(struct ch9431_priv *ch9431, u8 *buf,
				 int rx_buf_idx)
{
	return ch9431_spi_trans(ch9431, buf,
				INSTRUCTION_READ_RXBS(rx_buf_idx),
				CAN_FRAME_HEADER_LEN, WAIT_DATA_US);
}

static int ch9431_v2_hw_rx_frame(struct ch9431_priv *ch9431, u8 *buf,
				 int rx_buf_idx)
{
	int ret;

	ret = ch9431_spi_trans(ch9431, buf,
			       INSTRUCTION_QREAD_RXBD(rx_buf_idx),
			       CH9431_FRAME_SIZE, BULK_WAIT_DATA_US);
	usleep_range(2, 3);

	return ret;
}

static int ch9431_write_reg(struct ch9431_priv *ch9431, u8 reg, u8 val)
{
	u8 cmd[3] = { CMD_CAN_WRITE, reg, val };
	int ret;

	ret = ch9431_spi_write(ch9431, cmd, 3);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, write reg 0x%02x value 0x%02x failed: %d\n",
			__func__, reg, val, ret);

	return ret;
}

static int ch9431_write_2regs(struct ch9431_priv *ch9431, u8 reg, u8 v1,
			      u8 v2)
{
	int ret;

	ret = ch9431_write_reg(ch9431, reg, v1);
	if (ret)
		return ret;

	return ch9431_write_reg(ch9431, reg + 1, v2);
}

static int ch9431_write_mem(struct ch9431_priv *ch9431, int tx_buf_idx,
			    u8 *buff, int len)
{
	return ch9431_spi_write(ch9431, buff, len);
}

static int ch9431_write_bits(struct ch9431_priv *ch9431, u8 reg, u8 mask,
			     u8 val)
{
	u8 cmd[4] = { CMD_CAN_BIT_MODIFY, reg, mask, val };
	int ret;

	ret = ch9431_spi_write(ch9431, cmd, 4);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, modify reg 0x%02x mask 0x%02x value 0x%02x failed: %d\n",
			__func__, reg, mask, val, ret);

	return ret;
}

static int ch9431_read_stat(struct ch9431_priv *ch9431, u8 *val)
{
	u8 stat;
	int ret;

	ret = ch9431_read_reg_raw(ch9431, CH9431_STAT, &stat);
	if (ret)
		return ret;

	*val = stat & CH9431_CTRL_REQOP_MASK;

	return 0;
}

static int ch9431_read_stat_poll_timeout(struct ch9431_priv *ch9431,
					 u8 mask, unsigned int delay_us,
					 unsigned int timeout_us)
{
	unsigned int elapsed_us = 0;
	u8 value;
	int ret;

	while (elapsed_us < timeout_us) {
		ret = ch9431_read_stat(ch9431, &value);
		if (ret)
			return ret;

		if (value == mask)
			return 0;

		usleep_range(delay_us / 2, delay_us);
		elapsed_us += delay_us;
	}

	return -ETIMEDOUT;
}

static int ch9431_get_version(struct ch9431_priv *ch9431)
{
	u8 cmd[2] = { 0x00, 0x76 };
	int ret;

	memset(ch9431->fw_version, 0, sizeof(ch9431->fw_version));
	ret = ch9431_spi_read_cmd(ch9431, cmd, sizeof(cmd),
				  ch9431->fw_version,
				  sizeof(ch9431->fw_version), 0);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, failed to read firmware version: %d\n",
			__func__, ret);

	dev_info(
		&ch9431->spi->dev,
		"ch9431 device probe, driver version: %s, fw version: %.*s\n",
		VERSION_DESC, (int)sizeof(ch9431->fw_version),
		ch9431->fw_version);

	return ret;
}

static int ch9431_clear_errf(struct ch9431_priv *ch9431, u8 mask)
{
	return ch9431_write_bits(ch9431, CH9431_EFLAG, mask, 0x00);
}

static int ch9431_disable_intr(struct ch9431_priv *ch9431)
{
	return ch9431_write_reg(ch9431, CH9431_INTE, 0x00);
}

static int ch9431_clear_intr(struct ch9431_priv *ch9431, u8 mask)
{
	return ch9431_write_bits(ch9431, CH9431_INTF, mask, 0x00);
}

static int ch9431_enable_intr(struct ch9431_priv *ch9431)
{
	return ch9431_write_reg(
		ch9431, CH9431_INTE,
		CH9431_INTE_RX0IE | CH9431_INTE_RX1IE | CH9431_INTE_ERRIE |
			CH9431_INTE_TX0IE | CH9431_INTE_TX1IE |
			CH9431_INTE_TX2IE);
}

static int ch9431_enable_intr_except_rx0(struct ch9431_priv *ch9431)
{
	return ch9431_write_reg(
		ch9431, CH9431_INTE,
		CH9431_INTE_RX1IE | CH9431_INTE_ERRIE | CH9431_INTE_TX0IE |
			CH9431_INTE_TX1IE | CH9431_INTE_TX2IE);
}

static const struct ethtool_ops ch9431_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static int ch9431_hw_tx_frame(struct ch9431_priv *ch9431, u8 *buf,
			      int tx_buf_idx)
{
	return ch9431_spi_write(ch9431, buf, CAN_FRAME_TX_CMD_LEN);
}

static int ch9431_hw_tx(struct ch9431_priv *ch9431,
			struct can_frame *frame, int tx_buf_idx)
{
	u32 sid, eid, exide, rtr;
	u8 buf[SPI_TRANSFER_BUF_LEN + CH9431_TXBD_CMD_LEN];
	u8 cmd;
	int ret;

	exide = (frame->can_id & CAN_EFF_FLAG) ?
			1 :
			0; /* Extended ID Enable */
	if (exide)
		sid = (frame->can_id & CAN_EFF_MASK) >> 18;
	else
		sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
	eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 :
					       0; /* Remote transmission */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	ch9431_tx_dbg(
		ch9431,
		"hw tx slot=%d can_id=0x%08x len=%u exide=%u rtr=%u\n",
		tx_buf_idx, frame->can_id, frame->len, exide, rtr);
#else
	ch9431_tx_dbg(
		ch9431,
		"hw tx slot=%d can_id=0x%08x dlc=%u exide=%u rtr=%u\n",
		tx_buf_idx, frame->can_id, frame->can_dlc, exide, rtr);
#endif

	buf[TXBS_LOAD_CMD] = INSTRUCTION_LOAD_TXBS(tx_buf_idx);
	buf[TXBSIDL_OFF] = (sid & 0xFF);
	buf[TXBSIDH_OFF] = ((eid & 0x03) << 6) | (exide << 4) |
			   ((sid >> 8) & 0x07);
	buf[TXBEIDL_OFF] = (eid >> 2) & 0xFF;
	buf[TXBEIDH_OFF] = (eid >> 10) & 0xFF;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	buf[TXBDLC_OFF] = (rtr << 6) | (frame->len & 0x0F);
	buf[TXBD_LOAD_CMD] = INSTRUCTION_LOAD_TXBD(tx_buf_idx);

	memcpy(buf + TXBDAT_OFF, frame->data, frame->len);
	ret = ch9431_hw_tx_frame(ch9431, buf, tx_buf_idx);
	if (ret)
		return ret;
	ret = ch9431_write_mem(ch9431, tx_buf_idx, buf + TXBD_LOAD_CMD,
			       frame->len + CH9431_TXBD_CMD_LEN);
#else
	buf[TXBDLC_OFF] = (rtr << 6) | (frame->can_dlc & 0x0F);
	buf[TXBD_LOAD_CMD] = INSTRUCTION_LOAD_TXBD(tx_buf_idx);

	memcpy(buf + TXBDAT_OFF, frame->data, frame->can_dlc);
	ret = ch9431_hw_tx_frame(ch9431, buf, tx_buf_idx);
	if (ret)
		return ret;
	ret = ch9431_write_mem(ch9431, tx_buf_idx, buf + TXBD_LOAD_CMD,
			       frame->can_dlc + CH9431_TXBD_CMD_LEN);
#endif
	if (ret)
		return ret;

	/* use INSTRUCTION_RTS, to avoid "repeated frame problem" */
	cmd = INSTRUCTION_RTS(1 << tx_buf_idx);
	return ch9431_spi_write(ch9431, &cmd, 1);
}

static int ch9431_clear_tx_buffers(struct ch9431_priv *ch9431)
{
	int i, ret;

	for (i = 0; i < CH9431_TX_BUF_NUM; i++) {
		ret = ch9431_write_reg(ch9431, TXBCTRL(i), 0);
		if (ret)
			return ret;
	}

	return 0;
}

static void ch9431_v1_hw_rx(struct ch9431_priv *ch9431, int rx_buf_idx)
{
	struct net_device *ndev = ch9431->ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 can_id;
	u8 buf[CH9431_FRAME_SIZE];
	u8 dlc, len;
	int ret;

	ret = ch9431_v1_hw_rx_frame(ch9431, buf, rx_buf_idx);
	if (unlikely(ret)) {
		ndev->stats.rx_errors++;
		return;
	}

	if (buf[RXBSIDH_OFF] & RXBSIDH_EXIDE) {
		/* Extended ID format */
		can_id = CAN_EFF_FLAG;
		can_id |=
			/* Extended ID part */
			(buf[RXBEIDH_OFF] << 10) |
			(buf[RXBEIDL_OFF] << 2) |
			((buf[RXBSIDH_OFF] >> 6) & 0x03) |
			/* Standard ID part */
			((((buf[RXBSIDH_OFF] & 0x07) << 8) |
			  buf[RXBSIDL_OFF])
			 << 18);
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR) {
			can_id |= CAN_RTR_FLAG;
		}
	} else {
		/* Standard ID format */
		can_id = ((((buf[RXBSIDH_OFF] & 0x07) << 8) |
			   buf[RXBSIDL_OFF]));
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR) {
			can_id |= CAN_RTR_FLAG;
		}
	}

	/* Data length */
	dlc = buf[RXBDLC_OFF] & RXBDLC_LEN_MASK;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	len = min_t(u8, can_cc_dlc2len(dlc), CAN_MAX_DLEN);
#else
	len = min_t(u8, get_can_dlc(dlc), CAN_MAX_DLEN);
#endif

	skb = alloc_can_skb(ndev, &frame);
	if (!skb) {
		dev_err(&ch9431->spi->dev, "%s, cannot allocate RX skb\n",
			__func__);
		ch9431->ndev->stats.rx_dropped++;
		return;
	}

	frame->can_id = can_id;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	frame->len = len;
#else
	frame->can_dlc = len;
#endif

	if (!(can_id & CAN_RTR_FLAG) && len) {
		ret = ch9431_read_mem(ch9431, rx_buf_idx, frame->data,
				      len);
		if (unlikely(ret)) {
			ndev->stats.rx_errors++;
			dev_kfree_skb_any(skb);
			return;
		}
		ndev->stats.rx_bytes += len;
	}

	ch9431->ndev->stats.rx_packets++;
	ch9431_rx_dbg(ch9431, "v1 rx buf=%d can_id=0x%08x len=%u\n",
		      rx_buf_idx, can_id, len);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	netif_rx(skb);
#else
	netif_rx_ni(skb);
#endif
}

static void ch9431_v2_hw_rx(struct ch9431_priv *ch9431, int rx_buf_idx)
{
	struct net_device *ndev = ch9431->ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 can_id;
	u8 buf[CH9431_FRAME_SIZE];
	u8 dlc, len;
	int ret;

	ret = ch9431_v2_hw_rx_frame(ch9431, buf, rx_buf_idx);
	if (unlikely(ret)) {
		ndev->stats.rx_errors++;
		return;
	}

	if (buf[RXBSIDH_OFF] & RXBSIDH_EXIDE) {
		/* Extended ID format */
		can_id = CAN_EFF_FLAG;
		can_id |=
			/* Extended ID part */
			(buf[RXBEIDH_OFF] << 10) |
			(buf[RXBEIDL_OFF] << 2) |
			((buf[RXBSIDH_OFF] >> 6) & 0x03) |
			/* Standard ID part */
			((((buf[RXBSIDH_OFF] & 0x07) << 8) |
			  buf[RXBSIDL_OFF])
			 << 18);
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR) {
			can_id |= CAN_RTR_FLAG;
		}
	} else {
		/* Standard ID format */
		can_id = ((((buf[RXBSIDH_OFF] & 0x07) << 8) |
			   buf[RXBSIDL_OFF]));
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR) {
			can_id |= CAN_RTR_FLAG;
		}
	}

	/* Data length */
	dlc = buf[RXBDLC_OFF] & RXBDLC_LEN_MASK;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	len = min_t(u8, can_cc_dlc2len(dlc), CAN_MAX_DLEN);
#else
	len = min_t(u8, get_can_dlc(dlc), CAN_MAX_DLEN);
#endif

	skb = alloc_can_skb(ndev, &frame);
	if (!skb) {
		dev_err(&ch9431->spi->dev, "%s, cannot allocate RX skb\n",
			__func__);
		ch9431->ndev->stats.rx_dropped++;
		return;
	}

	frame->can_id = can_id;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	frame->len = len;
#else
	frame->can_dlc = len;
#endif

	if (!(can_id & CAN_RTR_FLAG) && len) {
		memcpy(frame->data, &buf[RXBDAT_OFF], len);
		ndev->stats.rx_bytes += len;
	}

	ch9431->ndev->stats.rx_packets++;
	ch9431_rx_dbg(ch9431, "v2 rx buf=%d can_id=0x%08x len=%u\n",
		      rx_buf_idx, can_id, len);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	netif_rx(skb);
#else
	netif_rx_ni(skb);
#endif
}

static void ch9431_hw_rx_bulk(struct ch9431_priv *ch9431, int rx_buf_idx)
{
	struct net_device *ndev = ch9431->ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	u8 buf[CH9431_BULK_SIZE];
	int i, valid_frames;
	int ret;

	ret = ch9431_read_bulk(ch9431, rx_buf_idx, buf, sizeof(buf));
	if (unlikely(ret)) {
		ndev->stats.rx_errors++;
		return;
	}

	valid_frames = buf[0] + 1;
	if (valid_frames > CH9431_MAX_FRAMES)
		valid_frames = CH9431_MAX_FRAMES;
	ch9431_rx_dbg(ch9431, "bulk rx buf=%d count=%d raw=0x%02x\n",
		      rx_buf_idx, valid_frames, buf[0]);

	for (i = 0; i < valid_frames; i++) {
		u8 *p = &buf[1 + i * CH9431_FRAME_SIZE];
		u8 dlc, len;

		skb = alloc_can_skb(ndev, &frame);
		if (!skb) {
			ndev->stats.rx_dropped++;
			continue;
		}

		if (p[RXBSIDH_OFF] & RXBSIDH_EXIDE) {
			/* Extended ID format */
			frame->can_id = CAN_EFF_FLAG;
			frame->can_id |=
				/* Extended ID part */
				(p[RXBEIDH_OFF] << 10) |
				(p[RXBEIDL_OFF] << 2) |
				((p[RXBSIDH_OFF] >> 6) & 0x03) |
				/* Standard ID part */
				((((p[RXBSIDH_OFF] & 0x07) << 8) |
				  p[RXBSIDL_OFF])
				 << 18);
			/* Remote transmission request */
			if (p[RXBDLC_OFF] & RXBDLC_RTR) {
				frame->can_id |= CAN_RTR_FLAG;
			}
		} else {
			/* Standard ID format */
			frame->can_id = ((((p[RXBSIDH_OFF] & 0x07) << 8) |
					  p[RXBSIDL_OFF]));
			/* Remote transmission request */
			if (p[RXBDLC_OFF] & RXBDLC_RTR) {
				frame->can_id |= CAN_RTR_FLAG;
			}
		}

		dlc = p[RXBDLC_OFF] & RXBDLC_LEN_MASK;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
		len = min_t(u8, can_cc_dlc2len(dlc), CAN_MAX_DLEN);
		frame->len = len;
#else
		len = min_t(u8, get_can_dlc(dlc), CAN_MAX_DLEN);
		frame->can_dlc = len;
#endif

		if (!(frame->can_id & CAN_RTR_FLAG) && len) {
			memcpy(frame->data, &p[RXBDAT_OFF], len);
			ndev->stats.rx_bytes += len;
		}

		ndev->stats.rx_packets++;
		ch9431_rx_dbg(ch9431,
			      "bulk rx[%d] buf=%d can_id=0x%08x len=%u\n",
			      i, rx_buf_idx, frame->can_id, len);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
		netif_rx(skb);
#else
		netif_rx_ni(skb);
#endif
	}
}

static int ch9431_get_optional_regulator(struct device *dev,
					 const char *id,
					 struct regulator **reg)
{
	int ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
	*reg = devm_regulator_get_optional(dev, id);
#else
	*reg = devm_regulator_get(dev, id);
#endif
	if (!IS_ERR(*reg))
		return 0;

	ret = PTR_ERR(*reg);
	if (ret == -ENODEV || ret == -ENOSYS) {
		*reg = NULL;
		return 0;
	}

	if (ret != -EPROBE_DEFER)
		dev_err(dev, "failed to get %s regulator: %d\n", id, ret);

	return ret;
}

static int ch9431_power_enable(struct regulator *reg, int enable)
{
	if (!reg)
		return 0;
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static int ch9431_hw_sleep(struct ch9431_priv *ch9431)
{
	return ch9431_write_bits(ch9431, CH9431_CTRL,
				 CH9431_CTRL_REQOP_MASK,
				 CH9431_CTRL_REQOP_SLEEP_DEEP);
}

/* May only be called when device is sleeping! */
static int ch9431_hw_wake(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	bool irq_disabled = false;
	u8 cmd;
	int ret;

	/*
	 * Mask IRQ while waking the device.  Callers may already hold ops_lock
	 * while a threaded IRQ is blocked on the same mutex; disable_irq()
	 * would wait for that thread and deadlock.
	 */
	disable_irq_nosync(spi->irq);
	irq_disabled = true;
	ch9431_power_dbg(ch9431, "wake begin irq=%d\n", spi->irq);

	cmd = CMD_CAN_RD_STATUS;
	ret = ch9431_spi_write(ch9431, &cmd, 1);
	if (ret) {
		dev_err(&spi->dev,
			"CH9431 CAN read status cmd send fail.\n");
		goto out_enable_irq;
	}

	/* Wait for oscillator startup timer after wake up */
	mdelay(OST_DELAY_MS);

	/* Put device into config mode */
	ret = ch9431_write_bits(ch9431, CH9431_CTRL,
				CH9431_CTRL_REQOP_MASK,
				CH9431_CTRL_REQOP_CONFIG);
	if (ret)
		goto out_enable_irq;

	/* Wait for the device to enter config mode */
	ret = ch9431_read_stat_poll_timeout(ch9431,
					    CH9431_CTRL_REQOP_CONFIG,
					    OST_DELAY_MS * 1000, 10000);
	if (ret) {
		dev_err(&spi->dev,
			"%s, ret = %d, ch9431 didn't enter in config mode!\n",
			__func__, ret);
		goto out_enable_irq;
	}

	/* Disable and clear pending interrupts */
	ret = ch9431_write_2regs(ch9431, CH9431_INTE, 0x00, 0x00);

out_enable_irq:
	if (irq_disabled)
		enable_irq(spi->irq);

	ch9431_power_dbg(ch9431, "wake done ret=%d\n", ret);

	return ret;
}

static int ch9431_set_normal_mode(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	const char *mode_name;
	u8 reqop;
	int ret;

	/* Enable interrupts */
	ret = ch9431_enable_intr(ch9431);
	if (ret)
		return ret;

	if (ch9431->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		reqop = CH9431_CTRL_REQOP_LOOPBACK;
		mode_name = "loopback";
	} else if (ch9431->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		reqop = CH9431_CTRL_REQOP_LISTEN;
		mode_name = "listen-only";
	} else {
		reqop = CH9431_CTRL_REQOP_NORMAL;
		mode_name = "normal";
	}

	ret = ch9431_write_bits(ch9431, CH9431_CTRL,
				CH9431_CTRL_REQOP_MASK, reqop);
	if (ret)
		return ret;

	ret = ch9431_read_stat_poll_timeout(ch9431, reqop,
					    OST_DELAY_MS * 1000, 10000);
	if (ret) {
		dev_err(&spi->dev,
			"%s, ret = %d, ch9431 didn't enter in %s mode!\n",
			__func__, ret, mode_name);
		return ret;
	}

	ch9431->can.state = CAN_STATE_ERROR_ACTIVE;
	ch9431_flow_dbg(ch9431, "set mode done ctrlmode=0x%x state=%d\n",
			ch9431->can.ctrlmode, ch9431->can.state);
	return 0;
}

/* Set CNF1/CNF2/CNF3 Set Baud Rate*/
static int ch9431_do_set_bittiming(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	struct can_bittiming *bt = &ch9431->can.bittiming;
	u8 cfg1, cfg2, cfg3;
	int ret;

	cfg1 = (bt->brp - 1);
	cfg2 = (((bt->sjw - 1) << 4) |
		(bt->prop_seg + bt->phase_seg1 - 1));
	cfg3 = (bt->phase_seg2 - 1);

	ret = ch9431_write_reg(ch9431, BTIMER1, cfg1);
	if (ret)
		return ret;

	ret = ch9431_write_reg(ch9431, BTIMER2, cfg2);
	if (ret)
		return ret;

	ret = ch9431_write_reg(ch9431, BTIMER3, cfg3);
	if (ret)
		return ret;

	dev_dbg(&ch9431->spi->dev, "%s, CNF: 0x%02x 0x%02x 0x%02x\n",
		__func__, ch9431_read_reg(ch9431, BTIMER1),
		ch9431_read_reg(ch9431, BTIMER2),
		ch9431_read_reg(ch9431, BTIMER3));

	return 0;
}

static int ch9431_set_rxfilter(struct ch9431_priv *ch9431)
{
	int ret;

	ret = ch9431_write_reg(ch9431, RXF0SIDH, 0x08);
	if (ret)
		return ret;

	ret = ch9431_write_reg(ch9431, RXF1SIDH, 0x18);
	if (ret)
		return ret;

	return 0;
}

static int ch9431_set_rxmask(struct ch9431_priv *ch9431, int rxm_id,
			     u8 val)
{
	int ret, i;

	for (i = 0; i < 4; i++) {
		ret = ch9431_write_reg(ch9431, RXMID(rxm_id) + i, val);
		if (ret)
			return ret;
	}

	return 0;
}

static int ch9431_v1_setup(struct ch9431_priv *ch9431)
{
	struct net_device *ndev = ch9431->ndev;
	int ret;

	ret = ch9431_write_reg(ch9431, RXB0CTRL, RXBCTRL_BUKT);
	if (ret)
		return ret;

	ret = ch9431_set_rxmask(ch9431, 0, 0x00);
	if (ret)
		return ret;

	ret = ch9431_set_rxmask(ch9431, 1, 0x00);
	if (ret)
		return ret;

	ret = ch9431_set_rxfilter(ch9431);
	if (ret)
		return ret;

	return ch9431_do_set_bittiming(ndev);
}

static int ch9431_v2_setup(struct ch9431_priv *ch9431)
{
	struct net_device *ndev = ch9431->ndev;
	int ret;

	ret = ch9431_write_reg(ch9431, RXB0CTRL, 0x00);
	if (ret)
		return ret;

	ret = ch9431_set_rxmask(ch9431, 0, 0x00);
	if (ret)
		return ret;

	ret = ch9431_set_rxmask(ch9431, 1, 0x00);
	if (ret)
		return ret;

	ret = ch9431_set_rxfilter(ch9431);
	if (ret)
		return ret;

	return ch9431_do_set_bittiming(ndev);
}

static void ch9431_v1_tx_delay(struct work_struct *work)
{
	struct ch9431_priv *ch9431 =
		container_of(work, struct ch9431_priv, tx_work);
	struct net_device *ndev = ch9431->ndev;
	struct can_frame *frame;
	int ret;

	mutex_lock(&ch9431->ops_lock);

	if (ch9431->tx_skb) {
		ch9431_tx_dbg(ch9431, "v1 tx work state=%d busy=%d\n",
			      ch9431->can.state, ch9431->tx_busy[0]);
		if (ch9431->can.state == CAN_STATE_BUS_OFF) {
			ch9431->dev_ops->tx_clean(ndev);
		} else {
			frame = (struct can_frame *)ch9431->tx_skb->data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
			if (frame->len > CAN_FRAME_MAX_DATA_LEN)
				frame->len = CAN_FRAME_MAX_DATA_LEN;
#else
			if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
				frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
#endif
			ret = ch9431_hw_tx(ch9431, frame, 0);
			if (unlikely(ret)) {
				ndev->stats.tx_errors++;
				ch9431_tx_dbg(ch9431,
					      "v1 tx failed ret=%d\n",
					      ret);
				dev_kfree_skb_any(ch9431->tx_skb);
				ch9431->tx_skb = NULL;
				netif_wake_queue(ndev);
				goto out_unlock;
			}

			ch9431->tx_busy[0] = true;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
			can_put_echo_skb(ch9431->tx_skb, ndev, 0, 0);
#else
			can_put_echo_skb(ch9431->tx_skb, ndev, 0);
#endif
			ch9431->tx_skb = NULL;
			ch9431_tx_dbg(ch9431,
				      "v1 tx queued echo busy=%d\n",
				      ch9431->tx_busy[0]);
		}
	}

out_unlock:
	mutex_unlock(&ch9431->ops_lock);
}

static void ch9431_v2_tx_delay(struct work_struct *work)
{
	struct ch9431_tx_agg *tx_agg =
		container_of(work, struct ch9431_tx_agg, work);
	struct ch9431_priv *ch9431 = tx_agg->dev;
	struct net_device *ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	unsigned long flags;
	int tx_buff_id;
	int ret;

	if (!ch9431) {
		kfree(tx_agg);
		return;
	}

	ndev = ch9431->ndev;
	if (!ndev) {
		kfree(tx_agg);
		return;
	}

	skb = tx_agg->skb;
	tx_buff_id = tx_agg->tx_buff_id;
	mutex_lock(&ch9431->ops_lock);

	if (!skb || ch9431->force_quit ||
	    ch9431->can.state == CAN_STATE_BUS_OFF ||
	    ch9431->can.state == CAN_STATE_STOPPED) {
		mutex_unlock(&ch9431->ops_lock);
		ch9431_tx_dbg(
			ch9431,
			"v2 tx drop slot=%d skb=%p force=%d state=%d\n",
			tx_buff_id, skb, ch9431->force_quit,
			ch9431->can.state);
		dev_kfree_skb_any(skb);
		ch9431_v2_release_tx_slot(ch9431, tx_buff_id);
		kfree(tx_agg);
		return;
	}

	frame = (struct can_frame *)skb->data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	frame->len = min_t(u8, frame->len, CAN_FRAME_MAX_DATA_LEN);
#else
	frame->can_dlc = min_t(u8, frame->can_dlc, CAN_FRAME_MAX_DATA_LEN);
#endif

	ret = ch9431_hw_tx(ch9431, frame, tx_buff_id);
	if (unlikely(ret)) {
		mutex_unlock(&ch9431->ops_lock);
		ndev->stats.tx_errors++;
		ch9431_tx_dbg(ch9431, "v2 tx failed slot=%d ret=%d\n",
			      tx_buff_id, ret);
		dev_kfree_skb_any(skb);
		ch9431_v2_release_tx_slot(ch9431, tx_buff_id);
		kfree(tx_agg);
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
	can_put_echo_skb(skb, ndev, tx_buff_id, 0);
#else
	can_put_echo_skb(skb, ndev, tx_buff_id);
#endif
	spin_lock_irqsave(&ch9431->tx_lock, flags);
	ch9431->tx_busy[tx_buff_id] = true;
	spin_unlock_irqrestore(&ch9431->tx_lock, flags);
	ch9431_tx_dbg(ch9431, "v2 tx queued echo slot=%d pending=0x%x\n",
		      tx_buff_id, ch9431->tx_pending);
	mutex_unlock(&ch9431->ops_lock);

	kfree(tx_agg);
}

static int ch9431_hw_reset(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	u8 cmd;
	int ret;

	/* Wait for oscillator startup timer after power up */
	mdelay(OST_DELAY_MS);

	cmd = CMD_CAN_RESET;
	ret = ch9431_spi_write(ch9431, &cmd, 1);
	if (ret) {
		dev_err(&spi->dev, "CH9431 CAN reset cmd send fail.\n");
		return ret;
	}

	/* Wait for oscillator startup timer after reset */
	msleep(RST_DELAY_MS);

	/* Wait for reset to finish */
	ret = ch9431_read_stat_poll_timeout(ch9431,
					    CH9431_CTRL_REQOP_CONFIG,
					    OST_DELAY_MS * 1000, 10000);
	if (ret)
		dev_err(&spi->dev,
			"%s, ch9431 didn't enter in conf mode after reset\n",
			__func__);
	return ret;
}

static void ch9431_error_skb(struct net_device *ndev, int can_id,
			     int data1)
{
	struct sk_buff *skb;
	struct can_frame *frame;

	skb = alloc_can_err_skb(ndev, &frame);
	if (skb) {
		frame->can_id |= can_id;
		frame->data[1] = data1;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
		netif_rx(skb);
#else
		netif_rx_ni(skb);
#endif
	} else {
		netdev_err(ndev, "%s, cannot allocate error skb\n",
			   __func__);
	}
}

static int ch9431_irq_gpio_value(struct ch9431_priv *ch9431)
{
	return gpio_get_value_cansleep(ch9431->gpio_irq_num);
}

static void ch9431_restart_tx(struct work_struct *work)
{
	struct ch9431_priv *ch9431 =
		container_of(work, struct ch9431_priv, restart_work);
	struct net_device *ndev = ch9431->ndev;
	int after_suspend;
	int ret = 0;

	mutex_lock(&ch9431->ops_lock);
	after_suspend = ch9431->after_suspend;
	ch9431_flow_dbg(
		ch9431,
		"restart work begin after_suspend=0x%x restart_tx=%d state=%d\n",
		after_suspend, ch9431->restart_tx, ch9431->can.state);
	if (after_suspend) {
		if (after_suspend & AFTER_SUSPEND_POWER) {
			ret = ch9431_hw_reset(ch9431);
			if (!ret)
				ret = ch9431->dev_ops->dev_setup(ch9431);
		} else {
			ret = ch9431_hw_wake(ch9431);
		}

		if (ret)
			goto restore_failed;

		if (after_suspend & AFTER_SUSPEND_RESTART) {
			ret = ch9431_set_normal_mode(ch9431);
			if (ret)
				goto restore_failed;
		} else if (after_suspend & AFTER_SUSPEND_UP) {
			ch9431->dev_ops->tx_clean(ndev);
			ret = ch9431_set_normal_mode(ch9431);
			if (ret)
				goto restore_failed;
			netif_device_attach(ndev);
			netif_wake_queue(ndev);
		} else {
			ret = ch9431_hw_sleep(ch9431);
			if (ret)
				goto restore_failed;
		}

		ch9431->force_quit = 0;
		ch9431->after_suspend = 0;
	}

	if (ch9431->restart_tx) {
		ch9431->restart_tx = 0;
		ch9431_tx_dbg(ch9431, "restart tx cleanup pending=0x%x\n",
			      ch9431->tx_pending);
		ret = ch9431_clear_tx_buffers(ch9431);
		if (ret)
			goto restore_failed;
		ch9431->dev_ops->tx_clean(ndev);
		netif_wake_queue(ndev);
		ch9431_error_skb(ndev, CAN_ERR_RESTARTED, 0);
	}
	goto out;

restore_failed:
	netdev_err(ndev, "%s, failed to restore controller state: %d\n",
		   __func__, ret);
	ch9431_flow_dbg(ch9431, "restart work failed ret=%d\n", ret);

out:
	ch9431_flow_dbg(
		ch9431,
		"restart work done ret=%d force=%d state=%d pending=0x%x\n",
		ret, ch9431->force_quit, ch9431->can.state,
		ch9431->tx_pending);
	mutex_unlock(&ch9431->ops_lock);
}

static irqreturn_t ch9431_v1_rx_threaded_irq(int irq, void *pw)
{
	struct ch9431_priv *ch9431 = pw;
	struct net_device *ndev = ch9431->ndev;
	enum can_state old_state, new_state;
	u8 intf = 0, errf = 0;
	u8 clear_intf = 0;
	int can_id = 0, data1 = 0;
	int gpio_level = 0;
	bool gpio_read_failed = false;
	unsigned long timeout = 0;
	int ret;

	if (ch9431->force_quit) {
		ch9431_irq_dbg(ch9431, "v1 irq ignored force_quit\n");
		return IRQ_HANDLED;
	}

	mutex_lock(&ch9431->ops_lock);
	old_state = ch9431->can.state;
	ch9431_irq_dbg(ch9431, "v1 irq begin state=%d rx0_flag=%d\n",
		       old_state, ch9431->rx0_flag);

	ret = ch9431_disable_intr(ch9431);
	if (unlikely(ret)) {
		ch9431_irq_dbg(ch9431, "v1 disable intr failed ret=%d\n",
			       ret);
		goto out;
	}

	ret = ch9431_read_2regs(ch9431, CH9431_INTF, &intf, &errf);
	if (unlikely(ret)) {
		ch9431->rx0_flag = false;
		ch9431_irq_dbg(ch9431, "v1 read intf/errf failed ret=%d\n",
			       ret);
		goto out;
	}
	clear_intf |= intf &
		      (CH9431_INTF_TX | CH9431_INTF_RX | CH9431_INTF_ERR);
	ch9431_irq_dbg(ch9431,
		       "v1 irq intf=0x%02x errf=0x%02x clear=0x%02x\n",
		       intf, errf, clear_intf);

	/* Receive buffer 0 */
	if (intf & CH9431_INTF_RX0IF && !ch9431->rx0_flag) {
		ch9431_irq_dbg(ch9431, "v1 irq rx0\n");
		ch9431->dev_ops->hw_rx(ch9431, 0);
	}

	/* Receive buffer 1 */
	if (intf & CH9431_INTF_RX1IF) {
		ch9431_irq_dbg(ch9431, "v1 irq rx1\n");
		ch9431->dev_ops->hw_rx(ch9431, 1);
	}

	/*
	 * if only RX0 interrupt is pending
	 * defer clearing RX0IF to allow packet reception.
	 * set rx0_flag to indicate this condition.
	 */
	if ((clear_intf & CH9431_INTF_RX) == CH9431_INTF_RX0IF) {
		clear_intf &= ~CH9431_INTF_RX0IF;
		ch9431->rx0_flag = true;
		ch9431_irq_dbg(ch9431, "v1 defer rx0 clear\n");
	} else {
		ch9431->rx0_flag = false;
	}

	if (clear_intf) {
		ret = ch9431_clear_intr(ch9431, clear_intf);
		if (unlikely(ret)) {
			ch9431_irq_dbg(
				ch9431,
				"v1 clear intf=0x%02x failed ret=%d\n",
				clear_intf, ret);
			goto out;
		}
	}

	if (errf & (CH9431_EFLAG_RX0OVR | CH9431_EFLAG_RX1OVR)) {
		ret = ch9431_clear_errf(ch9431, errf);
		if (unlikely(ret)) {
			ch9431_irq_dbg(
				ch9431,
				"v1 clear errf=0x%02x failed ret=%d\n",
				errf, ret);
			goto out;
		}
	}

	/* Update can state */
	if (errf & CH9431_EFLAG_TXBO) {
		new_state = CAN_STATE_BUS_OFF;
		can_id |= CAN_ERR_BUSOFF;
	} else if (errf & CH9431_EFLAG_TXEP) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_TX_PASSIVE;
	} else if (errf & CH9431_EFLAG_RXEP) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_RX_PASSIVE;
	} else if (errf & CH9431_EFLAG_TXWAR) {
		new_state = CAN_STATE_ERROR_WARNING;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_TX_WARNING;
	} else if (errf & CH9431_EFLAG_RXWAR) {
		new_state = CAN_STATE_ERROR_WARNING;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_RX_WARNING;
	} else {
		new_state = CAN_STATE_ERROR_ACTIVE;
	}

	/* Update can state statistics */
	switch (old_state) {
	case CAN_STATE_ERROR_ACTIVE:
		if (new_state >= CAN_STATE_ERROR_WARNING &&
		    new_state <= CAN_STATE_BUS_OFF)
			ch9431->can.can_stats.error_warning++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		fallthrough;
#endif
	case CAN_STATE_ERROR_WARNING: /* fallthrough */
		if (new_state >= CAN_STATE_ERROR_PASSIVE &&
		    new_state <= CAN_STATE_BUS_OFF)
			ch9431->can.can_stats.error_passive++;
		break;
	default:
		break;
	}
	ch9431->can.state = new_state;
	if (new_state != old_state || errf)
		ch9431_irq_dbg(ch9431,
			       "v1 state %d->%d can_id=0x%x data1=0x%x\n",
			       old_state, new_state, can_id, data1);

	if (intf & CH9431_INTF_ERRIF) {
		/* Handle overflow counters */
		if (errf & (CH9431_EFLAG_RX0OVR | CH9431_EFLAG_RX1OVR)) {
			if (errf & CH9431_EFLAG_RX0OVR) {
				ndev->stats.rx_over_errors++;
				ndev->stats.rx_errors++;
			}
			if (errf & CH9431_EFLAG_RX1OVR) {
				ndev->stats.rx_over_errors++;
				ndev->stats.rx_errors++;
			}
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_OVERFLOW;
		}
		ch9431_error_skb(ndev, can_id, data1);
	}

	if (new_state == CAN_STATE_BUS_OFF &&
	    old_state != CAN_STATE_BUS_OFF) {
		ch9431->force_quit = 1;
		ch9431->can.can_stats.bus_off++;
		ch9431_irq_dbg(ch9431, "v1 bus off, enter sleep\n");
		can_bus_off(ndev);
		ret = ch9431_hw_sleep(ch9431);
		if (ret)
			dev_warn(
				ndev->dev.parent,
				"failed to put controller to sleep after bus-off: %d\n",
				ret);
		goto out;
	}

	if (intf & CH9431_INTF_TX) {
		if (ch9431->tx_busy[0]) {
			ndev->stats.tx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
			ndev->stats.tx_bytes +=
				can_get_echo_skb(ndev, 0, NULL);
#else
			ndev->stats.tx_bytes += can_get_echo_skb(ndev, 0);
#endif
			ch9431->tx_busy[0] = false;
			ch9431_irq_dbg(ch9431, "v1 tx done slot=0\n");
		}
		netif_wake_queue(ndev);
	}

out:
	timeout = jiffies + usecs_to_jiffies(500);
	do {
		gpio_level = ch9431_irq_gpio_value(ch9431);
		if (gpio_level < 0) {
			if (!gpio_read_failed)
				dev_warn(
					ndev->dev.parent,
					"ch9431 IRQ GPIO read failed: %d\n",
					gpio_level);
			ch9431_irq_dbg(
				ch9431,
				"v1 gpio read failed intf=0x%02x errf=0x%02x ret=%d\n",
				intf, errf, gpio_level);
			gpio_read_failed = true;
			gpio_level = 0;
		}
		if (gpio_level)
			break;
		if (time_after(jiffies, timeout)) {
			if (gpio_read_failed)
				dev_warn(
					ndev->dev.parent,
					"ch9431 IRQ timeout after GPIO read errors\n");
			else
				dev_warn(
					ndev->dev.parent,
					"ch9431 IRQ timeout: GPIO stuck low\n");
			ch9431_irq_dbg(
				ch9431,
				"v1 gpio stuck low intf=0x%02x errf=0x%02x\n",
				intf, errf);
			break;
		}
		cpu_relax();
	} while (1);

	if (!ch9431->force_quit) {
		if (ch9431->rx0_flag)
			ch9431_enable_intr_except_rx0(ch9431);
		else
			ch9431_enable_intr(ch9431);
	}
	ch9431_irq_dbg(
		ch9431,
		"v1 irq done ret=%d gpio=%d force=%d state=%d rx0_flag=%d\n",
		ret, gpio_level, ch9431->force_quit, ch9431->can.state,
		ch9431->rx0_flag);

	mutex_unlock(&ch9431->ops_lock);

	return IRQ_HANDLED;
}

static irqreturn_t ch9431_v2_rx_threaded_irq(int irq, void *pw)
{
	struct ch9431_priv *ch9431 = pw;
	struct net_device *ndev = ch9431->ndev;
	enum can_state old_state, new_state;
	u8 intf = 0, errf = 0;
	u8 clear_intf = 0;
#if CH9431_DEBUG
	u8 tx_done_mask = 0;
#endif
	int can_id = 0, data1 = 0;
	int gpio_level = 0;
	bool gpio_read_failed = false;
	unsigned long timeout = 0;
	int ret;

	if (ch9431->force_quit) {
		ch9431_irq_dbg(ch9431, "v2 irq ignored force_quit\n");
		return IRQ_HANDLED;
	}

	mutex_lock(&ch9431->ops_lock);
	old_state = ch9431->can.state;
	ch9431_irq_dbg(ch9431, "v2 irq begin state=%d pending=0x%x\n",
		       old_state, ch9431->tx_pending);

	ret = ch9431_disable_intr(ch9431);
	if (unlikely(ret)) {
		ch9431_irq_dbg(ch9431, "v2 disable intr failed ret=%d\n",
			       ret);
		goto out;
	}

	ret = ch9431_read_2regs(ch9431, CH9431_INTF, &intf, &errf);
	if (unlikely(ret)) {
		ch9431_irq_dbg(ch9431, "v2 read intf/errf failed ret=%d\n",
			       ret);
		goto out;
	}
	clear_intf |= intf & (CH9431_INTF_TX | CH9431_INTF_ERR);
	ch9431_irq_dbg(ch9431,
		       "v2 irq intf=0x%02x errf=0x%02x clear=0x%02x\n",
		       intf, errf, clear_intf);

	/* Receive buffer 0 */
	if (intf & CH9431_INTF_RX0IF) {
		if (errf & CH9431_EFLAG_RX0OVR) {
			ch9431_irq_dbg(ch9431,
				       "v2 irq rx0 bulk overflow\n");
			ch9431_hw_rx_bulk(ch9431, 0);
		} else {
			ch9431_irq_dbg(ch9431, "v2 irq rx0\n");
			ch9431->dev_ops->hw_rx(ch9431, 0);
		}
	}

	/* Receive buffer 1 */
	if (intf & CH9431_INTF_RX1IF) {
		if (errf & CH9431_EFLAG_RX1OVR) {
			ch9431_irq_dbg(ch9431,
				       "v2 irq rx1 bulk overflow\n");
			ch9431_hw_rx_bulk(ch9431, 1);
		} else {
			ch9431_irq_dbg(ch9431, "v2 irq rx1\n");
			ch9431->dev_ops->hw_rx(ch9431, 1);
		}
	}

	if (clear_intf) {
		ret = ch9431_clear_intr(ch9431, clear_intf);
		if (unlikely(ret)) {
			ch9431_irq_dbg(
				ch9431,
				"v2 clear intf=0x%02x failed ret=%d\n",
				clear_intf, ret);
			goto out;
		}
	}

	if (errf & (CH9431_EFLAG_RX0OVR | CH9431_EFLAG_RX1OVR)) {
		ret = ch9431_clear_errf(ch9431, errf);
		if (unlikely(ret)) {
			ch9431_irq_dbg(
				ch9431,
				"v2 clear errf=0x%02x failed ret=%d\n",
				errf, ret);
			goto out;
		}
	}

	/* Update can state */
	if (errf & CH9431_EFLAG_TXBO) {
		new_state = CAN_STATE_BUS_OFF;
		can_id |= CAN_ERR_BUSOFF;
	} else if (errf & CH9431_EFLAG_TXEP) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_TX_PASSIVE;
	} else if (errf & CH9431_EFLAG_RXEP) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_RX_PASSIVE;
	} else if (errf & CH9431_EFLAG_TXWAR) {
		new_state = CAN_STATE_ERROR_WARNING;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_TX_WARNING;
	} else if (errf & CH9431_EFLAG_RXWAR) {
		new_state = CAN_STATE_ERROR_WARNING;
		can_id |= CAN_ERR_CRTL;
		data1 |= CAN_ERR_CRTL_RX_WARNING;
	} else {
		new_state = CAN_STATE_ERROR_ACTIVE;
	}

	/* Update can state statistics */
	switch (old_state) {
	case CAN_STATE_ERROR_ACTIVE:
		if (new_state >= CAN_STATE_ERROR_WARNING &&
		    new_state <= CAN_STATE_BUS_OFF)
			ch9431->can.can_stats.error_warning++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		fallthrough;
#endif
	case CAN_STATE_ERROR_WARNING: /* fallthrough */
		if (new_state >= CAN_STATE_ERROR_PASSIVE &&
		    new_state <= CAN_STATE_BUS_OFF)
			ch9431->can.can_stats.error_passive++;
		break;
	default:
		break;
	}

	ch9431->can.state = new_state;
	if (new_state != old_state || errf)
		ch9431_irq_dbg(ch9431,
			       "v2 state %d->%d can_id=0x%x data1=0x%x\n",
			       old_state, new_state, can_id, data1);

	if (new_state == CAN_STATE_BUS_OFF &&
	    old_state != CAN_STATE_BUS_OFF) {
		ch9431->force_quit = 1;
		ch9431->can.can_stats.bus_off++;
		ch9431_irq_dbg(ch9431, "v2 bus off, enter sleep\n");
		can_bus_off(ndev);
		ret = ch9431_hw_sleep(ch9431);
		if (ret)
			dev_warn(
				ndev->dev.parent,
				"failed to put controller to sleep after bus-off: %d\n",
				ret);
		goto out;
	}

	if (intf & CH9431_INTF_TX) {
		unsigned long flags;
		bool wake_queue = false;
		int i;

		spin_lock_irqsave(&ch9431->tx_lock, flags);

		for (i = 0; i < CH9431_TX_BUF_NUM; i++) {
			if (!(intf & (CH9431_INTF_TX0IF << i)))
				continue;

#if CH9431_DEBUG
			tx_done_mask |= BIT(i);
#endif
			if (ch9431->tx_busy[i]) {
				ndev->stats.tx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
				ndev->stats.tx_bytes +=
					can_get_echo_skb(ndev, i, NULL);
#else
				ndev->stats.tx_bytes +=
					can_get_echo_skb(ndev, i);
#endif
				ch9431->tx_busy[i] = false;
			}
			ch9431->tx_pending &= ~BIT(i);
		}

		if (!ch9431->force_quit &&
		    ch9431->can.state != CAN_STATE_BUS_OFF &&
		    ch9431->can.state != CAN_STATE_STOPPED &&
		    netif_queue_stopped(ndev) &&
		    ch9431->tx_pending != CH9431_TX_MASK)
			wake_queue = true;

		spin_unlock_irqrestore(&ch9431->tx_lock, flags);

		if (wake_queue)
			netif_wake_queue(ndev);
		ch9431_irq_dbg(
			ch9431,
			"v2 tx done mask=0x%x pending=0x%x wake=%d\n",
			tx_done_mask, ch9431->tx_pending, wake_queue);
	}

out:
	timeout = jiffies + usecs_to_jiffies(500);
	do {
		gpio_level = ch9431_irq_gpio_value(ch9431);
		if (gpio_level < 0) {
			if (!gpio_read_failed)
				dev_warn(
					ndev->dev.parent,
					"ch9431 IRQ GPIO read failed: %d\n",
					gpio_level);
			ch9431_irq_dbg(
				ch9431,
				"v2 gpio read failed intf=0x%02x errf=0x%02x ret=%d\n",
				intf, errf, gpio_level);
			gpio_read_failed = true;
			gpio_level = 0;
		}
		if (gpio_level)
			break;
		if (time_after(jiffies, timeout)) {
			if (gpio_read_failed)
				dev_warn(
					ndev->dev.parent,
					"ch9431 IRQ timeout after GPIO read errors\n");
			else
				dev_warn(
					ndev->dev.parent,
					"ch9431 IRQ timeout: GPIO stuck low\n");
			ch9431_irq_dbg(
				ch9431,
				"v2 gpio stuck low intf=0x%02x errf=0x%02x\n",
				intf, errf);
			break;
		}
		cpu_relax();
	} while (1);

	if (!ch9431->force_quit)
		ch9431_enable_intr(ch9431);
	ch9431_irq_dbg(
		ch9431,
		"v2 irq done ret=%d gpio=%d force=%d state=%d pending=0x%x\n",
		ret, gpio_level, ch9431->force_quit, ch9431->can.state,
		ch9431->tx_pending);
	mutex_unlock(&ch9431->ops_lock);

	return IRQ_HANDLED;
}

/*
 * Open can device
 * Called when the can device is marked active, such as a user executing
 * 'ifconfig candev up' on the device
 */
static int ch9431_open(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	struct spi_device *spi = ch9431->spi;
	bool transceiver_enabled = false;
	int i;
	int ret;

	ret = open_candev(ndev);
	if (ret) {
		dev_err(&spi->dev, "%s, Unable to init baudrate!\n",
			__func__);
		return ret;
	}

	mutex_lock(&ch9431->ops_lock);
	ch9431_flow_dbg(ch9431, "open begin irq=%d\n", spi->irq);

	ret = ch9431_power_enable(ch9431->transceiver, 1);
	if (ret)
		goto out_close;
	transceiver_enabled = true;

	ch9431->force_quit = 0;
	ch9431->tx_skb = NULL;
	ch9431->tx_pending = 0;
	ch9431->rx0_flag = false;
	for (i = 0; i < CH9431_TX_BUF_NUM; i++) {
		ch9431->tx_busy[i] = false;
	}

	ret = ch9431_hw_wake(ch9431);
	if (ret)
		goto out_close;

	ret = ch9431->dev_ops->dev_setup(ch9431);
	if (ret)
		goto out_close;

	ret = ch9431_set_normal_mode(ch9431);
	if (ret)
		goto out_close;

	netif_wake_queue(ndev);
	ch9431_flow_dbg(ch9431, "open done state=%d pending=0x%x\n",
			ch9431->can.state, ch9431->tx_pending);
	mutex_unlock(&ch9431->ops_lock);

	return 0;

out_close:
	ch9431_hw_sleep(ch9431);
	if (transceiver_enabled)
		ch9431_power_enable(ch9431->transceiver, 0);
	close_candev(ndev);
	netdev_err(ndev, "%s : ch9431 open failed, ret: %d\n", __func__,
		   ret);
	ch9431_flow_dbg(ch9431, "open failed ret=%d\n", ret);
	mutex_unlock(&ch9431->ops_lock);
	return ret;
}

static int ch9431_hw_probe(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	u8 ctrl;
	int ret;

	ret = ch9431_hw_reset(ch9431);
	if (ret)
		return ret;

	ret = ch9431_read_reg_raw(ch9431, CH9431_CTRL, &ctrl);
	if (ret)
		return ret;

	dev_dbg(&spi->dev, "%s, CH9431_CTRL 0x%02x\n", __func__, ctrl);

	/* Check for power up default value */
	if ((ctrl & 0x17) != 0x07)
		return -ENODEV;

	ret = ch9431_get_version(ch9431);
	if (ret)
		return ret;

	return 0;
}

/*
 * Close can device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int ch9431_stop(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	int err;
	int ret = 0;

	netif_stop_queue(ndev);
	ch9431->force_quit = 1;
	ch9431->restart_tx = 0;
	ch9431->after_suspend = 0;
	ch9431_flow_dbg(ch9431, "stop begin state=%d pending=0x%x\n",
			ch9431->can.state, ch9431->tx_pending);

	cancel_work_sync(&ch9431->restart_work);
	cancel_work_sync(&ch9431->tx_work);
	flush_workqueue(ch9431->wq);

	close_candev(ndev);

	mutex_lock(&ch9431->ops_lock);

	/* Disable and clear pending interrupts */
	err = ch9431_write_2regs(ch9431, CH9431_INTE, 0x00, 0x00);
	if (err && !ret)
		ret = err;

	err = ch9431_clear_tx_buffers(ch9431);
	if (err && !ret)
		ret = err;
	ch9431->dev_ops->tx_clean(ndev);
	err = ch9431_hw_reset(ch9431);
	if (err && !ret)
		ret = err;
	err = ch9431_hw_sleep(ch9431);
	if (err && !ret)
		ret = err;
	err = ch9431_power_enable(ch9431->transceiver, 0);
	if (err && !ret)
		ret = err;
	ch9431->can.state = CAN_STATE_STOPPED;
	ch9431_flow_dbg(ch9431, "stop done ret=%d pending=0x%x\n", ret,
			ch9431->tx_pending);

	mutex_unlock(&ch9431->ops_lock);

	return ret;
}

static netdev_tx_t ch9431_v1_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	if (ch9431->tx_skb || ch9431->tx_busy[0]) {
		dev_warn(&ch9431->spi->dev,
			 "hard_xmit called while tx busy\n");
		ch9431_tx_dbg(ch9431, "v1 xmit busy tx_skb=%p busy=%d\n",
			      ch9431->tx_skb, ch9431->tx_busy[0]);
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);
	ch9431->tx_skb = skb;
	queue_work(ch9431->wq, &ch9431->tx_work);
	ch9431_tx_dbg(ch9431, "v1 xmit queued\n");

	return NETDEV_TX_OK;
}

static netdev_tx_t ch9431_v2_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	struct ch9431_tx_agg *tx_agg;
	int tx_buff_id;

	tx_buff_id = ch9431_v2_reserve_tx_slot(ch9431);
	if (tx_buff_id < 0) {
		ch9431_tx_dbg(ch9431, "v2 xmit busy pending=0x%x\n",
			      ch9431->tx_pending);
		return NETDEV_TX_BUSY;
	}

	tx_agg = kmalloc(sizeof(*tx_agg), GFP_ATOMIC);
	if (!tx_agg) {
		netdev_err(ndev, "failed to allocate tx work item\n");
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		ch9431_v2_release_tx_slot(ch9431, tx_buff_id);
		return NETDEV_TX_OK;
	}

	INIT_WORK(&tx_agg->work, ch9431_v2_tx_delay);
	tx_agg->dev = ch9431;
	tx_agg->skb = skb;
	tx_agg->tx_buff_id = tx_buff_id;
	queue_work(ch9431->wq, &tx_agg->work);
	ch9431_tx_dbg(ch9431, "v2 xmit queued slot=%d pending=0x%x\n",
		      tx_buff_id, ch9431->tx_pending);

	return NETDEV_TX_OK;
}

static netdev_tx_t ch9431_start_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	return ch9431->dev_ops->start_xmit(skb, ndev);
}

static int ch9431_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	switch (mode) {
	case CAN_MODE_START:
		ch9431->dev_ops->tx_clean(ndev);
		/* We have to delay work since SPI I/O may sleep */
		ch9431->can.state = CAN_STATE_ERROR_ACTIVE;
		ch9431->restart_tx = 1;
		ch9431->after_suspend = AFTER_SUSPEND_RESTART;
		queue_work(ch9431->wq, &ch9431->restart_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops ch9431_netdev_ops = {
	.ndo_open = ch9431_open,
	.ndo_stop = ch9431_stop,
	.ndo_start_xmit = ch9431_start_xmit,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	.ndo_change_mtu = can_change_mtu,
#endif
};

static ssize_t reg_dump_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct ch9431_priv *ch9431 = NULL;
	int i, len = 0;
	u8 val;

	dev_info(dev, "reg_dump_show");
	if (!spi) {
		dev_err(dev, "%s, spi_device is NULL\n", __func__);
		return -EINVAL;
	}

	ch9431 = dev_get_drvdata(dev);

	if (!ch9431) {
		dev_err(dev, "%s, ch9431 priv is NULL\n", __func__);
		return -EINVAL;
	}

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "netif_queue: %s, tx_pending: 0x%x\n",
			 netif_queue_stopped(ch9431->ndev) ? "stopped" :
							     "running",
			 ch9431->tx_pending);
	len += scnprintf(
		buf + len, PAGE_SIZE - len,
		"tx_busy[0]: %d, tx_busy[1]: %d, tx_busy[2]: %d\n",
		ch9431->tx_busy[0], ch9431->tx_busy[1],
		ch9431->tx_busy[2]);

	for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]); i++) {
		val = ch9431_read_reg(ch9431, reg_labels[i].reg);
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%s: 0x%02x\n", reg_labels[i].name, val);
		if (len >= PAGE_SIZE)
			break;
	}

	return len;
}

static ssize_t reg_dump_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct ch9431_priv *ch9431;
	u8 reg;
	u8 val;
	char reg_name[32];
	int ret;

	dev_info(dev, "reg_dump_store\n");
	if (!spi) {
		dev_err(dev, "%s, spi_device is NULL\n", __func__);
		return -EINVAL;
	}

	ch9431 = dev_get_drvdata(dev);
	if (!ch9431) {
		dev_err(dev, "%s, ch9431 priv is NULL\n", __func__);
		return -ENODEV;
	}

	if (sscanf(buf, "%31s %02hhx", reg_name, &val) == 2) {
		int i;

		for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]);
		     i++) {
			if (strcmp(reg_labels[i].name, reg_name) == 0) {
				reg = reg_labels[i].reg;
				mutex_lock(&ch9431->ops_lock);
				ret = ch9431_write_reg(ch9431, reg, val);
				mutex_unlock(&ch9431->ops_lock);
				if (ret < 0)
					dev_info(
						dev,
						"set reg: 0x%02x - value: 0x%02x failed!\n",
						reg, val);
				else
					dev_info(
						dev,
						"set reg: 0x%02x - value: 0x%02x success!\n",
						reg, val);
				break;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show,
		   reg_dump_store);

static struct attribute *ch9431_attributes[] = { &dev_attr_reg_dump.attr,
						 NULL };

static struct attribute_group ch9431_attribute_group = {
	.attrs = ch9431_attributes
};

static int ch9431_create_sysfs(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	char *link_name = ch9431->link_name;
	u8 cs_num;
	const char *ctrl_name;
	int ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0))
	ctrl_name = dev_name(&spi->controller->dev);
	cs_num = spi->chip_select[0];
#else
	ctrl_name = dev_name(&spi->master->dev);
	cs_num = spi->chip_select;
#endif

	ret = sysfs_create_group(&spi->dev.kobj, &ch9431_attribute_group);
	if (ret != 0) {
		dev_err(&spi->dev, "sysfs_create_group() failed!!");
		return -EIO;
	}

	snprintf(link_name, sizeof(ch9431->link_name), "ch9431-%s-%d",
		 ctrl_name, cs_num);

	ret = sysfs_create_link(NULL, &spi->dev.kobj, link_name);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to create link: %s", link_name);
		sysfs_remove_group(&spi->dev.kobj,
				   &ch9431_attribute_group);
		return -EIO;
	}

	dev_info(&spi->dev, "sysfs_create_group() and link %s succeeded!",
		 link_name);
	ch9431->sysfs_created = true;

	return ret;
}

static int ch9431_request_irq(struct ch9431_priv *ch9431)
{
	struct net_device *ndev = ch9431->ndev;
	struct spi_device *spi = ch9431->spi;
	int gpio_irq_num = -1;
	int ret;
	unsigned long flags = IRQF_TRIGGER_LOW;

	/* if your platform supports acquire irq number from dts */
#ifdef USE_IRQ_FROM_DTS
	/* get gpio number by dts */
	if (!spi->dev.of_node) {
		ret = -ENODEV;
		dev_err(&spi->dev, "missing device tree node\n");
		goto out;
	}

	gpio_irq_num =
		of_get_named_gpio(spi->dev.of_node, "interrupt-gpio", 0);
	if (gpio_irq_num < 0)
		gpio_irq_num = of_get_named_gpio(spi->dev.of_node,
						 "interrupt-gpios", 0);
	if (gpio_irq_num < 0) {
		ret = gpio_irq_num;
		dev_err(&spi->dev, "failed to get interrupt GPIO: %d\n",
			ret);
		goto out;
	}
#else
	gpio_irq_num = GPIO_NUMBER;
#endif
	if (!gpio_is_valid(gpio_irq_num)) {
		ret = -EINVAL;
		dev_err(&spi->dev, "invalid gpio number: %d\n",
			gpio_irq_num);
		goto out;
	}

	ret = devm_gpio_request(&spi->dev, gpio_irq_num, "gpioint");
	if (ret) {
		dev_err(&spi->dev, "gpio_request fail.\n");
		goto out;
	}

	ret = gpio_direction_input(gpio_irq_num);
	if (ret) {
		dev_err(&spi->dev, "gpio_direction_input fail.\n");
		goto out;
	}

	spi->irq = gpio_to_irq(gpio_irq_num);
	if (spi->irq < 0) {
		ret = spi->irq;
		dev_err(&spi->dev, "gpio_to_irq fail: %d\n", ret);
		goto out;
	}

	ret = irq_set_irq_type(spi->irq, flags);
	if (ret) {
		dev_err(&spi->dev, "irq_set_irq_type fail: %d\n", ret);
		goto out;
	}

	ch9431->gpio_irq_num = gpio_irq_num;
	ndev->irq = spi->irq;
	ret = request_threaded_irq(spi->irq, NULL,
				   ch9431->dev_ops->irq_handler,
				   flags | IRQF_ONESHOT,
				   dev_name(&spi->dev), ch9431);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
		goto out;
	}

	ch9431_irq_dbg(ch9431, "request irq=%d gpio=%d flags=0x%lx\n",
		       spi->irq, gpio_irq_num, flags);

out:
	return ret;
}

static void ch9431_hw_init(struct ch9431_priv *ch9431)
{
	static const u8 fw_v102a[] = { 'V', '1', '.', '0', '.', '2', 'a' };
	int version = -1;

	version = memcmp(ch9431->fw_version, fw_v102a, sizeof(fw_v102a));

	switch (version) {
	case 0:
		ch9431->dev_ops = &ch9431_v2_ops;
		ch9431_flow_dbg(ch9431, "select v2 ops fw=%.*s\n",
				(int)sizeof(ch9431->fw_version),
				ch9431->fw_version);
		break;
	default:
		ch9431->dev_ops = &ch9431_v1_ops;
		ch9431_flow_dbg(ch9431, "select v1 ops fw=%.*s\n",
				(int)sizeof(ch9431->fw_version),
				ch9431->fw_version);
		break;
	}

	return;
}

static int ch9431_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct ch9431_priv *ch9431;
	int ret = 0;

	/* Allocate can/net device */
	ndev = alloc_candev(sizeof(struct ch9431_priv), CH9431_TX_BUF_NUM);
	if (!ndev)
		return -ENOMEM;

	ndev->netdev_ops = &ch9431_netdev_ops;
	ndev->ethtool_ops = &ch9431_ethtool_ops;
	ndev->flags |= IFF_ECHO;

	ch9431 = netdev_priv(ndev);
	ch9431->can.bittiming_const = &ch9431_bittiming_const;
	ch9431->can.do_set_mode = ch9431_do_set_mode;
	ch9431->can.clock.freq = CH9431_CLK_FREQ;
	ch9431->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					 CAN_CTRLMODE_LISTENONLY;
	ch9431->ndev = ndev;
	ch9431->spi = spi;
	ch9431->force_quit = 1;

	spi_set_drvdata(spi, ch9431);

	mutex_init(&ch9431->ops_lock);
	mutex_init(&ch9431->reg_lock);
	spin_lock_init(&ch9431->tx_lock);

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	spi->max_speed_hz = spi->max_speed_hz ?: 12 * 1000 * 1000;
	ret = spi_setup(spi);
	if (ret)
		goto out_free;

	ret = ch9431_get_optional_regulator(dev, "vdd", &ch9431->power);
	if (ret)
		goto out_free;

	ret = ch9431_get_optional_regulator(dev, "xceiver",
					    &ch9431->transceiver);
	if (ret)
		goto out_free;

	ret = ch9431_power_enable(ch9431->power, 1);
	if (ret)
		goto out_free;

	ch9431->wq = alloc_ordered_workqueue("ch9431_wq", WQ_MEM_RECLAIM);
	if (!ch9431->wq) {
		ret = -ENOMEM;
		goto out_poweroff;
	}

	ch9431->spi_tx_buf =
		devm_kzalloc(&spi->dev, CH9431_SPI_TX_BUF_LEN, GFP_KERNEL);
	if (!ch9431->spi_tx_buf) {
		ret = -ENOMEM;
		goto error_probe;
	}

	ch9431->spi_rx_buf =
		devm_kzalloc(&spi->dev, CH9431_SPI_RX_BUF_LEN, GFP_KERNEL);
	if (!ch9431->spi_rx_buf) {
		ret = -ENOMEM;
		goto error_probe;
	}

	ret = ch9431_hw_probe(ch9431);
	if (ret) {
		if (ret == -ENODEV)
			dev_err(dev,
				"Cannot initialize ch9431. Wrong wiring?\n");
		goto error_probe;
	}

	ch9431_hw_init(ch9431);

	INIT_WORK(&ch9431->tx_work, ch9431_v1_tx_delay);
	INIT_WORK(&ch9431->restart_work, ch9431_restart_tx);

	SET_NETDEV_DEV(ndev, dev);

	ret = ch9431_hw_sleep(ch9431);
	if (ret)
		goto error_probe;

	ret = ch9431_request_irq(ch9431);
	if (ret)
		goto error_probe;

	ret = register_candev(ndev);
	if (ret)
		goto out_free_irq;

	ret = ch9431_create_sysfs(ch9431);
	if (ret)
		goto out_unregister_candev;

	return 0;

out_unregister_candev:
	unregister_candev(ndev);

out_free_irq:
	free_irq(spi->irq, ch9431);

error_probe:
	destroy_workqueue(ch9431->wq);
	ch9431->wq = NULL;

out_poweroff:
	ch9431_power_enable(ch9431->power, 0);

out_free:
	free_candev(ndev);

	dev_err(dev, "ch9431_probe failed, err=%d\n", -ret);
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch9431_drv_remove(struct spi_device *spi)
{
	struct ch9431_priv *ch9431 = spi_get_drvdata(spi);
	struct net_device *net = ch9431->ndev;

	if (ch9431->sysfs_created) {
		sysfs_remove_group(&spi->dev.kobj,
				   &ch9431_attribute_group);
		sysfs_remove_link(NULL, ch9431->link_name);
		ch9431->sysfs_created = false;
	}

	unregister_candev(net);

	free_irq(ch9431->spi->irq, ch9431);

	ch9431_power_enable(ch9431->power, 0);

	destroy_workqueue(ch9431->wq);
	ch9431->wq = NULL;

	free_candev(net);

	dev_info(&spi->dev, "ch9431 CAN device driver removed\n");
}
#else
static int ch9431_drv_remove(struct spi_device *spi)
{
	struct ch9431_priv *ch9431 = spi_get_drvdata(spi);
	struct net_device *net = ch9431->ndev;

	if (ch9431->sysfs_created) {
		sysfs_remove_group(&spi->dev.kobj,
				   &ch9431_attribute_group);
		sysfs_remove_link(NULL, ch9431->link_name);
		ch9431->sysfs_created = false;
	}

	unregister_candev(net);

	free_irq(ch9431->spi->irq, ch9431);

	ch9431_power_enable(ch9431->power, 0);

	destroy_workqueue(ch9431->wq);
	ch9431->wq = NULL;

	free_candev(net);

	dev_info(&spi->dev, "ch9431 CAN device driver removed\n");

	return 0;
}
#endif

static const struct of_device_id ch9431_match_table[] = {
	{ .compatible = "wch,ch9431" },
	{}
};

MODULE_DEVICE_TABLE(of, ch9431_match_table);

static const struct spi_device_id ch9431_id_table[] = {
	{
		.name = "ch9431",
		.driver_data = (kernel_ulong_t)0x9431,
	},
	{}
};
MODULE_DEVICE_TABLE(spi, ch9431_id_table);

static struct spi_driver ch9431_driver = {
	.driver = {
		.name = DRVNAME_CH9431,
		.of_match_table = ch9431_match_table,
	},
	.id_table = ch9431_id_table,
	.probe = ch9431_probe,
	.remove = ch9431_drv_remove,
};
module_spi_driver(ch9431_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
