/**
 * Copyright (C) 2014-2016 Institut de Recherche Technologique SystemX and OpenWide.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * @file imx_serial.c
 * @author Jimmy Durand Wesolowski (jimmy.durand-wesolowski@openwide.fr)
 * @author Jean Guyomarc'h (jean.guyomarch@openwide.fr)
 * @brief Motorola/Freescale i.MX serial emulator.
 * @details This source file implements the i.MX serial emulator.
 *
 * The source has been largely adapted from PL011 emulator.
 */

#include <vmm_error.h>
#include <vmm_heap.h>
#include <vmm_modules.h>
#include <vmm_devtree.h>
#include <vmm_devemu.h>
#include <vio/vmm_vserial.h>
#include <libs/fifo.h>
#include <libs/stringlib.h>

#define MODULE_DESC			"Zynq Serial Emulator"
#define MODULE_AUTHOR			"João Pedro Barros Silva"
#define MODULE_LICENSE			"GPL"
#define MODULE_IPRIORITY		(VMM_VSERIAL_IPRIORITY+1)
#define	MODULE_INIT			zynq_emulator_init
#define	MODULE_EXIT			zynq_emulator_exit

#define CADENCE_UART_RX_FIFO_SIZE           16
#define CADENCE_UART_TX_FIFO_SIZE           16

#define CADENCE_UART_R_MAX (0x48/4)

//19.3.3 - Zynq-7000 TRM
#define ZYNQ_FIFO_SIZE			64// 64 bytes (the size of the TxFIFO)

#define UART_SR_INTR_RTRIG     0x00000001
#define UART_SR_INTR_REMPTY    0x00000002
#define UART_SR_INTR_RFUL      0x00000004
#define UART_SR_INTR_TEMPTY    0x00000008
#define UART_SR_INTR_TFUL      0x00000010
/* somewhat awkwardly, TTRIG is misaligned between SR and ISR */
#define UART_SR_TTRIG          0x00002000
#define UART_INTR_TTRIG        0x00000400
/* bits fields in CSR that correlate to CISR. If any of these bits are set in
 * SR, then the same bit in CISR is set high too */
#define UART_SR_TO_CISR_MASK   0x0000001F

#define UART_INTR_ROVR         0x00000020
#define UART_INTR_FRAME        0x00000040
#define UART_INTR_PARE         0x00000080
#define UART_INTR_TIMEOUT      0x00000100
#define UART_INTR_DMSI         0x00000200
#define UART_INTR_TOVR         0x00001000

#define UART_SR_RACTIVE    0x00000400
#define UART_SR_TACTIVE    0x00000800
#define UART_SR_FDELT      0x00001000

#define UART_CR_RXRST       0x00000001
#define UART_CR_TXRST       0x00000002
#define UART_CR_RX_EN       0x00000004
#define UART_CR_RX_DIS      0x00000008
#define UART_CR_TX_EN       0x00000010
#define UART_CR_TX_DIS      0x00000020
#define UART_CR_RST_TO      0x00000040
#define UART_CR_STARTBRK    0x00000080
#define UART_CR_STOPBRK     0x00000100

#define UART_MR_CLKS            0x00000001
#define UART_MR_CHRL            0x00000006
#define UART_MR_CHRL_SH         1
#define UART_MR_PAR             0x00000038
#define UART_MR_PAR_SH          3
#define UART_MR_NBSTOP          0x000000C0
#define UART_MR_NBSTOP_SH       6
#define UART_MR_CHMODE          0x00000300
#define UART_MR_CHMODE_SH       8
#define UART_MR_UCLKEN          0x00000400
#define UART_MR_IRMODE          0x00000800

#define UART_DATA_BITS_6       (0x3 << UART_MR_CHRL_SH)
#define UART_DATA_BITS_7       (0x2 << UART_MR_CHRL_SH)
#define UART_PARITY_ODD        (0x1 << UART_MR_PAR_SH)
#define UART_PARITY_EVEN       (0x0 << UART_MR_PAR_SH)
#define UART_STOP_BITS_1       (0x3 << UART_MR_NBSTOP_SH)
#define UART_STOP_BITS_2       (0x2 << UART_MR_NBSTOP_SH)
#define NORMAL_MODE            (0x0 << UART_MR_CHMODE_SH)
#define ECHO_MODE              (0x1 << UART_MR_CHMODE_SH)
#define LOCAL_LOOPBACK         (0x2 << UART_MR_CHMODE_SH)
#define REMOTE_LOOPBACK        (0x3 << UART_MR_CHMODE_SH)

#define UART_INPUT_CLK         50000000

#define R_CR       (0x00/4)
#define R_MR       (0x04/4)
#define R_IER      (0x08/4)
#define R_IDR      (0x0C/4)
#define R_IMR      (0x10/4)
#define R_CISR     (0x14/4)
#define R_BRGR     (0x18/4)
#define R_RTOR     (0x1C/4)
#define R_RTRIG    (0x20/4)
#define R_MCR      (0x24/4)
#define R_MSR      (0x28/4)
#define R_SR       (0x2C/4)
#define R_TX_RX    (0x30/4)
#define R_BDIV     (0x34/4)
#define R_FDEL     (0x38/4)
#define R_PMIN     (0x3C/4)
#define R_PWID     (0x40/4)
#define R_TTRIG    (0x44/4)

/* Register definitions */
#define URXD0 0x0  /* Receiver Register */
#define URTX0 0x40 /* Transmitter Register */
#define UCR1  0x80 /* Control Register 1 */
#define UCR2  0x84 /* Control Register 2 */
#define UCR3  0x88 /* Control Register 3 */
#define UCR4  0x8c /* Control Register 4 */
#define UFCR  0x90 /* FIFO Control Register */
#define USR1  0x94 /* Status Register 1 */
#define USR2  0x98 /* Status Register 2 */
#define UESC  0x9c /* Escape Character Register */
#define UTIM  0xa0 /* Escape Timer Register */
#define UBIR  0xa4 /* BRM Incremental Register */
#define UBMR  0xa8 /* BRM Modulator Register */
#define UBRC  0xac /* Baud Rate Count Register */
#define IMX21_ONEMS 0xb0 /* One Millisecond register */

#define IMX_FIFO_SIZE			32

#define IMX1_UTS 0xd0 /* UART Test Register on i.mx1 */
#define IMX21_UTS 0xb4 /* UART Test Register on all other i.mx*/

/* UART Control Register Bit Fields.*/
#define URXD_DUMMY_READ (1<<16)
#define URXD_CHARRDY	(1<<15)
#define URXD_ERR	(1<<14)
#define URXD_OVRRUN	(1<<13)
#define URXD_FRMERR	(1<<12)
#define URXD_BRK	(1<<11)
#define URXD_PRERR	(1<<10)
#define URXD_RX_DATA	(0xFF<<0)
#define UCR1_ADEN	(1<<15) /* Auto detect interrupt */
#define UCR1_ADBR	(1<<14) /* Auto detect baud rate */
#define UCR1_TRDYEN	(1<<13) /* Transmitter ready interrupt enable */
#define UCR1_IDEN	(1<<12) /* Idle condition interrupt */
#define UCR1_ICD_REG(x) (((x) & 3) << 10) /* idle condition detect */
#define UCR1_RRDYEN	(1<<9)	/* Recv ready interrupt enable */
#define UCR1_RDMAEN	(1<<8)	/* Recv ready DMA enable */
#define UCR1_IREN	(1<<7)	/* Infrared interface enable */
#define UCR1_TXMPTYEN	(1<<6)	/* Transimitter empty interrupt enable */
#define UCR1_RTSDEN	(1<<5)	/* RTS delta interrupt enable */
#define UCR1_SNDBRK	(1<<4)	/* Send break */
#define UCR1_TDMAEN	(1<<3)	/* Transmitter ready DMA enable */
#define IMX1_UCR1_UARTCLKEN (1<<2) /* UART clock enabled, i.mx1 only */
#define UCR1_ATDMAEN    (1<<2)  /* Aging DMA Timer Enable */
#define UCR1_DOZE	(1<<1)	/* Doze */
#define UCR1_UARTEN	(1<<0)	/* UART enabled */
#define UCR2_ESCI	(1<<15)	/* Escape seq interrupt enable */
#define UCR2_IRTS	(1<<14)	/* Ignore RTS pin */
#define UCR2_CTSC	(1<<13)	/* CTS pin control */
#define UCR2_CTS	(1<<12)	/* Clear to send */
#define UCR2_ESCEN	(1<<11)	/* Escape enable */
#define UCR2_PREN	(1<<8)	/* Parity enable */
#define UCR2_PROE	(1<<7)	/* Parity odd/even */
#define UCR2_STPB	(1<<6)	/* Stop */
#define UCR2_WS		(1<<5)	/* Word size */
#define UCR2_RTSEN	(1<<4)	/* Request to send interrupt enable */
#define UCR2_ATEN	(1<<3)	/* Aging Timer Enable */
#define UCR2_TXEN	(1<<2)	/* Transmitter enabled */
#define UCR2_RXEN	(1<<1)	/* Receiver enabled */
#define UCR2_SRST	(1<<0)	/* SW reset */
#define UCR3_DTREN	(1<<13) /* DTR interrupt enable */
#define UCR3_PARERREN	(1<<12) /* Parity enable */
#define UCR3_FRAERREN	(1<<11) /* Frame error interrupt enable */
#define UCR3_DSR	(1<<10) /* Data set ready */
#define UCR3_DCD	(1<<9)	/* Data carrier detect */
#define UCR3_RI		(1<<8)	/* Ring indicator */
#define UCR3_ADNIMP	(1<<7)	/* Autobaud Detection Not Improved */
#define UCR3_RXDSEN	(1<<6)	/* Receive status interrupt enable */
#define UCR3_AIRINTEN	(1<<5)	/* Async IR wake interrupt enable */
#define UCR3_AWAKEN	(1<<4)	/* Async wake interrupt enable */
#define IMX21_UCR3_RXDMUXSEL	(1<<2)	/* RXD Muxed Input Select */
#define UCR3_INVT	(1<<1)	/* Inverted Infrared transmission */
#define UCR3_BPEN	(1<<0)	/* Preset registers enable */
#define UCR4_CTSTL_SHF	10	/* CTS trigger level shift */
#define UCR4_CTSTL_MASK	0x3F	/* CTS trigger is 6 bits wide */
#define UCR4_INVR	(1<<9)	/* Inverted infrared reception */
#define UCR4_ENIRI	(1<<8)	/* Serial infrared interrupt enable */
#define UCR4_WKEN	(1<<7)	/* Wake interrupt enable */
#define UCR4_REF16	(1<<6)	/* Ref freq 16 MHz */
#define UCR4_IDDMAEN    (1<<6)  /* DMA IDLE Condition Detected */
#define UCR4_IRSC	(1<<5)	/* IR special case */
#define UCR4_TCEN	(1<<3)	/* Transmit complete interrupt enable */
#define UCR4_BKEN	(1<<2)	/* Break condition interrupt enable */
#define UCR4_OREN	(1<<1)	/* Receiver overrun interrupt enable */
#define UCR4_DREN	(1<<0)	/* Recv data ready interrupt enable */
#define UFCR_RXTL_SHF	0	/* Receiver trigger level shift */
#define UFCR_DCEDTE	(1<<6)	/* DCE/DTE mode select */
#define UFCR_RFDIV	(7<<7)	/* Reference freq divider mask */
#define UFCR_RFDIV_REG(x)	(((x) < 7 ? 6 - (x) : 6) << 7)
#define UFCR_TXTL_SHF	10	/* Transmitter trigger level shift */
#define USR1_PARITYERR	(1<<15) /* Parity error interrupt flag */
#define USR1_RTSS	(1<<14) /* RTS pin status */
#define USR1_TRDY	(1<<13) /* Transmitter ready interrupt/dma flag */
#define USR1_RTSD	(1<<12) /* RTS delta */
#define USR1_ESCF	(1<<11) /* Escape seq interrupt flag */
#define USR1_FRAMERR	(1<<10) /* Frame error interrupt flag */
#define USR1_RRDY	(1<<9)	 /* Receiver ready interrupt/dma flag */
#define USR1_AGTIM	(1<<8)   /* Ageing timer interrfupt flag */
#define USR1_TIMEOUT	(1<<7)	 /* Receive timeout interrupt status */
#define USR1_RXDS	 (1<<6)	 /* Receiver idle interrupt flag */
#define USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define USR1_AWAKE	 (1<<4)	 /* Aysnc wake interrupt flag */
#define USR2_ADET	 (1<<15) /* Auto baud rate detect complete */
#define USR2_TXFE	 (1<<14) /* Transmit buffer FIFO empty */
#define USR2_DTRF	 (1<<13) /* DTR edge interrupt flag */
#define USR2_IDLE	 (1<<12) /* Idle condition */
#define USR2_IRINT	 (1<<8)	 /* Serial infrared interrupt flag */
#define USR2_WAKE	 (1<<7)	 /* Wake */
#define USR2_RTSF	 (1<<4)	 /* RTS edge interrupt flag */
#define USR2_TXDC	 (1<<3)	 /* Transmitter complete */
#define USR2_BRCD	 (1<<2)	 /* Break condition */
#define USR2_ORE	(1<<1)	 /* Overrun error */
#define USR2_RDR	(1<<0)	 /* Recv data ready */
#define UTS_FRCPERR	(1<<13) /* Force parity error */
#define UTS_LOOP	(1<<12)	 /* Loop tx and rx */
#define UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define UTS_TXFULL	 (1<<4)	 /* TxFIFO full */
#define UTS_RXFULL	 (1<<3)	 /* RxFIFO full */
#define UTS_SOFTRST	 (1<<0)	 /* Software reset */

#define USR1_WR_MASK	(USR1_PARITYERR | USR1_RTSD | USR1_ESCF |   \
			 USR1_FRAMERR | USR1_AGTIM | USR1_TIMEOUT | \
			 USR1_AIRINT | USR1_AWAKE)
#define USR2_WR_MASK	(USR2_ADET | USR2_DTRF | USR2_IDLE | (1 << 11) | \
			 (1 << 10) | USR2_IRINT | USR2_WAKE | (1 << 6) | \
			 USR2_RTSF | USR2_BRCD | USR2_ORE)
#define UTS_WR_MASK	(UTS_FRCPERR | UTS_LOOP | (7 << 9) | UTS_TXEMPTY \
			 | UTS_RXEMPTY | UTS_TXFULL | UTS_RXFULL | UTS_SOFTRST)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct zynq_state {
	struct vmm_guest *guest;
	struct vmm_vserial *vser;
	vmm_spinlock_t lock;
	u32 irq;
    u32 r[CADENCE_UART_R_MAX];
    u8 r_fifo[CADENCE_UART_RX_FIFO_SIZE];
    u32 rx_wpos;
    u32 rx_count;
    u64 char_tx_time;
	struct fifo *rd_fifo;
};

struct imx_state {
	struct vmm_guest *guest;
	struct vmm_vserial *vser;
	vmm_spinlock_t lock;
	const struct imx_uart_data *data;
	u32 txirq;
	u32 rdirq;
	struct fifo *rd_fifo;
	u16 regs[(IMX21_ONEMS - UCR1) / 4 + 1];
	u16 uts;
	u8 tx;
};

/* i.MX21 type uart runs on all i.mx except i.MX1 and i.MX6q */
enum imx_uart_type {
	IMX1_UART,
	IMX21_UART,
	IMX6Q_UART,
};

/* device type dependent stuff */
struct imx_uart_data {
	unsigned uts_reg;
	enum imx_uart_type devtype;
};

//DONE
static void uart_update_status(struct vmm_emudev *edev)
{

	struct zynq_state *s = edev->priv;
    s->r[R_CISR] |= s->r[R_SR] & UART_SR_TO_CISR_MASK;
    zynq_set_irq(s, !!(s->r[R_IMR] & s->r[R_CISR]));

}

static inline unsigned int _reg_offset_get(u32 reg)
{
        return (reg - UCR1) / 4;
}

static u16 _reg_read(const struct imx_state *s, u32 reg)
{
	const unsigned int offset = _reg_offset_get(reg);

	BUG_ON(offset >= ARRAY_SIZE(s->regs));
	return s->regs[offset];
}

static bool _txfe_irq_enabled(const struct imx_state *s)
{
	/* NOTE: we should also check for USR2_TXFE to be 1.
	 * If we want to emulate the hardware queues (which are not
	 * handled at the moment), we should add the following test:
	 *	_reg_read(s, USR2) & USR2_TXFE
	 */
	return (_reg_read(s, UCR1) & UCR1_TXMPTYEN);
}

static void _reg_set_mask(struct imx_state *s, u32 reg, u16 mask)
{
	const unsigned int offset = _reg_offset_get(reg);

	BUG_ON(offset >= ARRAY_SIZE(s->regs));
	s->regs[offset] |= mask;
}

static void _reg_clear_mask(struct imx_state *s, u32 reg, u16 mask)
{
	const unsigned int offset = _reg_offset_get(reg);

	BUG_ON(offset >= ARRAY_SIZE(s->regs));
	s->regs[offset] &= ~mask;
}

static void _reg_ack(struct imx_state *s, u32 reg, u16 mask)
{
	u16 tmp = 0;
	const unsigned int offset = _reg_offset_get(reg);

	BUG_ON(offset >= ARRAY_SIZE(s->regs));
	tmp = s->regs[offset] & mask;
	s->regs[offset] &= ~tmp;
}

static void imx_set_rdirq(struct imx_state *s, int level)
{
	vmm_devemu_emulate_irq(s->guest, s->rdirq, level);
}

//DONE
static void uart_read_rx_fifo(struct zynq_state *s, u32 *c)
{
    if ((s->r[R_CR] & UART_CR_RX_DIS) || !(s->r[R_CR] & UART_CR_RX_EN)) {
        return;
    }

    if (s->rx_count) {
        uint32_t rx_rpos = (CADENCE_UART_RX_FIFO_SIZE + s->rx_wpos -
                            s->rx_count) % CADENCE_UART_RX_FIFO_SIZE;
        *c = s->rx_fifo[rx_rpos];
        s->rx_count--;

    } else {
        *c = 0;
    }

    uart_update_status(s);
}

//DONE
static int zynq_reg_read(struct zynq_state *s, u32 offset, u32 *dst)
{

	offset >>= 2;
	if (offset >= CADENCE_UART_R_MAX) {
		*dst = 0;
	} else if (offset == R_TX_RX) {
		uart_read_rx_fifo(s, dst);
	} else {
		*dst = s->r[offset];
	}

	return VMM_OK;
}

static void imx_set_txirq(struct imx_state *s, int level)
{
	u32 irq = s->txirq;

	if (!irq) {
		irq = s->rdirq;
	}
	vmm_devemu_emulate_irq(s->guest, irq, level);
}

static void zynq_set_irq(struct zynq_state *s, u32 level)
{
		vmm_devemu_emulate_irq(s->guest, s->irq, level);
}

static int zynq_reg_write(struct zynq_state *s, u32 offset,
			   u32 src_mask, u32 src)
{


	/////////////////////////////////////COMPLETAR A CENA DA SRC_MASK

	    offset >>= 2;
	    if (offset >= CADENCE_UART_R_MAX) {
	        return VMM_EFAIL;
	    }
	    switch (offset) {
	    case R_IER: /* ier (wts imr) */
	        s->r[R_IMR] |= src;
	        break;
	    case R_IDR: /* idr (wtc imr) */
	        s->r[R_IMR] &= ~src;
	        break;
	    case R_IMR: /* imr (read only) */
	        break;
	    case R_CISR: /* cisr (wtc) */
	        s->r[R_CISR] &= ~src;
	        break;
	    case R_TX_RX: /* UARTDR */
	        switch (s->r[R_MR] & UART_MR_CHMODE) {
	        case NORMAL_MODE:
	            uart_write_tx_fifo(s, (uint8_t *) &src, 1);
	            break;
	        case LOCAL_LOOPBACK:
	            uart_write_rx_fifo(s, (uint8_t *) &src, 1);
	            break;
	        }
	        break;
	    case R_BRGR: /* Baud rate generator */
	        if (src >= 0x01) {
	            s->r[offset] = src & 0xFFFF;
	        }
	        break;
	    case R_BDIV:    /* Baud rate divider */
	        if (src >= 0x04) {
	            s->r[offset] = src & 0xFF;
	        }
	        break;
	    default:
	        s->r[offset] = src;
	    }

	    switch (offset) {
	    case R_CR:
	        uart_ctrl_update(s);
	        break;
	    case R_MR:
	        uart_parameters_setup(s);
	        break;
	    }
	    uart_update_status(s);
	    return VMM_OK;
	}

static int imx_reg_write(struct imx_state *s, u32 offset,
			   u32 src_mask, u32 src)
{
	const unsigned int reg = _reg_offset_get(offset);
	u16 ack = 0;
	u16 usr1 = 0;
	u16 usr2 = 0;
	bool recv_char = FALSE;
	u16 val = (u16)(src & ~src_mask);

	vmm_spin_lock(&s->lock);

	usr1 = _reg_read(s, USR1);
	usr2 = _reg_read(s, USR2);

	if (URXD0 == offset) {
		/* Do nothing */
	} else if (URTX0 == offset) {
		s->tx = (u8)src;
		recv_char = TRUE;
	} else if (USR1 == offset) {
		_reg_ack(s, USR1, val & USR1_WR_MASK);
		ack = 1;
	} else if (USR2 == offset) {
		_reg_ack(s, USR2, val & USR2_WR_MASK);
		ack = 1;
	} else {
		if (reg < ARRAY_SIZE(s->regs)) {
			s->regs[reg] = val;

			if (UCR2 == offset) {
				_reg_clear_mask(s, UCR2, UCR2_SRST);
			}
		} else if (s->data->uts_reg == offset) {
			s->uts = (s->uts & src_mask) | (val & UTS_WR_MASK);
		} else {
			vmm_printf("i.MX UART unmanaged read at 0x%x\n",
				   offset);
		}
	}

	if (ack && ((usr1 != _reg_read(s, USR1)) ||
		    (usr2 != _reg_read(s, USR2)))) {
		imx_set_txirq(s, 0);
	}

	vmm_spin_unlock(&s->lock);

	if (!(_reg_read(s, UCR1) | UCR1_UARTEN) ||
	    !(_reg_read(s, UCR2) | UCR2_TXEN)) {
		return VMM_ENOTAVAIL;
	}

	if (recv_char) {
		u8 *val = (u8 *)&src;
		u32 len = 0;

		if (0xFFFFFF00 == src_mask) {
			len = 1;
		} else if (0xFFFF0000 == src_mask) {
			len = 2;
		} else {
			len = 4;
		}

		/*
		 * TODO: Create a thread for transfering characters which would
		 * disable correctly set USR1_TRDY, USR2_TXDC, USR2_TXDC, and
		 * UTS_TX[FULL|EMPTY].
		 */
		vmm_vserial_receive(s->vser, val, len);
	}

	/* Is TX ready interrupt enabled? */
	if ((_reg_read(s, USR1) & USR1_TRDY) &&
	    (_reg_read(s, UCR1) & UCR1_TRDYEN)) {
		/*
		 * As we do not manage the vserial overflow, we are always have
		 * the TX ready interrupt.
		 */
		imx_set_txirq(s, 1);
	} else if (_txfe_irq_enabled(s)) {
		imx_set_txirq(s, 1);
	} else if (_reg_read(s, UCR1) & UCR1_RTSDEN) {
		imx_set_txirq(s, 0);
	}

	return VMM_OK;
}

//DONE
static bool zynq_vserial_can_send(struct vmm_vserial *vser)
{
	struct zynq_state *s = vmm_vserial_priv(vser);
	return !fifo_isfull(s->rd_fifo);
}

/*
 *
 *
 *
 *
 * falta esta
 *
 *
 *
 *
 *
 *
 * */
static int zynq_vserial_send(struct vmm_vserial *vser, u8 data)
{

	return VMM_OK;
}

static int imx_vserial_send(struct vmm_vserial *vser, u8 data)
{
	bool set_irq = FALSE;
	u32 rd_count;
	struct imx_state *s = vmm_vserial_priv(vser);

	if (!(_reg_read(s, UCR1) & UCR1_UARTEN) ||
	    !(_reg_read(s, UCR2) & UCR2_RXEN)) {
		return VMM_ENOTAVAIL;
	}

	vmm_spin_lock(&s->lock);

	if (fifo_isfull(s->rd_fifo)) {
		vmm_spin_unlock(&s->lock);
		return VMM_ENOTAVAIL;
	}

	fifo_enqueue(s->rd_fifo, &data, TRUE);
	rd_count = fifo_avail(s->rd_fifo);

	s->uts &= ~UTS_RXEMPTY;
	if (IMX_FIFO_SIZE == rd_count) {
		s->uts |= UTS_RXFULL;
	}

	_reg_set_mask(s, USR2, USR2_RDR);
	if (rd_count >= (_reg_read(s, UFCR) & 0x003f)) {
		_reg_set_mask(s, USR1, USR1_RRDY);
		if (_reg_read(s, UCR1) & UCR1_RRDYEN) {
			set_irq = TRUE;
		}
	}

	vmm_spin_unlock(&s->lock);

	if (set_irq) {
		imx_set_rdirq(s, 1);
	}

	return VMM_OK;
}
/////////////////////////////////////////////////

//DONE
static int zynq_emulator_read8(struct vmm_emudev *edev,
				physical_addr_t offset,
				u8 *dst)
{
	int rc;
	u32 regval = 0x0;

	rc = zynq_reg_read(edev->priv, offset, &regval);
	if (!rc) {
		*dst = regval & 0xFF;
	}

	return rc;
}
//DONE
static int zynq_emulator_read16(struct vmm_emudev *edev,
				 physical_addr_t offset,
				 u16 *dst)
{
	int rc;
	u32 regval = 0x0;

	rc = zynq_reg_read(edev->priv, offset, &regval);
	if (!rc) {
		*dst = regval & 0xFFFF;
	}

	return rc;
}
//DONE
static int zynq_emulator_read32(struct vmm_emudev *edev,
				 physical_addr_t offset,
				 u32 *dst)
{
	return zynq_reg_read(edev->priv, offset, dst);
}

//DONE
static int zynq_emulator_write8(struct vmm_emudev *edev,
				 physical_addr_t offset,
				 u8 src)
{
	return zynq_reg_write(edev->priv, offset, 0xFFFFFF00, src);
}

//DONE
static int zynq_emulator_write16(struct vmm_emudev *edev,
				  physical_addr_t offset,
				  u16 src)
{
	return zynq_reg_write(edev->priv, offset, 0xFFFF0000, src);
}

//DONE
static int zynq_emulator_write32(struct vmm_emudev *edev,
				  physical_addr_t offset,
				  u32 src)
{
	return zynq_reg_write(edev->priv, offset, 0x00000000, src);
}

//DONE
static void uart_rx_reset(struct vmm_emudev *edev)
{
	struct zynq_state *s = edev->priv;

    s->rx_wpos = 0;
    s->rx_count = 0;
    s->r[R_SR] |= UART_SR_INTR_REMPTY;
    s->r[R_SR] &= ~UART_SR_INTR_RFUL;
}

//DONE
static void uart_tx_reset(struct vmm_emudev *edev)
{
	struct zynq_state *s = edev->priv;

    s->r[R_SR] |= UART_SR_INTR_TEMPTY;
    s->r[R_SR] &= ~UART_SR_INTR_TFUL;
   // s->tx_count = 0;
}



//DONE
//CHAnGED
static int zynq_emulator_reset(struct vmm_emudev *edev)
{
	struct zynq_state *s = edev->priv;

	vmm_spin_lock(&s->lock);

    s->r[R_CR] = 0x00000128;
    s->r[R_IMR] = 0;
    s->r[R_CISR] = 0;
    s->r[R_RTRIG] = 0x00000020;
    s->r[R_BRGR] = 0x0000000F;
    s->r[R_TTRIG] = 0x00000020;

	uart_rx_reset(edev);
	uart_tx_reset(edev);

    s->rx_count = 0;
    s->rx_wpos = 0;

	vmm_spin_unlock(&s->lock);

	return VMM_OK;
}

//DONE
static int zynq_emulator_probe(struct vmm_guest *guest,
			      struct vmm_emudev *edev,
			      const struct vmm_devtree_nodeid *eid)
{
	int rc = VMM_OK;
	char name[64];
	struct zynq_state *s;

	s = vmm_zalloc(sizeof(struct zynq_state));
	if (!s) {
		rc = VMM_EFAIL;
		goto zynq_emulator_probe_done;
	}

	s->guest = guest;
	INIT_SPIN_LOCK(&s->lock);
	//s->data = eid->data;

	rc = vmm_devtree_irq_get(edev->node, &s->irq, 0);
	if (rc) {
		goto zynq_emulator_probe_freestate_fail;
	}

	s->rd_fifo = fifo_alloc(1, ZYNQ_FIFO_SIZE);
	if (!s->rd_fifo) {
		rc = VMM_EFAIL;
		goto zynq_emulator_probe_freestate_fail;
	}

	strlcpy(name, guest->name, sizeof(name));
	strlcat(name, "/", sizeof(name));
	if (strlcat(name, edev->node->name, sizeof(name)) >= sizeof(name)) {
		rc = VMM_EOVERFLOW;
		goto zynq_emulator_probe_freerbuf_fail;
	}

	s->vser = vmm_vserial_create(name,
				     &zynq_vserial_can_send,
				     &zynq_vserial_send,
				     ZYNQ_FIFO_SIZE, s);
	if (!(s->vser)) {
		goto zynq_emulator_probe_freerbuf_fail;
	}

	edev->priv = s;
	zynq_emulator_reset(edev);

	goto zynq_emulator_probe_done;

zynq_emulator_probe_freerbuf_fail:
	fifo_free(s->rd_fifo);
zynq_emulator_probe_freestate_fail:
	vmm_free(s);
zynq_emulator_probe_done:
	return rc;
}

//DONE
static int zynq_emulator_remove(struct vmm_emudev *edev)
{
	struct zynq_state *s = edev->priv;

	if (s) {
		vmm_vserial_destroy(s->vser);
		fifo_free(s->rd_fifo);
		vmm_free(s);
		edev->priv = NULL;
	}

	return VMM_OK;
}

//DONE
static struct vmm_devtree_nodeid zynq_emuid_table[] = {
	{
		.type = "serial",
		.compatible = "xlnx,xuartps",
	},
	{ /* end of list */ },
};

//DONE
static struct vmm_emulator zynq_emulator = {
	.name = "zynq_serial",
	.match_table = zynq_emuid_table,//Isto apenas tem de dar match com o emulador
	//em vmm_devemu_probe_region...não meter nada no .data
	.endian = VMM_DEVEMU_LITTLE_ENDIAN,
	.probe = zynq_emulator_probe,
	.read8 = zynq_emulator_read8,
	.write8 = zynq_emulator_write8,
	.read16 = zynq_emulator_read16,
	.write16 = zynq_emulator_write16,
	.read32 = zynq_emulator_read32,
	.write32 = zynq_emulator_write32,
	.reset = zynq_emulator_reset,
	.remove = zynq_emulator_remove,
};

//DONE
static int __init zynq_emulator_init(void)
{
	return vmm_devemu_register_emulator(&zynq_emulator);
}

//DONE
static void __exit zynq_emulator_exit(void)
{
	vmm_devemu_unregister_emulator(&zynq_emulator);
}

VMM_DECLARE_MODULE(MODULE_DESC,
			MODULE_AUTHOR,
			MODULE_LICENSE,
			MODULE_IPRIORITY,
			MODULE_INIT,
			MODULE_EXIT);
