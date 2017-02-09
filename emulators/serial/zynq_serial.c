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
#define MODULE_AUTHOR			"João Pedro Barros Silva "
#define MODULE_LICENSE			"GPL"
#define MODULE_IPRIORITY		(VMM_VSERIAL_IPRIORITY+1)
#define	MODULE_INIT			zynq_emulator_init
#define	MODULE_EXIT			zynq_emulator_exit

//19.3.3 - Zynq-7000 TRM
#define RX_FIFO_SIZE           64// 64 bytes (the size of the TxFIFO)

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

struct zynq_state {
	struct vmm_guest *guest;
	struct vmm_vserial *vser;
	vmm_spinlock_t lock;
	u32 irq;
    u32 r[CADENCE_UART_R_MAX];
    u8 r_fifo[RX_FIFO_SIZE];//RETIRAR ISTO
    u32 rx_wpos;
    u32 rx_count;
    u64 char_tx_time;
	struct fifo *rd_fifo;
};


static void zynq_set_irq(struct zynq_state *s, u32 level)
{
		vmm_devemu_emulate_irq(s->guest, s->irq, level);
}


//DONE
//CHANGED
static void uart_rx_reset(struct zynq_state *s)
{
    s->rx_wpos = 0;
    s->rx_count = 0;
    s->r[R_SR] |= UART_SR_INTR_REMPTY;
    s->r[R_SR] &= ~UART_SR_INTR_RFUL;
}

//DONE
//CHANGED
static void uart_tx_reset(struct zynq_state *s)
{
    s->r[R_SR] |= UART_SR_INTR_TEMPTY;
    s->r[R_SR] &= ~UART_SR_INTR_TFUL;
   // s->tx_count = 0;
}

//DONE
//CHANGED
static void uart_update_status(struct zynq_state *s)
{
    s->r[R_CISR] |= s->r[R_SR] & UART_SR_TO_CISR_MASK;
    zynq_set_irq(s, !!(s->r[R_IMR] & s->r[R_CISR]));//TENHO DE SABER SE
    //É ESTE O VALOR OU O CONTRARIO, EM PRINCIPIO SERA ASSIM
}

static void uart_tx_redo(struct zynq_state *s)
{
    s->r[R_SR] |= UART_SR_INTR_TEMPTY;

    uart_update_status(s);
}

//DONE
//ACHO QUE ESTÁ COMPLETO...
static int zynq_reg_read(struct zynq_state *s, u32 offset, u32 *dst)
{
	u32 read_count = 0x0;
	u8 val = 0x0;

	offset >>= 2;
	if (offset >= CADENCE_UART_R_MAX) {
		*dst = 0;
	} else if (offset == R_TX_RX) {

	    if ((s->r[R_CR] & UART_CR_RX_DIS) || !(s->r[R_CR] & UART_CR_RX_EN)) {
	        goto label;
	    }

	    s->r[R_SR] &= ~UART_SR_INTR_RFUL;

	    read_count = fifo_avail(s->rd_fifo);
	    if (read_count) {
	        fifo_dequeue(s->rd_fifo, &val);//remove from the queue
	        read_count = fifo_avail(s->rd_fifo);
			*dst = val;

	        if (!read_count) {
	            s->r[R_SR] |= UART_SR_INTR_REMPTY;
	        }
	    } else {
	        *dst = 0;
	        s->r[R_SR] |= UART_SR_INTR_REMPTY;
	    }

	    if (read_count < s->r[R_RTRIG]) {
	        s->r[R_SR] &= ~UART_SR_INTR_RTRIG;
	    }
	    s->r[R_CISR] |= s->r[R_SR] & UART_SR_TO_CISR_MASK;
	    zynq_set_irq(s, !!(s->r[R_IMR] & s->r[R_CISR]));

	} else {
		*dst = s->r[offset];
	}

label:
	return VMM_OK;
}

static void uart_ctrl_update(struct zynq_state *s)
{
    if (s->r[R_CR] & UART_CR_TXRST) {
        uart_tx_reset(s);
    }

    if (s->r[R_CR] & UART_CR_RXRST) {
        uart_rx_reset(s);
    }

    s->r[R_CR] &= ~(UART_CR_TXRST | UART_CR_RXRST);

    if ((s->r[R_CR] & UART_CR_TX_EN) && !(s->r[R_CR] & UART_CR_TX_DIS)) {
            uart_tx_redo(s);
    }
}

static int zynq_reg_write(struct zynq_state *s, u32 offset,
			   u32 src_mask, u32 src)
{
	u8 val;
	u32 rd_count;

	    offset >>= 2;
	    if (offset >= CADENCE_UART_R_MAX) {
	        return VMM_EFAIL;
	    }
	    switch (offset) {
	    case R_IER: /* ier (wts imr) */
	        s->r[R_IMR] = (s->r[R_IMR] & src_mask) | (src & ~src_mask);
	        break;
	    case R_IDR: /* idr (wtc imr) */
	        s->r[R_IMR] = (s->r[R_IMR] & src_mask) & ~(src & ~src_mask);
	        break;
	    case R_IMR: /* imr (read only) */
	        break;
	    case R_CISR: /* cisr (wtc) */
	        s->r[R_CISR] = (s->r[R_CISR] & src_mask) & ~(src & ~src_mask);
	        break;
	    case R_TX_RX: /* UARTDR */
	        switch (s->r[R_MR] & UART_MR_CHMODE) {
	        case NORMAL_MODE:
	        	if ((s->r[R_CR] & UART_CR_TX_DIS) || !(s->r[R_CR] & UART_CR_TX_EN))
	        	{
	        	}
	        	else
	        	{
	        		val = src;
	        	   	vmm_vserial_receive(s->vser, &val, 1);
	        	}
	        	break;
	        case LOCAL_LOOPBACK:
	        	if ((s->r[R_CR] & UART_CR_RX_DIS) || \
	    	    		!(s->r[R_CR] & UART_CR_RX_EN)) {
	        		goto label;
	    	    }

	    	    s->r[R_SR] &= ~UART_SR_INTR_REMPTY;
	    		rd_count = fifo_avail(s->rd_fifo);

	    	    if (rd_count == RX_FIFO_SIZE) {
	    	        s->r[R_CISR] |= UART_INTR_ROVR;
	    	    } else {
	    	    		fifo_enqueue(s->rd_fifo, &src, TRUE);
	    	    		rd_count = fifo_avail(s->rd_fifo);
	    	            if (rd_count == RX_FIFO_SIZE) {
	    	                s->r[R_SR] |= UART_SR_INTR_RFUL;
	    	                //MAYBE GOTO LABEL HERE
	    	            }

	    	            if (rd_count >= s->r[R_RTRIG]) {
	    	                s->r[R_SR] |= UART_SR_INTR_RTRIG;
	    	            }
	    	        }
				uart_update_status(s);
	            break;
	        }
	        break;
	    default:
	        s->r[offset] = src;
	    }
label:
	    switch (offset) {
	    case R_CR:
	        uart_ctrl_update(s);//done
	        break;
	    case R_MR:
	        //uart_parameters_setup(s);
	        break;
	    }

	    s->r[R_CISR] |= s->r[R_SR] & UART_SR_TO_CISR_MASK;
	    zynq_set_irq(s, !!(s->r[R_IMR] & s->r[R_CISR]));

	    return VMM_OK;
	}

//DONE
//NO CHANGES
static bool zynq_vserial_can_send(struct vmm_vserial *vser)
{
	struct zynq_state *s = vmm_vserial_priv(vser);
	return !fifo_isfull(s->rd_fifo);
}

//DONE
static int zynq_vserial_send(struct vmm_vserial *vser, u8 data)
{
	struct zynq_state *s = vmm_vserial_priv(vser);
	bool recv_char = FALSE;
	u32 rd_count;
	u8 val;

	u32 ch_mode = s->r[R_MR] & UART_MR_CHMODE;

	if (ch_mode == NORMAL_MODE || ch_mode == ECHO_MODE) {
	    if ((s->r[R_CR] & UART_CR_RX_DIS) || !(s->r[R_CR] & UART_CR_RX_EN)) {
	        goto label;
	    }

	    s->r[R_SR] &= ~UART_SR_INTR_REMPTY;
		rd_count = fifo_avail(s->rd_fifo);

	    if (rd_count == RX_FIFO_SIZE) {
	        s->r[R_CISR] |= UART_INTR_ROVR;
	    } else {
	    		fifo_enqueue(s->rd_fifo, &data, TRUE);
	    		rd_count = fifo_avail(s->rd_fifo);
	            if (rd_count == RX_FIFO_SIZE) {
	                s->r[R_SR] |= UART_SR_INTR_RFUL;
	                goto label1;
	                //MAYBE GOTO LABEL HERE
	            }

	            if (rd_count >= s->r[R_RTRIG]) {
	                s->r[R_SR] |= UART_SR_INTR_RTRIG;
	            }
	        }
label1:
	    uart_update_status(s);
	}
	if (ch_mode == REMOTE_LOOPBACK || ch_mode == ECHO_MODE) {
	    if ((s->r[R_CR] & UART_CR_TX_DIS) || !(s->r[R_CR] & UART_CR_TX_EN)) {
	        goto label;
	    }
		recv_char = TRUE;
	}
	if (recv_char) {//CONFIRMAR SE ISTO VAI DAR CERTO, NOS
		//OUTROS EMULADORES NAO TEM ESTA PARTE
		val = data;
		vmm_vserial_receive(s->vser, &val, 1);
	}
	/*if(set_irq)//ACHo QUE SE PODE POR LÀ EM CIMA ISTO
	{
		uart_update_status(s);
	}*/

label:
	return VMM_OK;
}

//DONE
//NO CHANGES
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
//NO CHANGES
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
//NO CHANGES
static int zynq_emulator_read32(struct vmm_emudev *edev,
				 physical_addr_t offset,
				 u32 *dst)
{
	return zynq_reg_read(edev->priv, offset, dst);
}

//DONE
//NO CHANGES
static int zynq_emulator_write8(struct vmm_emudev *edev,
				 physical_addr_t offset,
				 u8 src)
{
	return zynq_reg_write(edev->priv, offset, 0xFFFFFF00, src);
}

//DONE
//NO CHANGES
static int zynq_emulator_write16(struct vmm_emudev *edev,
				  physical_addr_t offset,
				  u16 src)
{
	return zynq_reg_write(edev->priv, offset, 0xFFFF0000, src);
}

//DONE
//NO CHANGES
static int zynq_emulator_write32(struct vmm_emudev *edev,
				  physical_addr_t offset,
				  u32 src)
{
	return zynq_reg_write(edev->priv, offset, 0x00000000, src);
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

	uart_rx_reset(s);
	uart_tx_reset(s);

	vmm_spin_unlock(&s->lock);

	return VMM_OK;
}

//DONE
//NO CHANGES
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
//NO CHANGES
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
//NO CHANGES
static struct vmm_devtree_nodeid zynq_emuid_table[] = {
	{
		.type = "serial",
		.compatible = "xlnx,xuartps",
	},
	{ /* end of list */ },
};

//DONE
//NO CHANGES
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
//NO CHANGES
static int __init zynq_emulator_init(void)
{
	return vmm_devemu_register_emulator(&zynq_emulator);
}

//DONE
//NO CHANGES
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
