/**
 * Copyright (C) 2014 Institut de Recherche Technologique SystemX and OpenWide.
 * Jimmy Durand Wesolowski (jimmy.durand-wesolowski@openwide.fr)
 * All rights reserved.
 * Inspired from pl01x.c, written by Anup Patel.
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
 * @file imx.c
 * @author Jimmy Durand Wesolowski (jimmy.durand-wesolowski@openwide.fr)
 * @brief source file for i.MX serial port driver.
 */

#include <arm_io.h>
#include <arm_math.h>
#include <serial/zynq.h>

void zynq_putc(u32 base, char ch)
{
	/* Wait until there is space in the FIFO */
	while (!(arm_readl((void*)(base + 0x2c)) & ZYNQ_UART_SR_TXEMPTY)) {
			;
		}

	/* Send the character */
	arm_writel(ch, (void*)(base + 0x30));
}

char zynq_getc(u32 base)
{
	u16 data;

	/* Wait until there is data in the FIFO */
	while ((arm_readl((void*)(base + 0x2c)) & ZYNQ_UART_SR_RXEMPTY)) {
		;
	}

	data = (char)arm_readl((void*)(base + 0x30));


	return data;
}

static void zynq_uart_setbrg(u32 base)
{

    /* Calculation results. */
    unsigned int calc_bauderror, bdiv, bgen;
    unsigned long calc_baud = 0;
    u32 input_clock = UART_FREQ_CLK;
    u32 baudrate = UART_BAUD_RATE;

    /*                master clock
     * Baud rate = ------------------
     *              bgen * (bdiv + 1)
     *
     * Find acceptable values for baud generation.
     */
    for (bdiv = 4; bdiv < 255; bdiv++) {
            bgen = arm_udiv32(input_clock, (baudrate * (bdiv + 1)));
            if (bgen < 2 || bgen > 65535)
                    continue;

            calc_baud = arm_udiv32(input_clock, (bgen * (bdiv + 1)));

            /*
             * Use first calculated baudrate with
             * an acceptable (<3%) error
             */
            if (baudrate > calc_baud)
                    calc_bauderror = baudrate - calc_baud;
            else
                    calc_bauderror = calc_baud - baudrate;
            if (arm_udiv32((calc_bauderror * 100), baudrate) < 3)
                    break;
    }

    arm_writel(bdiv, (void*)(base + 0x34));
    arm_writel(bgen, (void*)(base + 0x18));

}

void zynq_init(u32 base, u32 baudrate, u32 input_clock)
{


	arm_writel(ZYNQ_UART_CR_TX_EN | ZYNQ_UART_CR_RX_EN | ZYNQ_UART_CR_TXRST | \
	                                       ZYNQ_UART_CR_RXRST, (void*)(base + 0x0));
	arm_writel(ZYNQ_UART_MR_PARITY_NONE, (void*)(base + 0x4)); /* 8 bit, no parity */

	/* Set baud rate here */
	zynq_uart_setbrg(base);

}
