/**
 * Copyright (C) 2014 Institut de Recherche Technologique SystemX and OpenWide.
 * Inspired by sp804.c, Copyright (c) 2013 Anup Patel.
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
 * @file imx_gpt.c
 * @author Jimmy Durand Wesolowski (jimmy.durand-wesolowski@openwide.fr)
 * @brief i.MX6 GPT timer
 */

#include <arm_io.h>
#include <arm_irq.h>
#include <arm_math.h>

#include <timer/arm_private_timer.h>

#define TWD_TIMER_LOAD			0x00
#define TWD_TIMER_COUNTER		0x04
#define TWD_TIMER_CONTROL		0x08
#define TWD_TIMER_INTSTAT		0x0C

#define TWD_TIMER_CONTROL_ENABLE	(1 << 0)
#define TWD_TIMER_CONTROL_ONESHOT	(0 << 1)
#define TWD_TIMER_CONTROL_PERIODIC	(1 << 1)
#define TWD_TIMER_CONTROL_IT_ENABLE	(1 << 2)
#define TWD_TIMER_PRESCALE_SHIFT  		8
#define TWD_TIMER_INTERRUPT_CLEAR 		(1<<0)


#define CLK_FREQ	666666000
#define PRIV_TIMER_FREQ		(CLK_FREQ / 2)

static u32 arm_priv_timer_base;
static u32 arm_priv_timer_irq;
static u32 arm_priv_timer_prescale = 0;
static u64 arm_priv_timer_irq_count = 0;
static u64 arm_priv_timer_freerun = 0;
static u64 arm_priv_timer_timestamp_sum = 0;
static u64 arm_priv_timer_tstamp = 0;
static u64 arm_priv_timer_irq_delay = 0;
static u64 arm_priv_timer_period = 0;


void arm_priv_timer_enable(void)
{
	u32 ctrl;

	ctrl = arm_readl((void *)(arm_priv_timer_base + TWD_TIMER_CONTROL));
	ctrl |= TWD_TIMER_CONTROL_ENABLE;
	arm_writel(ctrl, (void *)(arm_priv_timer_base + TWD_TIMER_CONTROL));
}

void arm_priv_timer_disable(void)
{
	u32 ctrl;

	ctrl = arm_readl((void *)(arm_priv_timer_base + TWD_TIMER_CONTROL));
	ctrl &= ~TWD_TIMER_CONTROL_ENABLE;
	arm_writel(ctrl, (void *)(arm_priv_timer_base + TWD_TIMER_CONTROL));
}

static void arm_priv_timer_next_event(void)
{
	arm_writel(arm_priv_timer_period, (void *)(arm_priv_timer_base + TWD_TIMER_LOAD));
}

void arm_priv_timer_change_period(u32 usec)
{
	// Cortex-A9 MPCore - Technical Reference Manual (section 4.1.1)
		/*              (PRESCALER_value+1) x (Load_value+1) x 2
		 * interval = ---------------------------------------------
		 *                         CPU_CLK_frequency = 666 666 000
		 */
	arm_priv_timer_period = ((usec * 333) - 1);


	if (!arm_priv_timer_freerun) {
		arm_priv_timer_timestamp_sum += arm_readl((void *)(arm_priv_timer_base +
				TWD_TIMER_COUNTER));
	}

	arm_priv_timer_next_event();
}

u64 arm_priv_timer_irqcount(void)
{
	return arm_priv_timer_irq_count;
}

u64 arm_priv_timer_irqdelay(void)
{
	return arm_priv_timer_irq_delay;
}

u64 arm_priv_timer_timestamp(void)
{
	u32 counter = 0;
	u64 timestamp;

	counter = arm_readl((void *)(arm_priv_timer_base + TWD_TIMER_LOAD));
	counter -= arm_readl((void *)(arm_priv_timer_base + TWD_TIMER_COUNTER));
	if (arm_priv_timer_freerun) {
		/*
		 * In freerun mode, we only have to make a 64 bit counter
		 * from a 32 bit one.
		 */
	} else {
		/*
		 * In restart mode, we retrieve the period sum and the
		 * current value
		 */
		timestamp = arm_priv_timer_timestamp_sum + counter;
	}
	return timestamp * PRIV_TIMER_FREQ;
}

int arm_priv_timer_irqhndl(u32 irq_no, struct pt_regs * regs)
{
	u64 tstamp = 0;

	arm_priv_timer_disable();

	tstamp = arm_priv_timer_timestamp();
	arm_priv_timer_irq_delay = tstamp - arm_priv_timer_tstamp;
	arm_priv_timer_tstamp = tstamp;
	arm_priv_timer_irq_count++;

	if (!arm_priv_timer_freerun) {
		arm_priv_timer_timestamp_sum += arm_priv_timer_period;
	}

	arm_priv_timer_next_event();
	arm_writel(TWD_TIMER_INTERRUPT_CLEAR, (void *)(arm_priv_timer_base +
			TWD_TIMER_INTSTAT)); //Clear interrupt flag
	arm_priv_timer_enable();

	return 0;
}

int arm_priv_timer_init(u32 usecs, u32 base, u32 irq, u32 freerun)
{
	u32 val;

	arm_priv_timer_base = base;
	arm_priv_timer_irq = irq;
	arm_priv_timer_irq_count = 0;
	arm_priv_timer_prescale = 0;

	/* Register interrupt handler */
	arm_irq_register(arm_priv_timer_irq, &arm_priv_timer_irqhndl);

	if (freerun) {
		arm_priv_timer_freerun = 1;
		//Configuraçao one shot
	}
	val = (arm_priv_timer_prescale << TWD_TIMER_PRESCALE_SHIFT) |
			TWD_TIMER_CONTROL_IT_ENABLE | TWD_TIMER_CONTROL_PERIODIC;
	arm_writel(val, (void *)(arm_priv_timer_base + TWD_TIMER_CONTROL));

	arm_priv_timer_disable();
	arm_priv_timer_change_period(usecs);
	arm_priv_timer_enable();

	return 0;
}
