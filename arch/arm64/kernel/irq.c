/*
 * Based on arch/arm/kernel/irq.c
 *
 * Copyright (C) 1992 Linus Torvalds
 * Modifications for ARM processor Copyright (C) 1995-2000 Russell King.
 * Support for Dynamic Tick Timer Copyright (C) 2004-2005 Nokia Corporation.
 * Dynamic Tick Timer written by Tony Lindgren <tony@atomide.com> and
 * Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>.
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel_stat.h>
#include <linux/irq.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/seq_file.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
<<<<<<< HEAD
=======
#include <soc/qcom/htc_util.h>
#endif
>>>>>>> 0e91d2a... Nougat

unsigned long irq_err_count;

int arch_show_interrupts(struct seq_file *p, int prec)
{
#ifdef CONFIG_SMP
	show_ipi_list(p, prec);
#endif
	seq_printf(p, "%*s: %10lu\n", prec, "Err", irq_err_count);
	return 0;
}

#ifdef CONFIG_HTC_POWER_DEBUG
unsigned int *previous_irqs;
static int pre_nr_irqs = 0;
static char irq_output[1024];
static void htc_show_interrupt(int i)
{
        struct irqaction *action;
        unsigned long flags;
        struct irq_desc *desc;
        char irq_piece[32];
        char name_resize[16];

        if (i < nr_irqs) {
                desc = irq_to_desc(i);
		if (!desc)
			return;
                raw_spin_lock_irqsave(&desc->lock, flags);
                action = desc->action;
                if (!action)
                        goto unlock;
                if (!(kstat_irqs_cpu(i, 0)) || previous_irqs[i] == (kstat_irqs_cpu(i, 0)))
                        goto unlock;
                memset(irq_piece, 0, sizeof(irq_piece));
                memset(name_resize, 0, sizeof(name_resize));
                strncat(name_resize, action->name, 8);
                snprintf(irq_piece, sizeof(irq_piece), "%s(%d:%s,%u)",
                    strlen(irq_output)>0 ? "," : "",
                    i,
                    name_resize,
                    kstat_irqs_cpu(i, 0)-previous_irqs[i]);
                safe_strcat(irq_output, irq_piece);
                previous_irqs[i] = kstat_irqs_cpu(i, 0);
unlock:
                raw_spin_unlock_irqrestore(&desc->lock, flags);
        } else if (i == nr_irqs) {
		if (previous_irqs[nr_irqs] == irq_err_count)
                        return;
                printk("Err: %lud\n", irq_err_count-previous_irqs[nr_irqs]);
                previous_irqs[nr_irqs] = irq_err_count;
        }
}

void htc_show_interrupts(void)
{
        int i = 0;
               if(pre_nr_irqs != nr_irqs) {
                       pre_nr_irqs = nr_irqs;
                       previous_irqs = (unsigned int *)kcalloc(nr_irqs, sizeof(int),GFP_KERNEL);
               }
               memset(irq_output, 0, sizeof(irq_output));
               for (i = 0; i <= nr_irqs; i++) {
                       htc_show_interrupt(i);
               }
               k_pr_embedded("[K] irq: %s\n", irq_output);
}
#endif

/*
 * handle_IRQ handles all hardware IRQ's.  Decoded IRQs should
 * not come via this function.  Instead, they should provide their
 * own 'handler'.  Used by platform code implementing C-based 1st
 * level decoding.
 */
void handle_IRQ(unsigned int irq, struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

	irq_enter();

	/*
	 * Some hardware gives randomly wrong interrupts.  Rather
	 * than crashing, do something sensible.
	 */
	if (unlikely(irq >= nr_irqs)) {
		pr_warn_ratelimited("Bad IRQ%u\n", irq);
		ack_bad_irq(irq);
	} else {
		generic_handle_irq(irq);
	}

	irq_exit();
	set_irq_regs(old_regs);
}

void __init set_handle_irq(void (*handle_irq)(struct pt_regs *))
{
	if (handle_arch_irq)
		return;

	handle_arch_irq = handle_irq;
}

void __init init_IRQ(void)
{
	irqchip_init();
	if (!handle_arch_irq)
		panic("No interrupt controller found.");
}

#ifdef CONFIG_HOTPLUG_CPU
static bool migrate_one_irq(struct irq_desc *desc)
{
	struct irq_data *d = irq_desc_get_irq_data(desc);
	const struct cpumask *affinity = d->affinity;

	if (irqd_is_per_cpu(d) || !cpumask_test_cpu(smp_processor_id(), affinity))
		return false;

	if (cpumask_any_and(affinity, cpu_online_mask) >= nr_cpu_ids)
		affinity = cpu_online_mask;
	return irq_set_affinity_locked(d, affinity, 0) == 0;
}

void migrate_irqs(void)
{
	unsigned int i;
	struct irq_desc *desc;
	unsigned long flags;

	local_irq_save(flags);

	for_each_irq_desc(i, desc) {
		bool affinity_broken;

		raw_spin_lock(&desc->lock);
		affinity_broken = migrate_one_irq(desc);
		raw_spin_unlock(&desc->lock);

		if (affinity_broken)
			pr_warn_ratelimited("IRQ%u no longer affine to CPU%u\n",
					    i, smp_processor_id());
	}

	local_irq_restore(flags);
}
#endif 
