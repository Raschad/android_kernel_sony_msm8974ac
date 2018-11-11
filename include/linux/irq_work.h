#ifndef _LINUX_IRQ_WORK_H
#define _LINUX_IRQ_WORK_H

#include <linux/llist.h>

#define IRQ_WORK_PENDING	1UL
#define IRQ_WORK_BUSY		2UL
#define IRQ_WORK_FLAGS		3UL
#define IRQ_WORK_LAZY		4UL /* Doesn't want IPI, wait for tick */

struct irq_work {
	unsigned long flags;
	struct llist_node llnode;
	void (*func)(struct irq_work *);
};

static inline
void init_irq_work(struct irq_work *work, void (*func)(struct irq_work *))
{
	work->flags = 0;
	work->func = func;
}
#define DEFINE_IRQ_WORK(name, _f) struct irq_work name = { .func = (_f), }
bool irq_work_queue(struct irq_work *work);
void irq_work_run(void);
void irq_work_sync(struct irq_work *work);

#endif /* _LINUX_IRQ_WORK_H */
