/*
 * binder_prio: raise the binder priority of a few UI-critical processes.
 *
 * sapphire (5.15 GKI) build. Template: ChicKernel
 * device_xiaomi_gemstones-kernel (android13-5.15-lts)
 * drivers/staging/binder_prio/binder_prio.c. The stock sapphire
 * binder_prio.ko is a different (Xiaomi) revision of this driver; the
 * hook bodies below were rewritten to match it:
 *  - set_priority: a synchronous transaction coming from the main thread
 *    of an RT com.miui.home / ndroid.systemui / surfaceflinger process
 *    makes the target thread SCHED_FIFO (kernel prio 98), otherwise the
 *    target node's own min priority/policy is applied when it is RT;
 *  - trans: a synchronous transaction to surfaceflinger sets its
 *    default_priority to SCHED_FIFO / 98;
 *  - no registration error handling or log strings besides init/exit.
 */
#include <linux/swap.h>
#include <linux/module.h>
#include <trace/hooks/binder.h>
#include <uapi/linux/android/binder.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/prio.h>
#include "binder_internal.h"
#include <linux/string.h>

#define BINDER_PRIO_RT_PRIO	98

/* same as rt_policy() in kernel/sched/sched.h */
static inline int binder_prio_rt_policy(int policy)
{
	return policy == SCHED_FIFO || policy == SCHED_RR;
}

static void extend_surfacefinger_binder_set_priority_handler(
	void *data, struct binder_transaction *t, struct task_struct *task)
{
	struct sched_param params;
	struct binder_node *node = t->buffer->target_node;
	unsigned int prio = node->min_priority;
	unsigned int policy = node->sched_policy;
	struct task_struct *from;

	if (t && t->from && (from = t->from->task) &&
	    !(t->flags & TF_ONE_WAY) &&
	    binder_prio_rt_policy(from->policy) &&
	    from->pid == from->tgid &&
	    (!strncmp(from->comm, "com.miui.home", strlen("com.miui.home")) ||
	     !strncmp(from->comm, "ndroid.systemui", strlen("ndroid.systemui")) ||
	     !strncmp(from->comm, "surfaceflinger", strlen("surfaceflinger")))) {
		policy = SCHED_FIFO;
		prio = BINDER_PRIO_RT_PRIO;
	} else if (!binder_prio_rt_policy(policy)) {
		return;
	}

	if (task->policy != policy) {
		params.sched_priority = MAX_RT_PRIO - 1 - prio;
		sched_setscheduler_nocheck(task, policy | SCHED_RESET_ON_FORK,
					   &params);
	}
}

static void extend_surfacefinger_binder_trans_handler(
	void *data, struct binder_proc *target_proc, struct binder_proc *proc,
	struct binder_thread *thread, struct binder_transaction_data *tr)
{
	// Determine the current binder status
	// 1. Whether the peer is surfaceflinger
	// 2. Whether it is non-oneway
	if (target_proc && target_proc->tsk &&
	    strncmp(target_proc->tsk->comm, "surfaceflinger", strlen("surfaceflinger")) == 0) {
		if (proc && thread && tr && thread->transaction_stack &&
		    (!(thread->transaction_stack->flags & TF_ONE_WAY))) {
			target_proc->default_priority.sched_policy = SCHED_FIFO;
			target_proc->default_priority.prio = BINDER_PRIO_RT_PRIO;
		}
	}
}

int __init binder_prio_init(void)
{
	pr_info("binder_prio: module init!");
	// extend_surfacefinger_binder_set_priority_handler 函数对应
	// trace_android_vh_binder_set_priority hook 点
	register_trace_android_vh_binder_set_priority(
		extend_surfacefinger_binder_set_priority_handler, NULL);
	// extend_surfacefinger_binder_trans_handler 函数对应
	// trace_android_vh_binder_trans hook 点
	register_trace_android_vh_binder_trans(
		extend_surfacefinger_binder_trans_handler, NULL);
	return 0;
}

void __exit binder_prio_exit(void)
{
	unregister_trace_android_vh_binder_set_priority(
		extend_surfacefinger_binder_set_priority_handler, NULL);
	unregister_trace_android_vh_binder_trans(
		extend_surfacefinger_binder_trans_handler, NULL);

	pr_info("binder_prio: module exit!");
}

module_init(binder_prio_init);
module_exit(binder_prio_exit);
MODULE_LICENSE("GPL");
/* sapphire: the stock module was built in-tree from drivers/staging, keep its modinfo */
MODULE_INFO(intree, "Y");
MODULE_INFO(staging, "Y");
