/*
 *  drivers/cpufreq/cpufreq_syncedplus.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
			  (C)  2018 Muhamed Trumic <muhamedtrumic97@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/lcd_notify.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define MICRO_FREQUENCY_MIN_SAMPLE_RATE	(10000)

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */

#define MIN_SAMPLING_RATE_RATIO			(2)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

#define UP_THRESHOLD					(85)
#define SAMPLING_RATE_SCREEN_OFF		(80000)
#define MAX_SCREEN_OFF_FREQ				(1344000)

static unsigned int min_sampling_rate;
static unsigned int current_sampling_rate;
static unsigned int screen_off;
static unsigned int last_cpu_load;

static struct notifier_block syncedplus_lcd_notif;

static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct cpufreq_frequency_table *freq_table;	
	struct delayed_work work;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct *dbs_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int ignore_nice;	

} dbs_tuners_ins = {
	.sampling_rate = 20000,
	.ignore_nice = 0,
};

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}

/* keep track of frequency transitions */
static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	 */
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};

/************************** sysfs interface ************************/

/* cpufreq_syncedplus Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}

show_one(sampling_rate, sampling_rate);
show_one(ignore_nice_load, ignore_nice);

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input < min_sampling_rate)
		input = min_sampling_rate;

	dbs_tuners_ins.sampling_rate = input;
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) /* nothing to do */
		return count;

	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(ignore_nice_load);

static struct attribute *dbs_attributes[] = {
	&sampling_rate.attr,
	&ignore_nice_load.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "syncedplus",
};

/************************** sysfs end ************************/

static int syncedplus_lcd_notifier_callback(struct notifier_block *this,
								unsigned long event, void *data)
{
	switch (event)
	{
		case LCD_EVENT_OFF_END:
			screen_off = 1;
			break;

		case LCD_EVENT_ON_START:
			screen_off = 0;
			break;

		default:
			break;
	}
	return 0;
}

static void screen_off_freq_change(struct cpu_dbs_info_s *this_dbs_info, 
							struct cpufreq_policy *policy, unsigned int max_load)
{
	int freq_dir = 0;

	current_sampling_rate = SAMPLING_RATE_SCREEN_OFF;

	freq_dir = max_load - last_cpu_load;

	if (freq_dir > 0) {
		this_dbs_info->requested_freq += (freq_dir * policy->max) / 100;

		if (this_dbs_info->requested_freq > MAX_SCREEN_OFF_FREQ)
			this_dbs_info->requested_freq = MAX_SCREEN_OFF_FREQ;

		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, 
			CPUFREQ_RELATION_H);

		last_cpu_load = max_load;
		return;
	}

	if (freq_dir < 0) {
		this_dbs_info->requested_freq -= (abs(freq_dir) * policy->max) / 100;

		if (this_dbs_info->requested_freq < policy->min)
			this_dbs_info->requested_freq = policy->min;

		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, 
			CPUFREQ_RELATION_H);

		last_cpu_load = max_load;
		return;
	}

	if (freq_dir == 0) {
		this_dbs_info->requested_freq = MAX_SCREEN_OFF_FREQ * max_load / 100 + policy->min;

		if (this_dbs_info->requested_freq > MAX_SCREEN_OFF_FREQ)
			this_dbs_info->requested_freq = MAX_SCREEN_OFF_FREQ;

		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, 
			CPUFREQ_RELATION_H);

		last_cpu_load = max_load;
		return;
	}
}

static void screen_on_freq_change(struct cpu_dbs_info_s *this_dbs_info,
						   struct cpufreq_policy *policy, unsigned int max_load)
{
	int freq_dir = 0;

	current_sampling_rate = dbs_tuners_ins.sampling_rate;

	freq_dir = max_load - last_cpu_load;

	if (freq_dir > 0) {
		this_dbs_info->requested_freq += (freq_dir * policy->max) / 100;

		if (this_dbs_info->requested_freq > policy->max)
			this_dbs_info->requested_freq = policy->max;	
	
		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, 
			CPUFREQ_RELATION_L);

		last_cpu_load = max_load;
		return;
	}

	if (freq_dir < 0) {
		this_dbs_info->requested_freq -= (abs(freq_dir) * policy->max) / 100;

		if (this_dbs_info->requested_freq < policy->min)
			this_dbs_info->requested_freq = policy->min;
	
		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, 
			CPUFREQ_RELATION_L);

		last_cpu_load = max_load;
		return;
	}

	if (freq_dir == 0) {

		if (max_load > UP_THRESHOLD)
			this_dbs_info->requested_freq = policy->max;
		else
			this_dbs_info->requested_freq = (policy->max * max_load / 100) + (2 * policy->min);

		if (this_dbs_info->requested_freq > policy->max)
			this_dbs_info->requested_freq = policy->max;

		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, 
			CPUFREQ_RELATION_L);

		last_cpu_load = max_load;
		return;
	}
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load = 0;
	unsigned int max_load = 0;
	
	struct cpufreq_policy *policy;
	unsigned int j;

	policy = this_dbs_info->cur_policy;

	/* Get Absolute Load */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		if (dbs_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice;
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		if (load > max_load)
			max_load = load;
	}

	/* Set new freq */
	if (screen_off == 1)
		screen_off_freq_change(this_dbs_info, policy, max_load);
	else
		screen_on_freq_change(this_dbs_info, policy, max_load);
}		

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(current_sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	queue_delayed_work_on(cpu, dbs_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(current_sampling_rate);

	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice)
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}

		this_dbs_info->requested_freq = policy->cur;

		screen_off = 0;
		current_sampling_rate = 20000;
		last_cpu_load = 0;
	
		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;
			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		/*
		 * Stop the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SYNCEDPLUS
static
#endif
struct cpufreq_governor cpufreq_gov_syncedplus = {
	.name			= "syncedplus",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	u64 idle_time;
	int cpu = get_cpu();
	
	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		/*
		 * In nohz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}

	dbs_wq = alloc_workqueue("syncedplus_dbs_wq", WQ_HIGHPRI, 0);
	if (!dbs_wq) {
		printk(KERN_ERR "Failed to create syncedplus_dbs_wq workqueue\n");
		return -EFAULT;
	}

	syncedplus_lcd_notif.notifier_call = syncedplus_lcd_notifier_callback;
	if (lcd_register_client(&syncedplus_lcd_notif) != 0) {
		pr_err("%s: Failed to register lcd callback\n", __func__);
		return -EFAULT;
	}

	return cpufreq_register_governor(&cpufreq_gov_syncedplus);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_syncedplus);
	destroy_workqueue(dbs_wq);
	lcd_unregister_client(&syncedplus_lcd_notif);
}


MODULE_AUTHOR("Muhamed Trumic <muhamedtrumic97@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_syncedplus' - responsive cpufreq governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SYNCEDPLUS
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
