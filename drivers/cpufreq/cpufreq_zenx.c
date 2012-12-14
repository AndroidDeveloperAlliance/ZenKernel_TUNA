/*
 * drivers/cpufreq/cpufreq_zenx.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Mike Chan (mike@android.com)
 * Author: Brandon Berhent (bbedward@gmail.com)
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>

#include <asm/cputime.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_zenx.h>

static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_zenx_cpuinfo {
	struct timer_list cpu_timer;
	int timer_idlecancel;
	u64 time_in_idle;
	u64 time_in_idle_timestamp;
	u64 target_set_time;
	u64 target_set_time_in_idle;
	/*
	 * Measurement for how long cur_load has been
	 * above and below unplug_load[cpu].
	 */
	unsigned long total_below_unplug_time;
	unsigned long total_above_unplug_time;
	/*
	 * Last time we were there checking unplug_time
	 */
	u64 last_time_below_unplug_time;
	u64 last_time_above_unplug_time;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	unsigned int floor_freq;
	unsigned int cur_load;
	u64 floor_validate_time;
	u64 hispeed_validate_time;
	struct rw_semaphore mutex;
	int governor_enabled;
};

static DEFINE_PER_CPU(struct cpufreq_zenx_cpuinfo, cpuinfo);

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;
static struct mutex set_speed_lock;

/* realtime thread handles hotplugging */
static struct task_struct *hotplug_task;
static cpumask_t hotplug_cpumask;
static spinlock_t hotplug_cpumask_lock;

/* Hi speed to bump to from lo speed when load burst (default max) */
static unsigned int hispeed_freq;

/* Go to hi speed when CPU load at or above this value. */
#define DEFAULT_GO_HISPEED_LOAD 85
static unsigned long go_hispeed_load;

/* Unplug auxillary CPUs below these values. */
#define DEFAULT_UNPLUG_LOAD_CPU1 25
#define DEFAULT_UNPLUG_LOAD_CPU2 60
#define DEFAULT_UNPLUG_LOAD_CPU3 75

static unsigned int unplug_load[3];

/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 90
static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};
static spinlock_t target_loads_lock;
static unsigned int *target_loads = default_target_loads;
static int ntarget_loads = ARRAY_SIZE(default_target_loads);

/*
 * The minimum amount of time we should be <= unplug_load
 * before removing CPUs.
 */
#define DEFAULT_UNPLUG_DELAY (5000 * USEC_PER_MSEC)
static unsigned long unplug_delay;

/*
 * The minimum amount of time we should be > unplug_load
 * before inserting CPUs.
 */
#define DEFAULT_INSERT_DELAY (80 * USEC_PER_MSEC)
static unsigned long insert_delay;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_MIN_SAMPLE_TIME (80 * USEC_PER_MSEC)
static unsigned long min_sample_time;

/*
 * The sample rate of the timer used to increase frequency
 */
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
static unsigned long timer_rate;

/*
 * Wait this long before raising speed above hispeed, by default a single
 * timer interval.
 */
#define DEFAULT_ABOVE_HISPEED_DELAY DEFAULT_TIMER_RATE
static unsigned long above_hispeed_delay_val;

/*
 * Non-zero means longer-term speed boost active.
 */

static int boost_val;

static bool governidle;
module_param(governidle, bool, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(governidle,
	"Set to 1 to wake up CPUs from idle to reduce speed (default 0)");

static int cpufreq_governor_zenx(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ZENX
static
#endif
struct cpufreq_governor cpufreq_gov_zenx = {
	.name = "ZenX",
	.governor = cpufreq_governor_zenx,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void cpufreq_zenx_timer_resched(
	struct cpufreq_zenx_cpuinfo *pcpu)
{
	mod_timer_pinned(&pcpu->cpu_timer,
			 jiffies + usecs_to_jiffies(timer_rate));
	pcpu->time_in_idle =
		get_cpu_idle_time_us(smp_processor_id(),
				     &pcpu->time_in_idle_timestamp);
}

static unsigned int freq_to_targetload(unsigned int freq)
{
	int i;
	unsigned int ret;

	spin_lock(&target_loads_lock);

	for (i = 0; i < ntarget_loads - 1 && freq >= target_loads[i+1]; i += 2)
		;

	ret = target_loads[i];
	spin_unlock(&target_loads_lock);
	return ret;
}

/*
 * If increasing frequencies never map to a lower target load then
 * choose_freq() will find the minimum frequency that does not exceed its
 * target load given the current load.
 */

static unsigned int choose_freq(
	struct cpufreq_zenx_cpuinfo *pcpu, unsigned int curload)
{
	unsigned int freq = pcpu->policy->cur;
	unsigned int loadadjfreq = freq * curload;
	unsigned int prevfreq, freqmin, freqmax;
	unsigned int tl;
	int index;

	freqmin = 0;
	freqmax = UINT_MAX;

	do {
		prevfreq = freq;
		tl = freq_to_targetload(freq);

		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		cpufreq_frequency_table_target(
			pcpu->policy, pcpu->freq_table, loadadjfreq / tl,
			CPUFREQ_RELATION_L, &index);
		freq = pcpu->freq_table[index].frequency;

		if (freq > prevfreq) {
			/* The previous frequency is too low. */
			freqmin = prevfreq;

			if (freq >= freqmax) {
				/*
				 * Find the highest frequency that is less
				 * than freqmax.
				 */
				cpufreq_frequency_table_target(
					pcpu->policy, pcpu->freq_table,
					freqmax - 1, CPUFREQ_RELATION_H,
					&index);
				freq = pcpu->freq_table[index].frequency;

				if (freq == freqmin) {
					/*
					 * The first frequency below freqmax
					 * has already been found to be too
					 * low.  freqmax is the lowest speed
					 * we found that is fast enough.
					 */
					freq = freqmax;
					break;
				}
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough. */
			freqmax = prevfreq;

			if (freq <= freqmin) {
				/*
				 * Find the lowest frequency that is higher
				 * than freqmin.
				 */
				cpufreq_frequency_table_target(
					pcpu->policy, pcpu->freq_table,
					freqmin + 1, CPUFREQ_RELATION_L,
					&index);
				freq = pcpu->freq_table[index].frequency;

				/*
				 * If freqmax is the first frequency above
				 * freqmin then we have already found that
				 * this speed is fast enough.
				 */
				if (freq == freqmax)
					break;
			}
		}

		/* If same frequency chosen as previous then done. */
	} while (freq != prevfreq);

	return freq;
}

static void cpufreq_zenx_timer(unsigned long data)
{
	u64 now;
	unsigned int delta_idle;
	unsigned int delta_time;
	int cpu_load;
	int load_since_change;
	struct cpufreq_zenx_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	u64 now_idle;
	unsigned int new_freq;
	unsigned int index;
	unsigned long flags;

	if (!down_read_trylock(&pcpu->mutex))
		return;

	if (!pcpu->governor_enabled)
		goto exit;

	now_idle = get_cpu_idle_time_us(data, &now);
	delta_idle = (unsigned int)(now_idle - pcpu->time_in_idle);
	delta_time = (unsigned int)(now - pcpu->time_in_idle_timestamp);

	/*
	 * If timer ran less than 1ms after short-term sample started, retry.
	 */
	if (delta_time < 1000)
		goto rearm;

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (delta_time - delta_idle) / delta_time;

	delta_idle = (unsigned int)(now_idle - pcpu->target_set_time_in_idle);
	delta_time = (unsigned int)(now - pcpu->target_set_time);

	if ((delta_time == 0) || (delta_idle > delta_time))
		load_since_change = 0;
	else
		load_since_change =
			100 * (delta_time - delta_idle) / delta_time;

	/*
	 * Choose greater of short-term load (since last idle timer
	 * started or timer function re-armed itself) or long-term load
	 * (since last frequency change).
	 */
	if (load_since_change > cpu_load)
		cpu_load = load_since_change;

	if ((cpu_load >= go_hispeed_load || boost_val) &&
	    pcpu->target_freq < hispeed_freq)
		new_freq = hispeed_freq;
	else
		new_freq = choose_freq(pcpu, cpu_load);

	if (pcpu->target_freq >= hispeed_freq &&
	    new_freq > pcpu->target_freq &&
	    now - pcpu->hispeed_validate_time < above_hispeed_delay_val) {
		trace_cpufreq_zenx_notyet(
			data, cpu_load, pcpu->target_freq,
			pcpu->policy->cur, new_freq);
		goto rearm;
	}

	pcpu->hispeed_validate_time = now;

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_L,
					   &index)) {
		pr_warn_once("timer %d: cpufreq_frequency_table_target error\n",
			     (int) data);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */
	if (new_freq < pcpu->floor_freq) {
		if (now - pcpu->floor_validate_time < min_sample_time) {
			trace_cpufreq_zenx_notyet(
				data, cpu_load, pcpu->target_freq,
				pcpu->policy->cur, new_freq);
			goto rearm;
		}
	}

	pcpu->floor_freq = new_freq;
	pcpu->floor_validate_time = now;

	if (pcpu->target_freq == new_freq) {
		trace_cpufreq_zenx_already(
			data, cpu_load, pcpu->target_freq,
			pcpu->policy->cur, new_freq);
		goto rearm_if_notmax;
	}

	trace_cpufreq_zenx_target(data, cpu_load, pcpu->target_freq,
					 pcpu->policy->cur, new_freq);
	pcpu->target_set_time_in_idle = now_idle;
	pcpu->target_set_time = now;

	pcpu->target_freq = new_freq;
	spin_lock_irqsave(&speedchange_cpumask_lock, flags);
	cpumask_set_cpu(data, &speedchange_cpumask);
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);
	wake_up_process(speedchange_task);

	/* Set current load on this CPU */
	pcpu->cur_load = load_since_change;
	spin_lock_irqsave(&hotplug_cpumask_lock, flags);
	cpumask_set_cpu(data, &hotplug_cpumask);
	spin_unlock_irqrestore(&hotplug_cpumask_lock, flags);
	wake_up_process(hotplug_task);

rearm_if_notmax:
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

rearm:
	if (!timer_pending(&pcpu->cpu_timer)) {
		/*
		 * If governing speed in idle and already at min, cancel the
		 * timer if that CPU goes idle.  We don't need to re-evaluate
		 * speed until the next idle exit.
		 */
		if (governidle && pcpu->target_freq == pcpu->policy->min)
			pcpu->timer_idlecancel = 1;

		cpufreq_zenx_timer_resched(pcpu);
	}

exit:
	up_read(&pcpu->mutex);
	return;
}

static void cpufreq_zenx_idle_start(void)
{
	struct cpufreq_zenx_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!down_read_trylock(&pcpu->mutex))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->mutex);
		return;
	}

	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending) {
			pcpu->timer_idlecancel = 0;
			cpufreq_zenx_timer_resched(pcpu);
		}
	} else if (governidle) {
		/*
		 * If at min speed and entering idle after load has
		 * already been evaluated, and a timer has been set just in
		 * case the CPU suddenly goes busy, cancel that timer.  The
		 * CPU didn't go busy; we'll recheck things upon idle exit.
		 */
		if (pending && pcpu->timer_idlecancel) {
			del_timer(&pcpu->cpu_timer);
			pcpu->timer_idlecancel = 0;
		}
	}

	up_read(&pcpu->mutex);
}

static void cpufreq_zenx_idle_end(void)
{
	struct cpufreq_zenx_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	if (!down_read_trylock(&pcpu->mutex))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->mutex);
		return;
	}

	/* Arm the timer for 1-2 ticks later if not already. */
	if (!timer_pending(&pcpu->cpu_timer)) {
		pcpu->timer_idlecancel = 0;
		cpufreq_zenx_timer_resched(pcpu);
	} else if (!governidle &&
		   time_after_eq(jiffies, pcpu->cpu_timer.expires)) {
		del_timer(&pcpu->cpu_timer);
		cpufreq_zenx_timer(smp_processor_id());
	}

	up_read(&pcpu->mutex);
}

static int cpufreq_zenx_hotplug_task(void *data)
{
	u64 now;
	cpumask_t tmp_mask;
	unsigned int cpu;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&hotplug_cpumask_lock, flags);

		if (cpumask_empty(&hotplug_cpumask)) {
			spin_unlock_irqrestore(&hotplug_cpumask_lock,
					       flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&hotplug_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = hotplug_cpumask;
		cpumask_clear(&hotplug_cpumask);
		spin_unlock_irqrestore(&hotplug_cpumask_lock, flags);

		get_cpu_idle_time_us(smp_processor_id(), &now);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j, avg_load;
			unsigned int total_load = 0;

			pcpu = &per_cpu(cpuinfo, cpu);

			if (!down_read_trylock(&pcpu->mutex))
				continue;
			if (!pcpu->governor_enabled) {
				up_read(&pcpu->mutex);
				continue;
			}

			/*
			 * Compute average CPU load
			 * Use cpu_online_mask to get the load across
			 * all online CPUs.
			 */
			for_each_cpu(j, cpu_online_mask) {
				struct cpufreq_zenx_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				total_load += pjcpu->cur_load;
			}
			avg_load = total_load / num_online_cpus();

			/*
			 * Determine which CPUs to remove/insert.
			 * Use cpu_possible_mask so we get online
			 * and offline CPUs.
			 */
			for_each_possible_cpu(j) {
				struct cpufreq_zenx_cpuinfo *pjcpu;

				if (j == 0)
					continue;
				else if (j > 3)
					break;

				pjcpu = &per_cpu(cpuinfo, j);

				/*
				 * The logic for hotplugging works as follows:
				 * if avg_load <= unplug_load[cpu], reset timers
				 * about how long we've been ABOVE it and
				 * figure out how long it has been since we've
				 * been below unplug_load.
				 * Logic works the same for last time we were above
				 * unplug_load.
				 */
				if (avg_load <= unplug_load[j - 1]) {
					/* Below: reset above counter */
					pjcpu->total_above_unplug_time = 0;
					pjcpu->last_time_above_unplug_time = 0;
					if (!pjcpu->last_time_below_unplug_time)
						pjcpu->last_time_below_unplug_time = now;
					pjcpu->total_below_unplug_time +=
						now - pjcpu->last_time_below_unplug_time;
				}
				if (avg_load > unplug_load[j - 1]) {
					/* Above: reset below counter */
					pjcpu->total_below_unplug_time = 0;
					pjcpu->last_time_below_unplug_time = 0;
					if (!pjcpu->last_time_above_unplug_time)
						pjcpu->last_time_above_unplug_time = now;
					pjcpu->total_above_unplug_time +=
						now - pjcpu->last_time_above_unplug_time;
				}

				/*
				 * If CPU is not online, it must be offline so there should
				 * be no need to do another cpu_online check.
				 * Also avoid the likely/unlikely branch prediction macros
				 * as we have no idea if it's online or offline.
				 */
				if (cpu_online(j) &&
					pjcpu->total_below_unplug_time > unplug_delay) {
					mutex_lock(&set_speed_lock);
					cpu_down(j);
					mutex_unlock(&set_speed_lock);
				} else if (pjcpu->total_above_unplug_time > insert_delay) {
					mutex_lock(&set_speed_lock);
					cpu_up(j);
					mutex_unlock(&set_speed_lock);
				}
			}

			up_read(&pcpu->mutex);
		}
	}

	return 0;
}

static int cpufreq_zenx_speedchange_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&speedchange_cpumask_lock, flags);

		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
					       flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq = 0;

			pcpu = &per_cpu(cpuinfo, cpu);

			if (!down_read_trylock(&pcpu->mutex))
				continue;
			if (!pcpu->governor_enabled) {
				up_read(&pcpu->mutex);
				continue;
			}

			mutex_lock(&set_speed_lock);

			for_each_cpu(j, pcpu->policy->cpus) {
				struct cpufreq_zenx_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				if (pjcpu->target_freq > max_freq)
					max_freq = pjcpu->target_freq;
			}

			if (max_freq != pcpu->policy->cur)
				__cpufreq_driver_target(pcpu->policy,
							max_freq,
							CPUFREQ_RELATION_H);
			mutex_unlock(&set_speed_lock);
			trace_cpufreq_zenx_setspeed(cpu,
						     pcpu->target_freq,
						     pcpu->policy->cur);

			up_read(&pcpu->mutex);
		}
	}

	return 0;
}

static void cpufreq_zenx_boost(void)
{
	int i;
	int anyboost = 0;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	spin_lock_irqsave(&speedchange_cpumask_lock, flags);

	for_each_online_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);

		if (pcpu->target_freq < hispeed_freq) {
			pcpu->target_freq = hispeed_freq;
			cpumask_set_cpu(i, &speedchange_cpumask);
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time_us(i, &pcpu->target_set_time);
			pcpu->hispeed_validate_time = pcpu->target_set_time;
			anyboost = 1;
		}

		/*
		 * Set floor freq and (re)start timer for when last
		 * validated.
		 */

		pcpu->floor_freq = hispeed_freq;
		pcpu->floor_validate_time = ktime_to_us(ktime_get());
	}

	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

	if (anyboost)
		wake_up_process(speedchange_task);
}

static ssize_t show_target_loads(
	struct kobject *kobj, struct attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;

	spin_lock(&target_loads_lock);

	for (i = 0; i < ntarget_loads; i++)
		ret += sprintf(buf + ret, "%u%s", target_loads[i],
			       i & 0x1 ? ":" : " ");

	ret += sprintf(buf + ret, "\n");
	spin_unlock(&target_loads_lock);
	return ret;
}

static ssize_t store_target_loads(
	struct kobject *kobj, struct attribute *attr, const char *buf,
	size_t count)
{
	int ret;
	const char *cp;
	unsigned int *new_target_loads = NULL;
	int ntokens = 1;
	int i;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err_inval;

	new_target_loads = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!new_target_loads) {
		ret = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &new_target_loads[i++]) != 1)
			goto err_inval;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_inval;

	spin_lock(&target_loads_lock);
	if (target_loads != default_target_loads)
		kfree(target_loads);
	target_loads = new_target_loads;
	ntarget_loads = ntokens;
	spin_unlock(&target_loads_lock);
	return count;

err_inval:
	ret = -EINVAL;
err:
	kfree(new_target_loads);
	return ret;
}

static struct global_attr target_loads_attr =
	__ATTR(target_loads, S_IRUGO | S_IWUSR,
		show_target_loads, store_target_loads);

static ssize_t show_hispeed_freq(struct kobject *kobj,
				 struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hispeed_freq);
}

static ssize_t store_hispeed_freq(struct kobject *kobj,
				  struct attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	long unsigned int val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	hispeed_freq = val;
	return count;
}

static struct global_attr hispeed_freq_attr = __ATTR(hispeed_freq, 0644,
		show_hispeed_freq, store_hispeed_freq);


static ssize_t show_go_hispeed_load(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", go_hispeed_load);
}

static ssize_t store_go_hispeed_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	go_hispeed_load = val;
	return count;
}

static struct global_attr go_hispeed_load_attr = __ATTR(go_hispeed_load, 0644,
		show_go_hispeed_load, store_go_hispeed_load);

static ssize_t show_unplug_load_cpu1(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", unplug_load[0]);
}

static ssize_t store_unplug_load_cpu1(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	unplug_load[0] = val;
        return count;
}

static struct global_attr unplug_load_cpu1_attr = __ATTR(unplug_load_cpu1, 0644,
	show_unplug_load_cpu1, store_unplug_load_cpu1);

static ssize_t show_unplug_load_cpu2(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", unplug_load[0]);
}

static ssize_t store_unplug_load_cpu2(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	unplug_load[1] = val;
        return count;
}

static struct global_attr unplug_load_cpu2_attr = __ATTR(unplug_load_cpu2, 0644,
	show_unplug_load_cpu2, store_unplug_load_cpu2);

static ssize_t show_unplug_load_cpu3(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", unplug_load[2]);
}

static ssize_t store_unplug_load_cpu3(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	unplug_load[2] = val;
        return count;
}

static struct global_attr unplug_load_cpu3_attr = __ATTR(unplug_load_cpu3, 0644,
	show_unplug_load_cpu3, store_unplug_load_cpu3);

static ssize_t show_unplug_delay(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", unplug_delay);
}

static ssize_t store_unplug_delay(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	unplug_delay = val;
	return count;
}

static struct global_attr unplug_delay_attr = __ATTR(unplug_delay, 0644,
		show_unplug_delay, store_unplug_delay);

static ssize_t show_insert_delay(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", insert_delay);
}

static ssize_t store_insert_delay(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	insert_delay = val;
	return count;
}

static struct global_attr insert_delay_attr = __ATTR(insert_delay, 0644,
		show_insert_delay, store_insert_delay);

static ssize_t show_min_sample_time(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	min_sample_time = val;
	return count;
}

static struct global_attr min_sample_time_attr = __ATTR(min_sample_time, 0644,
		show_min_sample_time, store_min_sample_time);

static ssize_t show_above_hispeed_delay(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", above_hispeed_delay_val);
}

static ssize_t store_above_hispeed_delay(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	above_hispeed_delay_val = val;
	return count;
}

define_one_global_rw(above_hispeed_delay);

static ssize_t show_timer_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate = val;
	return count;
}

static struct global_attr timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static ssize_t show_boost(struct kobject *kobj, struct attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", boost_val);
}

static ssize_t store_boost(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	boost_val = val;

	if (boost_val) {
		trace_cpufreq_zenx_boost("on");
		cpufreq_zenx_boost();
	} else {
		trace_cpufreq_zenx_unboost("off");
	}

	return count;
}

define_one_global_rw(boost);

static ssize_t store_boostpulse(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	trace_cpufreq_zenx_boost("pulse");
	cpufreq_zenx_boost();
	return count;
}

static struct global_attr boostpulse =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse);

static struct attribute *zenx_attributes[] = {
	&target_loads_attr.attr,
	&hispeed_freq_attr.attr,
	&go_hispeed_load_attr.attr,
	&above_hispeed_delay.attr,
	&unplug_load_cpu1_attr.attr,
	&unplug_load_cpu2_attr.attr,
	&unplug_load_cpu3_attr.attr,
	&unplug_delay_attr.attr,
	&insert_delay_attr.attr,
	&min_sample_time_attr.attr,
	&timer_rate_attr.attr,
	&boost.attr,
	&boostpulse.attr,
	NULL,
};

static struct attribute_group zenx_attr_group = {
	.attrs = zenx_attributes,
	.name = "zenx",
};

#ifdef CONFIG_EARLYSUSPEND
/*
 * Enable all CPUs when waking up the device
 */
static void zenx_late_resume(struct early_suspend *handler) {
	unsigned int cpu;
	struct cpufreq_zenx_cpuinfo *pcpu;

	for_each_cpu_not(cpu, cpu_online_mask) {
		pcpu = &per_cpu(cpuinfo, cpu);
		if (!down_read_trylock(&pcpu->mutex))
			continue;
		if (!pcpu->governor_enabled) {
			up_read(&pcpu->mutex);
			continue;
		}
		mutex_lock(&set_speed_lock);
		cpu_up(cpu);
		mutex_unlock(&set_speed_lock);
		up_read(&pcpu->mutex);
	}
}

static struct early_suspend zenx_early_suspend = {
	.resume = zenx_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif

static int cpufreq_zenx_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	switch (val) {
	case IDLE_START:
		cpufreq_zenx_idle_start();
		break;
	case IDLE_END:
		cpufreq_zenx_idle_end();
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_zenx_idle_nb = {
	.notifier_call = cpufreq_zenx_idle_notifier,
};

static int cpufreq_governor_zenx(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j;
	struct cpufreq_zenx_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu))
			return -EINVAL;

		freq_table =
			cpufreq_frequency_get_table(policy->cpu);
		if (!hispeed_freq)
			hispeed_freq = policy->max;

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time_us(j,
					     &pcpu->target_set_time);
			pcpu->floor_freq = pcpu->target_freq;
			pcpu->floor_validate_time =
				pcpu->target_set_time;
			pcpu->hispeed_validate_time =
				pcpu->target_set_time;
			pcpu->governor_enabled = 1;
			pcpu->total_below_unplug_time = 0;
			pcpu->last_time_below_unplug_time = 0;
			pcpu->cur_load = 0;
			smp_wmb();
			pcpu->cpu_timer.expires =
				jiffies + usecs_to_jiffies(timer_rate);
			add_timer_on(&pcpu->cpu_timer, j);
		}

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

		rc = sysfs_create_group(cpufreq_global_kobject,
				&zenx_attr_group);
		if (rc)
			return rc;

#ifdef CONFIG_EARLYSUSPEND
		register_early_suspend(&zenx_early_suspend);
#endif
		idle_notifier_register(&cpufreq_zenx_idle_nb);
		break;

	case CPUFREQ_GOV_STOP:
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			down_write(&pcpu->mutex);
			pcpu->governor_enabled = 0;
			del_timer_sync(&pcpu->cpu_timer);
			up_write(&pcpu->mutex);
		}

		if (atomic_dec_return(&active_count) > 0)
			return 0;

		idle_notifier_unregister(&cpufreq_zenx_idle_nb);
		sysfs_remove_group(cpufreq_global_kobject,
				&zenx_attr_group);

#ifdef CONFIG_EARLYSUSPEND
		unregister_early_suspend(&zenx_early_suspend);
#endif
		/*
		 * XXX BIG CAVEAT: Stopping the governor with CPU1 offline
		 * will result in it remaining offline until the user onlines
		 * it again.  It is up to the user to do this (for now).
		 */
		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static int __init cpufreq_zenx_init(void)
{
	unsigned int i;
	struct cpufreq_zenx_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
	unplug_load[0] = DEFAULT_UNPLUG_LOAD_CPU1;
	unplug_load[1] = DEFAULT_UNPLUG_LOAD_CPU2;
	unplug_load[2] = DEFAULT_UNPLUG_LOAD_CPU3;
	unplug_delay = DEFAULT_UNPLUG_DELAY;
	insert_delay = DEFAULT_INSERT_DELAY;
	min_sample_time = DEFAULT_MIN_SAMPLE_TIME;
	above_hispeed_delay_val = DEFAULT_ABOVE_HISPEED_DELAY;
	timer_rate = DEFAULT_TIMER_RATE;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		if (governidle)
			init_timer(&pcpu->cpu_timer);
		else
			init_timer_deferrable(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_zenx_timer;
		pcpu->cpu_timer.data = i;
		init_rwsem(&pcpu->mutex);
	}

	spin_lock_init(&target_loads_lock);
	spin_lock_init(&speedchange_cpumask_lock);
	mutex_init(&set_speed_lock);

	speedchange_task =
		kthread_create(cpufreq_zenx_speedchange_task, NULL,
			       "cfzenx");
	if (IS_ERR(speedchange_task))
		return PTR_ERR(speedchange_task);

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);

	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(speedchange_task);

	spin_lock_init(&hotplug_cpumask_lock);
	hotplug_task =
		kthread_create(cpufreq_zenx_hotplug_task, NULL,
				"hpzenx");
	if (IS_ERR(hotplug_task))
		return PTR_ERR(hotplug_task);

	sched_setscheduler_nocheck(hotplug_task, SCHED_FIFO, &param);
	get_task_struct(hotplug_task);

	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(hotplug_task);

	return cpufreq_register_governor(&cpufreq_gov_zenx);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ZENX
fs_initcall(cpufreq_zenx_init);
#else
module_init(cpufreq_zenx_init);
#endif

static void __exit cpufreq_zenx_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_zenx);
	/* Stop the threads */
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
	kthread_stop(hotplug_task);
	put_task_struct(hotplug_task);
}

module_exit(cpufreq_zenx_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_DESCRIPTION("'cpufreq_zenx' - A cpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
