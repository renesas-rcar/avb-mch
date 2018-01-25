/*************************************************************************/ /*
 avb-mch

 Copyright (C) 2016-2017 Renesas Electronics Corporation

 License        Dual MIT/GPLv2

 The contents of this file are subject to the MIT license as set out below.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 Alternatively, the contents of this file may be used under the terms of
 the GNU General Public License Version 2 ("GPL") in which case the provisions
 of GPL are applicable instead of those above.

 If you wish to allow use of your version of this file only under the terms of
 GPL, and not to allow others to use your version of this file under the terms
 of the MIT license, indicate your decision by deleting the provisions above
 and replace them with the notice and other provisions required by GPL as set
 out in the file called "GPL-COPYING" included in this distribution. If you do
 not delete the provisions above, a recipient may use your version of this file
 under the terms of either the MIT license or GPL.

 This License is also included in this distribution in the file called
 "MIT-COPYING".

 EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 GPLv2:
 If you wish to use this file under the terms of GPL, following terms are
 effective.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ /*************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <../drivers/net/ethernet/renesas/ravb.h>
#include "mch_core.h"

struct mch_private *mch_priv_ptr;

#define PPB_SCALE           (1000000000ULL)
#define S0D1                (800 * 1000000)
#define AVB_COUNTER_FRQ     (796800) /* 796812.7 / (16 * 166) = 300.0048Hz */
#define AVB_COUNTER_FRQ_MIN (S0D1 / (0x3FFC0 / 32))
#define AVB_COUNTER_FRQ_MAX 25000000
#define CS2000_ID           0x06

#define MCH_CORRECT_CYCLE      (5 * 1000000) /* 5msec */
#define MCH_TIME_TOLERANCE     (2 * 1000000) /* 2msec */
#define MCH_CORRECT_JUDGE_OVER (3 * 1000000) /* 3msec */
#define MCH_CORRECT_JUDGE      (300)         /* 300nsec */

struct reg_val_table {
	const char *name;
	u32 val;
};

enum ADG_SYNC_SEL {
	ADG_SYNC_SEL_AUDIO_CLK_A = 0x08,
	ADG_SYNC_SEL_AUDIO_CLK_B,
	ADG_SYNC_SEL_AUDIO_CLK_C,
	ADG_SYNC_SEL_WS0 = 0x10,
	ADG_SYNC_SEL_WS1,
	ADG_SYNC_SEL_WS2,
	ADG_SYNC_SEL_WS3,
	ADG_SYNC_SEL_WS4,
	ADG_SYNC_SEL_WS5,
	ADG_SYNC_SEL_WS6,
	ADG_SYNC_SEL_WS7,
	ADG_SYNC_SEL_WS8_RESERVE,    /* WS8 is reserved */
	ADG_SYNC_SEL_WS9,
	ADG_SYNC_SEL_AVB_COUNTER0 = 0x20,
	ADG_SYNC_SEL_AVB_COUNTER1,
	ADG_SYNC_SEL_AVB_COUNTER2,
	ADG_SYNC_SEL_AVB_COUNTER3,
	ADG_SYNC_SEL_AVB_COUNTER4,
	ADG_SYNC_SEL_AVB_COUNTER5,
	ADG_SYNC_SEL_AVB_COUNTER6,
	ADG_SYNC_SEL_AVB_COUNTER7,
};

static struct reg_val_table avtp_cap_sync_sel_tbl[] = {
	{ "audio_clk_a",  ADG_SYNC_SEL_AUDIO_CLK_A },
	{ "audio_clk_b",  ADG_SYNC_SEL_AUDIO_CLK_B },
	{ "ws0",          ADG_SYNC_SEL_WS0 },
	{ "avb_counter0", ADG_SYNC_SEL_AVB_COUNTER0 },
	{ "avb_counter1", ADG_SYNC_SEL_AVB_COUNTER1 },
	{ "avb_counter2", ADG_SYNC_SEL_AVB_COUNTER2 },
	{ "avb_counter3", ADG_SYNC_SEL_AVB_COUNTER3 },
	{ "avb_counter4", ADG_SYNC_SEL_AVB_COUNTER4 },
	{ "avb_counter5", ADG_SYNC_SEL_AVB_COUNTER5 },
	{ "avb_counter6", ADG_SYNC_SEL_AVB_COUNTER6 },
	{ "avb_counter7", ADG_SYNC_SEL_AVB_COUNTER7 },
	{ },
};

enum CS2000 {
	CS2000_DEVICE_ID        = 0x01,
	CS2000_DEVICE_CTRL      = 0x02,
	CS2000_DEVICE_CFG1      = 0x03,
	CS2000_DEVICE_CFG2      = 0x04,
	CS2000_GLOBAL_CFG       = 0x05,
	CS2000_32BIT_RATIO0     = 0x06,
	CS2000_32BIT_RATIO1     = 0x0a,
	CS2000_32BIT_RATIO2     = 0x0e,
	CS2000_32BIT_RATIO3     = 0x12,
	CS2000_FUNCT_CFG1       = 0x16,
	CS2000_FUNCT_CFG2       = 0x17,
	CS2000_FUNCT_CFG3       = 0x1e,
	CS2000_REG_END,
};

enum TASK_EVENT {
	TASK_EVENT_NONE = 0x00,
	TASK_EVENT_PROC = 0x01,
	TASK_EVENT_TERM = 0x02,
};

static char *interface = "eth0";
module_param(interface, charp, 0440);

static int avtp_cap_cycle = 300;
module_param(avtp_cap_cycle, int, 0660);

static char *avtp_clk_name = "avb_counter0";
module_param(avtp_clk_name, charp, 0440);

static int avtp_clk_frq = AVB_COUNTER_FRQ;
module_param(avtp_clk_frq, int, 0440);

static int sample_rate = 48000;
module_param(sample_rate, int, 0440);

static u8 cs2000_data_bk[CS2000_REG_END];
static u8 cs2000_data[CS2000_REG_END];

static DEFINE_SPINLOCK(mch_lock);

static u32 get_avtp_cap_sync_sel_by_name(const char *name);
static u32 calc_avb_counter(u32 frq);

static int mch_clk_correct(struct mch_device *m_dev, s64 diff)
{
	struct mch_private *priv = m_dev->priv;
	u32 reg, val;
	int correct = 0;

	reg = get_avtp_cap_sync_sel_by_name(priv->param.avtp_clk_name);

	if ((reg < ADG_SYNC_SEL_AVB_COUNTER0) ||
	    (reg > ADG_SYNC_SEL_AVB_COUNTER7))
		return -EPERM;

	reg -= ADG_SYNC_SEL_AVB_COUNTER0;

	/*
	 * +Over        0          -Over
	 *  |  +JUDGE   |  -JUDGE   |
	 * -o-----o-----------o-----o-
	 * 011111100000000000001111110
	 */
	if ((diff > MCH_CORRECT_JUDGE_OVER) ||
	    (diff < -MCH_CORRECT_JUDGE_OVER)) {
		correct = 0;
	} else {
		if (diff > MCH_CORRECT_JUDGE)
			correct = 1;
		else if (diff < -MCH_CORRECT_JUDGE)
			correct = -1;
		else
			correct = 0;
	}

	if (m_dev->correct != correct) {
		val = calc_avb_counter(priv->param.avtp_clk_frq);
		val += correct;

		pr_debug("clk correct %d to %d MCH freq (%u)  avb_counter (%08x)\n",
			 m_dev->correct, correct,
			 (u32)((32 * (u64)S0D1) / val), val);
		mch_adg_avb_write(priv, AVB_CLK_DIV0 + (reg * 4), val);
		m_dev->correct = correct;
	}

	return 0;
}

static int mch_task(void *param)
{
	struct mch_device *m_dev = param;
	struct mch_timestamps *ts;
	unsigned int master_time = 0, device_time = 0;
	u64 pre_m_time, pre_d_time;
	u64 guess_m_time, guess_d_time;
	s64 m_tmp, d_tmp;
	s64 diff, diff_ave, pre_diff;
	int ret;
	int i, diff_cnt;
	unsigned long flags;
	u32 events;

	pr_debug("create mch task\n");

	while (!kthread_should_stop()) {
		if (list_empty(&m_dev->timestamps))
			ret = wait_event_interruptible(m_dev->wait_event,
						       m_dev->pending_events);
		else
			ret = 0;

		pr_debug("mch task run\n");
		events = m_dev->pending_events;
		m_dev->pending_events = TASK_EVENT_NONE;

		if (ret < 0) {
			pr_err("wait_event error\n");
			continue;
		}

		if (events & TASK_EVENT_TERM) {
			pr_debug("mch task terminate\n");
			/* restore clock correction */
			diff_ave = 0;
			mch_clk_correct(m_dev, diff_ave);
			break;
		}

		pr_debug("mch task proc\n");

		spin_lock_irqsave(&mch_lock, flags);

		ts = list_first_entry_or_null(&m_dev->timestamps,
					      struct mch_timestamps,
					      list);
		if (ts)
			list_del(&ts->list);

		spin_unlock_irqrestore(&mch_lock, flags);

		if (!ts) {
			pr_err("wait_event error\n");
			continue;
		}

		pre_m_time = ts->ts[0].master;
		pre_d_time = ts->ts[0].device;

		diff = 0;
		diff_ave = 0;
		pre_diff = 0;
		diff_cnt = 0;

		pr_debug("task loop start ts:%p count(%d)\n", ts, ts->count);

		for (i = 1; i < ts->count; i++) {
			if (m_dev->pending_events & TASK_EVENT_TERM) {
				pr_debug("mch task terminate req\n");
				diff_ave = 0;   /* restore clock correction */
				break;
			}

			master_time = ts->ts[i].master;
			device_time = ts->ts[i].device;

			guess_m_time = pre_m_time + m_dev->interval;
			guess_d_time = pre_d_time + m_dev->interval;

			m_tmp = master_time;
			if (master_time < pre_m_time)	/* wrap around */
				m_tmp += U32_MAX;

			d_tmp = device_time;
			if (device_time < pre_d_time)	/* wrap around */
				d_tmp += U32_MAX;

			diff = 0;
			if ((m_tmp >= guess_m_time - MCH_TIME_TOLERANCE) &&
			    (m_tmp <= guess_m_time + MCH_TIME_TOLERANCE) &&
			    (d_tmp >= guess_d_time - MCH_TIME_TOLERANCE) &&
			    (d_tmp <= guess_d_time + MCH_TIME_TOLERANCE)) {
				if ((m_tmp - d_tmp) < -(s32)(U32_MAX >> 1)) {
					if (pre_diff >= 0) { /* master gose ahead */
						diff = (m_tmp + U32_MAX) - d_tmp;
						diff_cnt++;
					} else {
						pr_debug("timestamp is too far away m(%u) d(%u)\n",
							 master_time,
							 device_time);
					}
				} else if ((m_tmp - d_tmp) > (U32_MAX >> 1)) {
					if (pre_diff <= 0) {/* device gose ahead */
						diff = m_tmp - (d_tmp + U32_MAX);
						diff_cnt++;
					} else {
						pr_debug("timestamp is too far away m(%u) d(%u)\n",
							 master_time,
							 device_time);
					}
				} else {
					diff = m_tmp - d_tmp;
					diff_cnt++;
				}

				diff_ave += diff;
				pre_diff = diff;
			} else {
				pr_debug("range out: master(%llu/%llu) device(%llu/%llu)\n",
					 m_tmp, guess_m_time,
					 d_tmp, guess_d_time);
			}

			pre_m_time = master_time;
			pre_d_time = device_time;
		}

		if (diff_cnt)
			diff_ave /= diff_cnt;

		mch_clk_correct(m_dev, diff_ave);

		kfree(ts->ts);
		kfree(ts);
	}

	pr_debug("mch task end\n");

	return 0;
}

static int init_mch_device(struct mch_device *m_dev)
{
	char taskname[32] = { '\0' };

	init_waitqueue_head(&m_dev->wait_event);
	spin_lock_init(&m_dev->qlock);

	INIT_LIST_HEAD(&m_dev->timestamps);

	m_dev->correct = 0;

	sprintf(taskname, "mch_task");
	m_dev->task = kthread_run(mch_task, m_dev, taskname);
	if (IS_ERR(m_dev->task))
		return -1;

	return 0;
}

/*
 * public functions
 */
void *mch_open(void)
{
	struct mch_private *priv = mch_priv_ptr;
	struct mch_device *m_dev = NULL;
	unsigned long flags;

	if (!priv)
		return NULL;

	if (priv->m_dev[0])
		return NULL;

	m_dev = kzalloc(sizeof(*m_dev), GFP_KERNEL);
	if (!m_dev)
		return NULL;

	if (init_mch_device(m_dev)) {
		kfree(m_dev);
		return NULL;
	}

	spin_lock_irqsave(&mch_lock, flags);

	m_dev->priv = priv;
	priv->m_dev[0] = m_dev;

	spin_unlock_irqrestore(&mch_lock, flags);

	pr_info("registered mch device %p\n", m_dev);

	return (void *)m_dev;
}
EXPORT_SYMBOL(mch_open);

int mch_close(void *mch)
{
	struct mch_private *priv;
	struct mch_device *m_dev = (struct mch_device *)mch;
	unsigned long flags;
	struct mch_timestamps *ts, *tmp;

	if (!m_dev)
		return -EINVAL;

	priv = m_dev->priv;
	if (!priv)
		return -ENODEV;

	m_dev->pending_events = TASK_EVENT_TERM;
	wake_up_interruptible(&m_dev->wait_event);
	kthread_stop(m_dev->task);

	spin_lock_irqsave(&mch_lock, flags);

	list_for_each_entry_safe(ts, tmp, &m_dev->timestamps, list) {
		list_del(&ts->list);
		kfree(ts->ts);
		kfree(ts);
	}

	kfree(m_dev);

	priv->m_dev[0] = NULL;

	spin_unlock_irqrestore(&mch_lock, flags);

	pr_info("close mch device\n");

	return 0;
}
EXPORT_SYMBOL(mch_close);

int mch_set_interval(void *mch, u32 ns)
{
	struct mch_device *m_dev = (struct mch_device *)mch;

	if (!m_dev)
		return -EINVAL;

	m_dev->interval = ns;

	return 0;
}
EXPORT_SYMBOL(mch_set_interval);

int mch_send_timestamps(void *mch,
			struct mch_timestamp *ts,
			int count)
{
	struct mch_private *priv;
	struct mch_device *m_dev = (struct mch_device *)mch;
	struct mch_timestamps *timestamps;
	int i;
	unsigned long flags;

	if (!m_dev)
		return -EINVAL;

	priv = m_dev->priv;

	if (!priv)
		return -ENODEV;

	if (!m_dev->interval)
		return -EPERM;

	if (count < 1)
		return -EINVAL;

	if (!ts)
		return -EINVAL;

	timestamps = kzalloc(sizeof(*timestamps), GFP_KERNEL);
	if (!timestamps)
		return -ENOMEM;

	timestamps->ts = kcalloc(count, sizeof(*timestamps->ts),
				 GFP_KERNEL);
	if (!timestamps->ts) {
		kfree(timestamps);
		return -ENOMEM;
	}

	for (i = 0; i < count; i++)
		timestamps->ts[i] = ts[i];
	timestamps->count = count;

	pr_debug("send ts addr : %p\n", timestamps);
	pr_debug("copy-timestamp count(%d)\n", count);

	spin_lock_irqsave(&mch_lock, flags);
	list_add_tail(&timestamps->list, &m_dev->timestamps);
	spin_unlock_irqrestore(&mch_lock, flags);

	pr_debug("mch_task wakeup req\n");
	m_dev->pending_events |= TASK_EVENT_PROC;
	wake_up_interruptible(&m_dev->wait_event);

	pr_debug("%s count %d\n", __func__, count);

	return 0;
}
EXPORT_SYMBOL(mch_send_timestamps);

int mch_get_recovery_value(void *mch, int *value)
{
	if (!mch)
		return -EINVAL;

	if (!value)
		return -EINVAL;

	*value = PPB_SCALE;

	return 0;
}
EXPORT_SYMBOL(mch_get_recovery_value);

static u32 get_avtp_cap_sync_sel_by_name(const char *name)
{
	struct reg_val_table *tbl = avtp_cap_sync_sel_tbl;

	for ( ; tbl->name; tbl++) {
		if (!strcasecmp(name, tbl->name))
			return tbl->val;
	}

	return 0;
}

static int mch_check_params(struct mch_param *param)
{
	u32 reg;

	if ((param->avtp_clk_frq < AVB_COUNTER_FRQ_MIN) ||
	    (param->avtp_clk_frq > AVB_COUNTER_FRQ_MAX)) {
		pr_err("invalid param.\n");
		return -EINVAL;
	}

	reg = get_avtp_cap_sync_sel_by_name(param->avtp_clk_name);
	if (reg != ADG_SYNC_SEL_AVB_COUNTER0) {
		pr_err("invalid param.\n");
		return -EINVAL;
	}

	if (param->sample_rate % 48000) {
		pr_err("invalid param.\n");
		return -EINVAL;
	}

	return 0;
}

static u32 calc_avb_counter(u32 frq)
{
	if (!frq)
		return 0;

	if ((frq < AVB_COUNTER_FRQ_MIN) || (frq > AVB_COUNTER_FRQ_MAX))
		return 0;

	return  (u32)((32 * (u64)S0D1) / frq);
}

static int mch_regist_adg(struct mch_private *priv)
{
	/* clear AVB_SYNC_SEL0-2 */
	mch_adg_avb_write(priv, AVB_SYNC_SEL0, 0x00000000);
	mch_adg_avb_write(priv, AVB_SYNC_SEL1, 0x00000000);
	mch_adg_avb_write(priv, AVB_SYNC_SEL2, 0x00000000);

	/* avb_count8 enable */
	mch_adg_avb_write(priv, AVB_CLK_CONFIG, 0x80000000);

	return 0;
}

int mch_set_adg_avb_sync_sel(struct mch_private *priv, int ch,
			     char *clk_name)
{
	u32 sync_sel, reg, val;

	switch (ch / 4) {
	case 0:
		reg = AVB_SYNC_SEL2;
		break;
	case 1:
		reg = AVB_SYNC_SEL1;
		break;
	case 2:
		reg = AVB_SYNC_SEL0;
		break;
	default:
		return -1;
	}
	val = mch_adg_avb_read(priv, reg);
	sync_sel = get_avtp_cap_sync_sel_by_name(clk_name);

	val |= (sync_sel << ((ch % 4) * 8));
	mch_adg_avb_write(priv, reg, val);

	return 0;
}

static int mch_set_adg_avb_sync_div(struct mch_private *priv, int freq,
				    int cap_cycle, char *clk_name)
{
	u32 sync_sel, clk_no, reg, val;
	int i, tmp;

	sync_sel = get_avtp_cap_sync_sel_by_name(clk_name);

	if ((sync_sel >= ADG_SYNC_SEL_AUDIO_CLK_A) &&
	    (sync_sel <= ADG_SYNC_SEL_AUDIO_CLK_C)) {
		reg = AVB_SYNC_DIV0;
		clk_no = sync_sel - ADG_SYNC_SEL_AUDIO_CLK_A;
	} else if ((sync_sel >= ADG_SYNC_SEL_AVB_COUNTER0) &&
		   (sync_sel <= ADG_SYNC_SEL_AVB_COUNTER7)) {
		reg = AVB_SYNC_DIV1;
		clk_no = sync_sel - ADG_SYNC_SEL_AVB_COUNTER0;
	} else {
		return -1;
	}

	/* divid */
	for (i = 0; i < 16; i++) {
		tmp = freq >> i;
		if ((tmp / 256) < cap_cycle)
			break;
	}
	if (i == 16)
		return -1;

	val = mch_adg_avb_read(priv, reg);
	val |= (i << (clk_no * 4));
	mch_adg_avb_write(priv, reg, val);

	return 0;
}

static int mch_set_adg_avbckr(struct mch_private *priv, char *clk_name)
{
	u32 sync_sel, clk_no, val;

	sync_sel = get_avtp_cap_sync_sel_by_name(clk_name);
	if ((sync_sel >= ADG_SYNC_SEL_AVB_COUNTER0) &&
	    (sync_sel <= ADG_SYNC_SEL_AVB_COUNTER7))
		clk_no = sync_sel - ADG_SYNC_SEL_AVB_COUNTER0;
	else
		return -1;

	val = 0x00000008 | clk_no;
	mch_adg_avb_write(priv, AVBCKR, val);

	return 0;
}

static int mch_set_adg_avb_clk_div(struct mch_private *priv, int freq,
				   char *clk_name)
{
	u32 sync_sel, clk_no, val, reg, calc_frq;

	sync_sel = get_avtp_cap_sync_sel_by_name(clk_name);
	if ((sync_sel >= ADG_SYNC_SEL_AVB_COUNTER0) &&
	    (sync_sel <= ADG_SYNC_SEL_AVB_COUNTER7))
		clk_no = sync_sel - ADG_SYNC_SEL_AVB_COUNTER0;
	else
		return -1;

	val = calc_avb_counter(freq);
	if (!val)
		return -1;

	reg = AVB_CLK_DIV0 + (clk_no * 4);
	mch_adg_avb_write(priv, reg, val);
	calc_frq = (int)((32 * (u64)S0D1) / val);

	/* avb_count8[clk_no] enable */
	val = (1 << clk_no);
	val |= mch_adg_avb_read(priv, AVB_CLK_CONFIG);
	mch_adg_avb_write(priv, AVB_CLK_CONFIG, val);

	pr_debug("freq (%d),  calc freq (%d)\n", freq, val);

	return (int)calc_frq;
}

static int mch_unregist_adg(struct mch_private *priv)
{
	int i;

	/* clear AVB_SYNC_SEL0-2 */
	mch_adg_avb_write(priv, AVB_SYNC_SEL0, 0x00000000);
	mch_adg_avb_write(priv, AVB_SYNC_SEL1, 0x00000000);
	mch_adg_avb_write(priv, AVB_SYNC_SEL2, 0x00000000);

	mch_adg_avb_write(priv, AVB_SYNC_DIV1, 0x00000000);

	for (i = 0; i < 8; i++)
		mch_adg_avb_write(priv, AVB_CLK_DIV0 + (i * 4), 0x00000000);

	/* avb_count8 disable */
	mch_adg_avb_write(priv, AVB_CLK_CONFIG, 0x00000000);

	mch_adg_avb_write(priv, AVBCKR, 0x00000000);

	return 0;
}

static int mch_get_i2c_client(struct mch_private *priv)
{
	struct i2c_client *client;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "cirrus,cs2000-cp");
	/* must call put_device() when done with returned i2c_client device */
	client = of_find_i2c_device_by_node(np);
	if (!client)
		return -ENODEV;

	priv->client = client;

	return 0;
}

static int mch_check_cs2000_id(struct mch_private *priv)
{
	struct i2c_client *client = priv->client;
	u8 val;

	val = i2c_smbus_read_byte_data(client, 0x01);

	if (val != CS2000_ID)
		return -ENODEV;

	return 0;
}

static int mch_cs2000_read_all(struct mch_private *priv, u8 *buf)
{
	struct i2c_client *client = priv->client;
	int i;
	u8 reg[] = {
		CS2000_DEVICE_ID, CS2000_DEVICE_CTRL, CS2000_DEVICE_CFG1,
		CS2000_DEVICE_CFG2, CS2000_GLOBAL_CFG,
		CS2000_32BIT_RATIO0 + 0,
		CS2000_32BIT_RATIO0 + 1,
		CS2000_32BIT_RATIO0 + 2,
		CS2000_32BIT_RATIO0 + 3,
		CS2000_32BIT_RATIO1 + 0,
		CS2000_32BIT_RATIO1 + 1,
		CS2000_32BIT_RATIO1 + 2,
		CS2000_32BIT_RATIO1 + 3,
		CS2000_32BIT_RATIO2 + 0,
		CS2000_32BIT_RATIO2 + 1,
		CS2000_32BIT_RATIO2 + 2,
		CS2000_32BIT_RATIO2 + 3,
		CS2000_32BIT_RATIO3 + 0,
		CS2000_32BIT_RATIO3 + 1,
		CS2000_32BIT_RATIO3 + 2,
		CS2000_32BIT_RATIO3 + 3,
		CS2000_FUNCT_CFG1, CS2000_FUNCT_CFG2, CS2000_FUNCT_CFG3
	};

	if (!buf)
		return -1;

	for (i = 0 ; i < ARRAY_SIZE(reg); i++)
		buf[reg[i]] = i2c_smbus_read_byte_data(client, reg[i]);

	return 0;
}

static int mch_cs2000_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	if (reg > CS2000_FUNCT_CFG3)
		return -1;

	cs2000_data[reg] = val;
	i2c_smbus_write_byte_data(client, reg, val);

	return 0;
}

static int mch_cs2000_reset(struct mch_private *priv)
{
	struct i2c_client *client = priv->client;

	/* 3   Freeze         = 1   ; Freeze (registers 0x02-0x04) */
	mch_cs2000_write_byte(client, CS2000_GLOBAL_CFG,
			      (cs2000_data_bk[CS2000_GLOBAL_CFG] | 0x08));

	mch_cs2000_write_byte(client, CS2000_FUNCT_CFG1,
			      cs2000_data_bk[CS2000_FUNCT_CFG1]);

	mch_cs2000_write_byte(client, CS2000_FUNCT_CFG2,
			      cs2000_data_bk[CS2000_FUNCT_CFG2]);

	mch_cs2000_write_byte(client, CS2000_FUNCT_CFG3,
			      cs2000_data_bk[CS2000_FUNCT_CFG3]);

	mch_cs2000_write_byte(client, CS2000_DEVICE_CFG1,
			      cs2000_data_bk[CS2000_DEVICE_CFG1]);

	mch_cs2000_write_byte(client, CS2000_DEVICE_CFG2,
			      cs2000_data_bk[CS2000_DEVICE_CFG2]);

	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 0,
			      cs2000_data_bk[CS2000_32BIT_RATIO1 + 0]);
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 1,
			      cs2000_data_bk[CS2000_32BIT_RATIO1 + 1]);
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 2,
			      cs2000_data_bk[CS2000_32BIT_RATIO1 + 2]);
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 3,
			      cs2000_data_bk[CS2000_32BIT_RATIO1 + 3]);

	/* 3   Freeze         = 0   ; take effect Immediately */
	mch_cs2000_write_byte(client, CS2000_GLOBAL_CFG,
			      (cs2000_data_bk[CS2000_GLOBAL_CFG] & ~0x08));

	return 0;
}

static int mch_set_cs2000(struct mch_private *priv, u32 out_frq, u32 in_frq)
{
	struct i2c_client *client = priv->client;
	u64 ratio;
	u8 val;

	/* 3   Freeze         = 1   ; Freeze (registers 0x02-0x04) */
	/* 0   EnDevCfg2      = 1   ; Enabled */
	mch_cs2000_write_byte(client, CS2000_GLOBAL_CFG, 0x09);

	/* 7   ClkSkipEn      = 0   ; Disable */
	/* 6   AuxLockCfg     = 0   ; Push-Pull, Active High */
	/* 4:3 RefClkDiv[1:0] = 01  ; REF_CLK / 2 (16MHz to 26MHz) */
	mch_cs2000_write_byte(client, CS2000_FUNCT_CFG1, 0x08);

	/* 4   ClkOutUni      = 0   ; Clock outputs are driven 'low' when PLL is unlocked */
	/* 3   LFRatioCfg     = 1   ; 12.20 - High Accuracy */
	mch_cs2000_write_byte(client, CS2000_FUNCT_CFG2, 0x18);

	/* 2:0 ClkIn_BW[2:0]  = 000 ; 1Hz */
	mch_cs2000_write_byte(client, CS2000_FUNCT_CFG3, 0x00);

	/* 7:5 RModSel[2:0]   = 000 ; Left-shift R-value by 0 (x1) */
	/* 4:3 RSel[1:0]      = 01  ; Ratio 1 */
	/* 2:1 AuxOutSrc[1:0] = 11  ; PLL Lock Status Indicator */
	/* 0   EnDevCfg1      = 1   ; Enabled */
	mch_cs2000_write_byte(client, CS2000_DEVICE_CFG1, 0x0f);

	/* 2:1 LockClk[1:0]   = 01  ; Ratio 1 */
	/* 0   FracNSrc       = 1   ; Dynamic Ratio from Digital PLL for */
	/*                          ; Hybrid PLL Mode */
	mch_cs2000_write_byte(client, CS2000_DEVICE_CFG2, 0x03);

	/* 31:0 32-Bit Ratio  = (outpit-clock / input-clock ) * 2^20 */
	ratio = (u64)out_frq << 20;
	do_div(ratio, in_frq);

	val = ((u32)ratio >> 24) & 0xff;
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 0, val);
	val = ((u32)ratio >> 16) & 0xff;
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 1, val);
	val = ((u32)ratio >> 8) & 0xff;
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 2, val);
	val = ((u32)ratio >> 0) & 0xff;
	mch_cs2000_write_byte(client, CS2000_32BIT_RATIO1 + 3, val);

	/* 3   Freeze         = 0   ; take effect Immediately */
	/* 0   EnDevCfg2      = 1   ; Enabled */
	mch_cs2000_write_byte(client, CS2000_GLOBAL_CFG, 0x01);

	return 0;
}

static const struct of_device_id net_device_match_table[] = {
	{ .compatible = "renesas,etheravb-rcar-gen3",
	  .data = (void *)RCAR_GEN3 },
	{ }
};
MODULE_DEVICE_TABLE(of, net_device_match_table);

static int mch_get_net_device_handle(struct mch_private *priv)
{
	struct net_device *ndev = dev_get_by_name(&init_net, interface);
	struct ravb_private *ndev_priv;
	struct device *pdev_dev;
	const struct of_device_id *match;

	if (!ndev)
		return -ENODEV;

	pdev_dev = ndev->dev.parent;

	/* check supported devices */
	match = of_match_device(of_match_ptr(net_device_match_table), pdev_dev);

	if (!match) {
		pr_err("unsupport network interface\n");
		return -ENODEV;
	}

	priv->ndev = ndev;

	ndev_priv = netdev_priv(priv->ndev);
	priv->ptp = &ndev_priv->ptp;

	return 0;
}

int mch_regist_ravb(struct mch_private *priv, int freq,
		    int cap_cycle, int ch)
{
	struct net_device *ndev = priv->ndev;
	u32 val, tmp;
	int i;

	for (i = 0; i < 16; i++) {
		tmp = freq >> i;
		if ((tmp / 256) < cap_cycle) {
			val = (tmp / cap_cycle) - 1;
			break;
		}
	}

	if (i == 16)
		return -1;

	ravb_write(ndev, (((ch + 1) << 8) | val), GACP);

	pr_debug("GACP (0x%08x)\n", (((ch + 1) << 8) | val));

	return 0;
}

static int mch_unregist_ravb(struct mch_private *priv)
{
	struct net_device *ndev = priv->ndev;
	int i;

	for (i = 0; i < AVTP_CAP_DEVICES; i++) {
		/* GACP all ch clear */
		ravb_write(ndev, (((i + 1) << 8) | 0x00000000), GACP);

		/* Disable all GIS interrupt */
		ravb_write(ndev, BIT(17 + i), GID);
	}

	for (i = 0; i < MCH_PTP_TIMER_MAX; i++)
		ravb_write(ndev, BIT(3 + i), GID);

	return 0;
}

int mch_regist_interrupt(struct mch_private *priv,
			 const char *ch,
			 const char *name,
			 irqreturn_t (*func_i)(int, void *),
			 irqreturn_t (*func_t)(int, void *))
{
	struct net_device *ndev = priv->ndev;
	struct ravb_private *net_priv = netdev_priv(ndev);
	struct device *dev = priv->dev;
	const char  *irq_name;
	int irq;
	int err;

	if (WARN_ON(net_priv->chip_id != RCAR_GEN3))
		return -EPERM;

	/* regist interrupt of avtp capture */
	irq = platform_get_irq_byname(net_priv->pdev, ch);
	if (irq < 0) {
		pr_err("init: unsupport irq name (%s)\n", ch);
		return irq;
	}

	irq_name = devm_kasprintf(dev, GFP_KERNEL,
				  "%s:%s:%s",
				  ndev->name, ch, name);

	if (func_t)
		err = devm_request_threaded_irq(dev,
						irq,
						func_i,
						func_t,
						IRQF_SHARED,
						irq_name,
						priv);
	else
		err = devm_request_irq(dev,
				       irq,
				       func_i,
				       IRQF_SHARED,
				       irq_name,
				       priv);

	if (err) {
		pr_err("request_irq(%d,%s) error\n", irq, irq_name);
		return  err;
	}

	priv->irq = irq;

	return 0;
}

static void mch_release(struct device *dev)
{
	/* reserved */
}

/*
 * module init/cleanup
 */
static int mch_probe(struct platform_device *pdev)
{
	int err;
	struct mch_private *priv;
	struct resource *res;
	struct clk *clk;
	struct mch_param *param;
	int i;

	dev_info(&pdev->dev, "probe: start\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	err = -ENOMEM;
	priv = vzalloc(sizeof(*priv));
	if (unlikely(!priv))
		goto no_memory;

	mch_priv_ptr = priv;

	param = &priv->param;
	param->avtp_cap_cycle = avtp_cap_cycle;
	param->avtp_clk_frq = avtp_clk_frq;
	strncpy(param->avtp_clk_name, avtp_clk_name,
		sizeof(param->avtp_clk_name));
	param->sample_rate = sample_rate;

	err = mch_check_params(param);
	if (err < 0)
		goto out_release;

	priv->pdev = pdev;
	priv->dev = &pdev->dev;
	priv->dev->release = mch_release;
	dev_set_drvdata(&pdev->dev, priv);
	priv->timestamp_diff_init = NSEC / param->avtp_cap_cycle;

	err = mch_get_net_device_handle(priv);
	if (err < 0)
		goto out_release;

	for (i = 0; i < MCH_DEVID_MAX; i++)
		priv->m_dev[i] = NULL;

	/* MCH PTP Capture device initialize */
	err = mch_ptp_capture_init(priv);
	if (err < 0)
		goto out_release;

	/* MCH PTP Timer device initialize */
	err = mch_ptp_timer_init(priv);
	if (err < 0)
		goto out_release;

	priv->adg_avb_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->adg_avb_addr)) {
		err = PTR_ERR(priv->adg_avb_addr);
		goto out_release;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	err = -ENODEV;
	clk = devm_clk_get(&pdev->dev, "adg");
	if (IS_ERR(clk))
		goto out_release;

	err = clk_prepare_enable(clk);
	if (err < 0)
		goto out_release;

	priv->clk = clk;

	err = mch_get_i2c_client(priv);
	if (err < 0)
		goto out_release;

	err = mch_check_cs2000_id(priv);
	if (err < 0)
		goto out_release;

	err = mch_cs2000_read_all(priv, cs2000_data_bk);
	if (err < 0)
		goto out_release;

	memcpy(cs2000_data, cs2000_data_bk, sizeof(cs2000_data));

	/* register initialize for ADG */
	mch_regist_adg(priv);

	param->frq = mch_set_adg_avb_clk_div(priv,
					     param->avtp_clk_frq,
					     param->avtp_clk_name);
	pr_debug("%s Frequency %u\n", param->avtp_clk_name, param->frq);

	mch_set_adg_avb_sync_div(priv,
				 param->frq,
				 param->avtp_cap_cycle,
				 param->avtp_clk_name);

	mch_set_adg_avbckr(priv, param->avtp_clk_name);

	/* CS2000-cp initialize */
	mch_set_cs2000(priv, 24576000, param->frq);

	dev_info(&pdev->dev, "probe: success\n");

	return 0;

out_release:
	if (priv)
		vfree(priv);

no_memory:
	mch_priv_ptr = NULL;

	dev_info(&pdev->dev, "probe: failed\n");

	return err;
}

static int mch_remove(struct platform_device *pdev)
{
	struct mch_private *priv = mch_priv_ptr;

	dev_info(&pdev->dev, "remove: start\n");

	mch_cs2000_reset(priv);
	put_device(&priv->client->dev);

	mch_unregist_adg(priv);
	mch_unregist_ravb(priv);

	if (priv->clk)
		clk_disable_unprepare(priv->clk);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	/* In-Kernel API close */
	mch_close(priv->m_dev[0]);

	/* MCH PTP Timer cleanup */
	mch_ptp_timer_cleanup(priv);

	/* MCH PTP Capture cleanup */
	mch_ptp_capture_cleanup(priv);

	mch_priv_ptr = NULL;
	vfree(priv);

	dev_info(&pdev->dev, "remove: end\n");

	return 0;
}

static const struct of_device_id ravb_mch_of_match[] = {
	 { .compatible = "renesas,avb-mch-gen3" },
	 { },
};
MODULE_DEVICE_TABLE(of, ravb_mch_of_match);

static struct platform_driver ravb_mch_driver = {
	.probe = mch_probe,
	.remove = mch_remove,
	.driver = {
		.name = "ravb-mch",
		.of_match_table = ravb_mch_of_match,
	},
};
module_platform_driver(ravb_mch_driver);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Clock recovery Handler");
MODULE_LICENSE("Dual MIT/GPL");
