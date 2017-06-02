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

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME "/" fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/ptp_clock.h>
#include <../drivers/net/ethernet/renesas/ravb.h>
#include "mch_core.h"

#define NSEC 1000000000UL

extern struct mch_private *mch_priv_ptr;

static DEFINE_SPINLOCK(mch_ptp_lock);

static int enqueue(struct ptp_queue *que, struct ptp_clock_time *clock_time)
{
	if (q_next(que->tail, MAX_TIMESTAMPS) == que->head)
		que->head = q_next(que->head, MAX_TIMESTAMPS);

	que->timestamps[que->tail] = *clock_time;

	que->tail = q_next(que->tail, MAX_TIMESTAMPS);

	if (que->cnt < MAX_TIMESTAMPS)
		que->cnt++;

	return 0;
}

static int dequeue(struct ptp_queue *que, struct ptp_clock_time *clock_time)
{
	if (que->head == que->tail)
		return -1;

	*clock_time = que->timestamps[que->head];

	que->head = q_next(que->head, MAX_TIMESTAMPS);

	if (!que->cnt)
		que->cnt--;

	return 0;
}

static int check_queue(struct ptp_queue *que)
{
	int ret;

	if (que->tail > que->head)
		ret = que->tail - que->head;
	else if (que->tail < que->head)
		ret = (que->tail + MAX_TIMESTAMPS) - que->head;
	else /* if (que->head == que->tail) */
		ret = 0;

	return ret;
}

static int mch_ptp_timestamp_enqueue(struct mch_private *priv,
				     u64 timestamp, int ch)
{
	int i;
	struct ptp_clock_time cap_time;
	struct ptp_device *p_dev;
	struct ptp_queue *queue;
	unsigned long flags;

	cap_time.sec = timestamp / NSEC;
	cap_time.nsec = timestamp % NSEC;

	for (i = 0; i < ARRAY_SIZE(priv->p_dev); i++) {
		if (priv->p_dev[i]) {
			if (ch == priv->param.avtp_cap_ch) {
				p_dev = priv->p_dev[i];
				/* add to queue */
				queue = &p_dev->que;
				spin_lock_irqsave(&p_dev->qlock, flags);
				enqueue(queue, &cap_time);
				spin_unlock_irqrestore(&p_dev->qlock, flags);
			}
		}
	}

	return 0;
}

static void mch_ptp_correct_timestamp(struct mch_private *priv,
				      int ch,
				      u64 ptp_timestamp_u32,
				      u64 ptp_timestamp_l32,
				      u64 ptp_timestamp_l)
{
	u32 timestamp;
	u64 timestamp_tmp;
	u64 cap_timestamp;

	timestamp = priv->timestamp[ch];

	switch (priv->avtp_cap_status[ch]) {
	case AVTP_CAP_STATE_INIT:
		cap_timestamp = 0;
		priv->avtp_cap_status[ch] = AVTP_CAP_STATE_UNLOCK;
		break;

	case AVTP_CAP_STATE_UNLOCK:
	case AVTP_CAP_STATE_LOCK:
		timestamp_tmp = (u64)timestamp;
		if (priv->pre_timestamp[ch] > timestamp)
			timestamp_tmp += U32_MAX;

		if ((priv->pre_timestamp[ch] <= priv->pre_ptp_timestamp_l32) &&
		    (timestamp_tmp <= ptp_timestamp_l)) {
			priv->avtp_cap_status[ch] = AVTP_CAP_STATE_LOCK;
			/* ptp precedes avtp_cap */
			if (ptp_timestamp_l32 >= timestamp) {
				cap_timestamp = ptp_timestamp_u32 | timestamp;
			} else {
				cap_timestamp = (ptp_timestamp_u32 - (u64)BIT_ULL(32)) | timestamp;
				/* TODO work around */
				if (cap_timestamp + NSEC < priv->pre_cap_timestamp[ch])
					cap_timestamp = ptp_timestamp_u32 | timestamp;
			}

		} else if ((priv->pre_timestamp[ch] > priv->pre_ptp_timestamp_l32) &&
			   (timestamp_tmp > ptp_timestamp_l)) {
			priv->avtp_cap_status[ch] = AVTP_CAP_STATE_LOCK;
			/* avtp_cap precedes ptp */
			if (ptp_timestamp_l32 <= timestamp) {
				cap_timestamp = ptp_timestamp_u32 | timestamp;
			} else {
				cap_timestamp = (ptp_timestamp_u32 + (u64)BIT_ULL(32)) | timestamp;
				/* TODO work around */
				if (cap_timestamp - NSEC > priv->pre_cap_timestamp[ch])
					cap_timestamp = ptp_timestamp_u32 | timestamp;
			}
		} else {
			/* TODO : timestamp error */
			cap_timestamp = 0;
			priv->avtp_cap_status[ch] = AVTP_CAP_STATE_UNLOCK;
			pr_info("ptp timestamp unlock\n");
			pr_debug("%d\t %u\t %llu\t %llu\t %llu\t%llu\t %u\n",
				 ch,
				 priv->pre_timestamp[ch],
				 priv->pre_ptp_timestamp_l32,
				 timestamp_tmp,
				 ptp_timestamp_l,
				 ptp_timestamp_l32,
				 timestamp);
		}
		break;

	default:
		cap_timestamp = 0;
		priv->avtp_cap_status[ch] = AVTP_CAP_STATE_INIT;
	}

	/* check enqueue */
	if (priv->avtp_cap_status[ch] != AVTP_CAP_STATE_INIT) {
		if (cap_timestamp < priv->pre_cap_timestamp[ch]) {
			priv->avtp_cap_status[ch] = AVTP_CAP_STATE_UNLOCK;
			pr_info("capture timestamp unlock  %llu  %llu\n", cap_timestamp,
				priv->pre_cap_timestamp[ch]);
			/* TODO: */
			priv->pre_cap_timestamp[ch] += priv->timestamp_diff[ch];
			mch_ptp_timestamp_enqueue(priv,
						  priv->pre_cap_timestamp[ch], ch);
		}
	}

	if (priv->avtp_cap_status[ch] == AVTP_CAP_STATE_LOCK) {
		if (priv->pre_cap_timestamp[ch]) {
			while (priv->pre_cap_timestamp[ch] + priv->timestamp_diff[ch] + 2000000 < cap_timestamp) {
				pr_debug("%llu interrupt fail %llu %llu\n", cap_timestamp,
					 priv->pre_cap_timestamp[ch], priv->timestamp_diff[ch]);
				/* timestamp jump, interpolation */
				priv->pre_cap_timestamp[ch] += priv->timestamp_diff[ch];
				mch_ptp_timestamp_enqueue(priv,
							  priv->pre_cap_timestamp[ch], ch);
			}
			if ((cap_timestamp - priv->pre_cap_timestamp[ch] > priv->timestamp_diff_init - 200000) &&
			    (cap_timestamp - priv->pre_cap_timestamp[ch] < priv->timestamp_diff_init + 200000))
				priv->timestamp_diff[ch] = cap_timestamp - priv->pre_cap_timestamp[ch];
		}

		mch_ptp_timestamp_enqueue(priv, cap_timestamp, ch);
		priv->pre_cap_timestamp[ch] = cap_timestamp;
	} else {
		priv->timestamp_diff[ch] = priv->timestamp_diff_init;
		priv->pre_cap_timestamp[ch] = 0;
	}

	priv->pre_timestamp[ch] = timestamp;
}

irqreturn_t mch_ptp_timestamp_interrupt(int irq, void *dev_id)
{
	struct mch_private *priv = dev_id;
	int i;
	u64 ptp_timestamp;
	u64 ptp_timestamp_u32, ptp_timestamp_l32;
	u64 ptp_timestamp_l;

	mch_ptp_get_time(&ptp_timestamp);
	ptp_timestamp_u32 = ptp_timestamp & 0xffffffff00000000;
	ptp_timestamp_l32 = ptp_timestamp & 0x00000000ffffffff;
	ptp_timestamp_l   = ptp_timestamp_l32;

	if (priv->pre_ptp_timestamp_l32 > ptp_timestamp_l)
		ptp_timestamp_l += U32_MAX;

	for (i = 0; i < AVTP_CAP_DEVICES ; i++)
		if (test_and_clear_bit(i, priv->timestamp_irqf))
			mch_ptp_correct_timestamp(
				priv, i,
				ptp_timestamp_u32, ptp_timestamp_l32,
				ptp_timestamp_l);

	priv->pre_ptp_timestamp_u32 = ptp_timestamp_u32;
	priv->pre_ptp_timestamp_l32 = ptp_timestamp_l32;

	return IRQ_HANDLED;
}

static int init_ptp_device(struct ptp_device *p_dev)
{
	struct mch_private *priv = p_dev->priv;
	struct net_device *ndev = priv->ndev;
	int ch;

	pr_debug("[%s] START\n", __func__);

	p_dev->que.head = 0;
	p_dev->que.tail = 0;
	p_dev->que.cnt = 0;

	spin_lock_init(&p_dev->qlock);

	ch = priv->param.avtp_cap_ch;

	priv->avtp_cap_status[ch] = AVTP_CAP_STATE_INIT;
	priv->pre_cap_timestamp[ch] = 0;
	priv->timestamp_diff_init = NSEC / priv->param.avtp_cap_cycle;
	priv->timestamp_diff[ch] = priv->timestamp_diff_init;

	priv->interrupt_enable_cnt[ch]++;
	if (priv->interrupt_enable_cnt[ch])
		ravb_write(ndev, BIT(17 + ch), GIE);

	pr_debug("[%s] END\n", __func__);

	return 0;
}

/*
 * public functions
 */

/*
 * In-Kernel PTP API
 */
int mch_ptp_get_time(u64 *ns)
{
	struct mch_private *priv = mch_priv_ptr;
	struct ravb_private *ndev_priv;
	struct timespec64 ts;
	struct ptp_clock_info *ptp;

	if (!priv)
		return -ENODEV;

	ndev_priv = netdev_priv(priv->ndev);
	ptp = &ndev_priv->ptp.info;

	if (!ptp->gettime64)
		return -ENODEV;

	ptp->gettime64(ptp, &ts);
	*ns = (u64)timespec64_to_ns(&ts);

	return 0;
}
EXPORT_SYMBOL(mch_ptp_get_time);

/*
 * In-Kernel PTP Capture API
 */
int mch_ptp_open(int *dev_id)
{
	struct mch_private *priv = mch_priv_ptr;
	int i;
	int ret;
	struct ptp_device *p_dev = NULL;
	unsigned long flags;

	*dev_id = -1;

	if (!dev_id)
		return -EINVAL;

	if (!priv)
		return -ENODEV;

	p_dev = kzalloc(sizeof(*p_dev), GFP_KERNEL);
	if (!p_dev)
		return -ENOMEM;

	spin_lock_irqsave(&mch_ptp_lock, flags);

	for (i = 0; i < ARRAY_SIZE(priv->p_dev) && priv->p_dev[i]; i++)
		;

	if (i >= ARRAY_SIZE(priv->p_dev)) {
		pr_err("cannot register mch_ptp device\n");
		spin_unlock_irqrestore(&mch_ptp_lock, flags);
		kfree(p_dev);
		return -EBUSY;
	}

	p_dev->dev_id = i;
	p_dev->priv = priv;
	priv->p_dev[i] = p_dev;

	spin_unlock_irqrestore(&mch_ptp_lock, flags);

	ret = init_ptp_device(p_dev);

	pr_info("registered mch_ptp device index=%d\n", i);
	*dev_id = i;

	return 0;
}
EXPORT_SYMBOL(mch_ptp_open);

int mch_ptp_close(int dev_id)
{
	struct mch_private *priv = mch_priv_ptr;
	struct net_device *ndev = priv->ndev;
	struct ptp_device *p_dev;
	int ch;
	unsigned long flags;

	if (!priv)
		return -ENODEV;

	if ((dev_id < 0) || (dev_id >= PTP_DEVID_MAX))
		return -EINVAL;

	spin_lock_irqsave(&mch_ptp_lock, flags);

	p_dev = priv->p_dev[dev_id];
	if (!p_dev)
		return 0;
	priv->p_dev[dev_id] = NULL;

	spin_unlock_irqrestore(&mch_ptp_lock, flags);

	ch = priv->param.avtp_cap_ch;
	priv->interrupt_enable_cnt[ch]--;
	if (!priv->interrupt_enable_cnt[ch])
		ravb_write(ndev, BIT(17 + ch), GID);

	kfree(p_dev);

	pr_info("close mch_ptp device  index=%d\n", dev_id);

	return 0;
}
EXPORT_SYMBOL(mch_ptp_close);

int mch_ptp_get_timestamps(int dev_id,
			   int ch,
			   int *count,
			   struct ptp_clock_time timestamps[])
{
	struct mch_private *priv = mch_priv_ptr;
	struct ptp_device *p_dev;
	struct ptp_queue *queue;
	struct ptp_clock_time clock_time;
	int i;
	unsigned long flags;

	pr_debug("[%s] START\n", __func__);

	if (!priv)
		return -ENODEV;

	if ((dev_id < 0) || (dev_id >= PTP_DEVID_MAX))
		return -EINVAL;

	p_dev = priv->p_dev[dev_id];
	if (!p_dev)
		return -EINVAL;

	spin_lock_irqsave(&p_dev->qlock, flags);

	queue = &p_dev->que;

	*count = check_queue(queue);
	if (*count == 0) {
		spin_unlock_irqrestore(&p_dev->qlock, flags);
		return 0;
	}

	for (i = 0; i < *count; i++) {
		dequeue(queue, &clock_time);
		timestamps[i] = clock_time;
	}

	spin_unlock_irqrestore(&p_dev->qlock, flags);

	pr_debug("[%s] END\n", __func__);

	return 0;
}
EXPORT_SYMBOL(mch_ptp_get_timestamps);
