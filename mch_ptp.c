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
#include <linux/list.h>
#include <../drivers/net/ethernet/renesas/ravb.h>

#include "mch_core.h"

extern struct mch_private *mch_priv_ptr;

static DEFINE_SPINLOCK(mch_ptp_cap_lock);

/*
 * PTP Capture
 */
static int enqueue(struct ptp_queue *que, u64 ts)
{
	if (q_next(que->tail, que->max_cnt) == que->head)
		que->head = q_next(que->head, que->max_cnt);

	que->timestamps[que->tail] = ts;

	que->tail = q_next(que->tail, que->max_cnt);

	return 0;
}

static int dequeue(struct ptp_queue *que, u64 *ts)
{
	if (que->head == que->tail)
		return -1;

	*ts = que->timestamps[que->head];

	que->head = q_next(que->head, que->max_cnt);

	return 0;
}

static int mch_ptp_timestamp_enqueue(struct mch_private *priv,
				     u64 timestamp, int ch)
{
	struct ptp_device *p_dev;
	struct ptp_capture_device *cap;
	unsigned long flags;

	cap = &priv->cap_dev[ch];

	list_for_each_entry(p_dev, &cap->active, list) {
		/* add to queue */
		spin_lock_irqsave(&p_dev->qlock, flags);
		enqueue(&p_dev->que, timestamp);
		spin_unlock_irqrestore(&p_dev->qlock, flags);
	}

	return 0;
}

static u64 mch_ptp_interpolate_timestamps(struct mch_private *priv,
					  int ch,
					  u64 base,
					  u64 target,
					  u64 diff)
{
	u64 timestamp;
	int loop_cnt = 0;

	timestamp = base;

	while (timestamp + diff + 2000000 < target) {
		pr_debug("interpolate: %llu %llu %llu\n",
			 target, timestamp, diff);

		timestamp += diff;
		mch_ptp_timestamp_enqueue(priv, timestamp, ch);

		loop_cnt++;
		if (loop_cnt > 15)
			break;
	}

	return timestamp;
}

static void mch_ptp_correct_timestamp(struct mch_private *priv,
				      int ch,
				      u64 ptp_timestamp_u32,
				      u64 ptp_timestamp_l32,
				      u64 ptp_timestamp_l)
{
	struct ptp_capture_device *cap;
	u32 timestamp;
	u64 timestamp_tmp;
	u64 timestamp_cap;
	u64 timestamp_cap_pre;
	u64 timestamp_diff;

	cap = &priv->cap_dev[ch];

	timestamp = cap->timestamp;
	timestamp_cap_pre = cap->timestamp_cap_pre;

	switch (cap->status) {
	case AVTP_CAP_STATE_INIT:
		timestamp_cap = 0;
		cap->status = AVTP_CAP_STATE_UNLOCK;
		break;

	case AVTP_CAP_STATE_UNLOCK:
	case AVTP_CAP_STATE_LOCK:
		timestamp_tmp = (u64)timestamp;
		if (cap->timestamp_pre > timestamp)
			timestamp_tmp += U32_MAX;

		if ((cap->timestamp_pre <= priv->pre_ptp_timestamp_l32) &&
		    (timestamp_tmp <= ptp_timestamp_l)) {
			cap->status = AVTP_CAP_STATE_LOCK;
			/* ptp precedes avtp_cap */
			if (ptp_timestamp_l32 >= timestamp) {
				timestamp_cap = ptp_timestamp_u32 | timestamp;
			} else {
				timestamp_cap = (ptp_timestamp_u32 - (u64)BIT_ULL(32)) | timestamp;
				/* TODO work around */
				if (timestamp_cap + NSEC < timestamp_cap_pre)
					timestamp_cap = ptp_timestamp_u32 | timestamp;
			}

		} else if ((cap->timestamp_pre > priv->pre_ptp_timestamp_l32) &&
			   (timestamp_tmp > ptp_timestamp_l)) {
			cap->status = AVTP_CAP_STATE_LOCK;
			/* avtp_cap precedes ptp */
			if (ptp_timestamp_l32 <= timestamp) {
				timestamp_cap = ptp_timestamp_u32 | timestamp;
			} else {
				timestamp_cap = (ptp_timestamp_u32 + (u64)BIT_ULL(32)) | timestamp;
				/* TODO work around */
				if (timestamp_cap - NSEC > timestamp_cap_pre)
					timestamp_cap = ptp_timestamp_u32 | timestamp;
			}
		} else {
			timestamp_cap = 0;
			cap->status = AVTP_CAP_STATE_UNLOCK;
			pr_debug("ptp timestamp unlock\n");
			pr_debug("%d\t %u\t %llu\t %llu\t %llu\t%llu\t %u\n",
				 ch,
				 cap->timestamp_pre,
				 priv->pre_ptp_timestamp_l32,
				 timestamp_tmp,
				 ptp_timestamp_l,
				 ptp_timestamp_l32,
				 timestamp);
		}
		break;

	default:
		timestamp_cap = 0;
		cap->status = AVTP_CAP_STATE_INIT;
		break;
	}

	/* check unlock */
	if (cap->status != AVTP_CAP_STATE_INIT) {
		if (timestamp_cap < timestamp_cap_pre) {
			cap->status = AVTP_CAP_STATE_UNLOCK;
			pr_debug("capture timestamp unlock  %llu  %llu\n",
				 timestamp_cap, timestamp_cap_pre);
		}
	}

	/* enqueue */
	if (cap->status == AVTP_CAP_STATE_LOCK) {
		if (timestamp_cap_pre) {
			timestamp_diff = cap->timestamp_diff;
			/* if occurred timestamp jump, interpolate timestamps */
			timestamp_cap_pre = mch_ptp_interpolate_timestamps(
					priv,
					ch,
					timestamp_cap_pre,
					timestamp_cap,
					timestamp_diff);

			timestamp_diff = timestamp_cap - timestamp_cap_pre;
			if ((timestamp_diff > priv->timestamp_diff_init - 200000) &&
			    (timestamp_diff < priv->timestamp_diff_init + 200000))
				cap->timestamp_diff = timestamp_diff;
		}

		mch_ptp_timestamp_enqueue(priv, timestamp_cap, ch);
	} else {
		cap->timestamp_diff = priv->timestamp_diff_init;
	}

	cap->timestamp_pre = timestamp;
	cap->timestamp_cap_pre = timestamp_cap;
}

static void mch_ptp_capture_attach(struct mch_private *priv, int ch)
{
	struct net_device *ndev = priv->ndev;
	struct ptp_capture_device *cap = &priv->cap_dev[ch];

	if (ch < 0)
		return;

	if (atomic_inc_return(&cap->attached) == 1) {
		cap->status = AVTP_CAP_STATE_INIT;
		cap->timestamp_cap_pre = 0;
		cap->timestamp_diff = priv->timestamp_diff_init;

		mch_set_adg_avb_sync_sel(priv, ch, priv->param.avtp_clk_name);

		mch_regist_ravb(priv, priv->param.frq,
				priv->param.avtp_cap_cycle, ch);

		ravb_write(ndev, BIT(17 + ch), GIE);
	}
}

static void mch_ptp_capture_detach(struct mch_private *priv, int ch)
{
	struct net_device *ndev = priv->ndev;
	struct ptp_capture_device *cap = &priv->cap_dev[ch];

	if (ch < 0)
		return;

	if (!atomic_dec_if_positive(&cap->attached))
		ravb_write(ndev, BIT(17 + ch), GID);
}

static irqreturn_t mch_ptp_timestamp_interrupt(int irq, void *data)
{
	struct mch_private *priv = data;
	struct net_device *ndev = priv->ndev;
	irqreturn_t ret = IRQ_NONE;
	u32 gis = ravb_read(ndev, GIS);
	u32 timestamp;
	int i;

	gis &= ravb_read(ndev, GIC);
	for (i = 0; i < AVTP_CAP_DEVICES; i++) {
		if (gis & BIT(17 + i)) {
			timestamp = ravb_read(ndev, GCAT0 + (4 * (i + 1)));
			priv->cap_dev[i].timestamp = timestamp;
			set_bit(i, priv->timestamp_irqf);
			ravb_write(ndev, ~BIT(17 + i), GIS);
			ret = IRQ_WAKE_THREAD;
		}
	}

	return ret;
}

static irqreturn_t mch_ptp_timestamp_interrupt_th(int irq, void *dev_id)
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
void *mch_ptp_open(void)
{
	struct mch_private *priv = mch_priv_ptr;
	struct ptp_device *p_dev = NULL;
	unsigned long flags;

	if (!priv)
		return NULL;

	p_dev = kzalloc(sizeof(*p_dev), GFP_KERNEL);
	if (!p_dev)
		return NULL;

	p_dev->priv = priv;
	p_dev->que.head = 0;
	p_dev->que.tail = 0;
	p_dev->que.max_cnt = -1;
	p_dev->que.timestamps = NULL;
	p_dev->capture_ch = AVTP_CAP_CH_INVALID;
	spin_lock_init(&p_dev->qlock);

	spin_lock_irqsave(&mch_ptp_cap_lock, flags);
	list_add_tail(&p_dev->list, &priv->ptp_capture_inactive);
	spin_unlock_irqrestore(&mch_ptp_cap_lock, flags);

	pr_debug("registered mch_ptp device index=%p\n", p_dev);

	return (void *)p_dev;
}
EXPORT_SYMBOL(mch_ptp_open);

int mch_ptp_close(void *ptp_handle)
{
	struct mch_private *priv;
	struct net_device *ndev;
	struct ptp_device *p_dev = (struct ptp_device *)ptp_handle;
	unsigned long flags;

	if (!ptp_handle)
		return -EINVAL;

	priv = p_dev->priv;
	if (!priv)
		return -ENODEV;

	ndev = priv->ndev;

	/* move from active or inactive list */
	spin_lock_irqsave(&mch_ptp_cap_lock, flags);
	list_del(&p_dev->list);
	spin_unlock_irqrestore(&mch_ptp_cap_lock, flags);

	/* capture stop */
	mch_ptp_capture_detach(priv, p_dev->capture_ch);

	/* free timestamp buffer */
	kfree(p_dev->que.timestamps);

	/* free ptp device */
	kfree(p_dev);

	pr_debug("close mch_ptp device  index=0x%p\n", p_dev);

	return 0;
}
EXPORT_SYMBOL(mch_ptp_close);

int mch_ptp_capture_start(void *ptp_handle,
			  int ch,
			  int max_count)
{
	struct ptp_device *p_dev = (struct ptp_device *)ptp_handle;
	struct mch_private *priv;
	struct net_device *ndev;
	struct ptp_capture_device *cap;
	unsigned long flags;
	int real_ch;
	void *timestamps;

	if (!ptp_handle)
		return -EINVAL;

	priv = p_dev->priv;
	if (!priv)
		return -ENODEV;

	ndev = priv->ndev;

	if ((ch < AVTP_CAP_CH_MIN) || (ch > AVTP_CAP_CH_MAX))
		return -EINVAL;

	if (p_dev->que.timestamps)
		return -EBUSY;

	timestamps = kcalloc(max_count + 1, sizeof(u64), GFP_KERNEL);
	if (!timestamps)
		return -ENOMEM;

	real_ch = ch - AVTP_CAP_CH_MIN;

	spin_lock_irqsave(&p_dev->qlock, flags);

	p_dev->que.timestamps = timestamps;
	p_dev->que.max_cnt = max_count + 1;
	p_dev->capture_ch = real_ch;

	spin_unlock_irqrestore(&p_dev->qlock, flags);

	spin_lock_irqsave(&mch_ptp_cap_lock, flags);

	/* move from inactive list to active list */
	cap = &priv->cap_dev[real_ch];
	list_move_tail(&p_dev->list, &cap->active);

	/* capture start */
	mch_ptp_capture_attach(priv, real_ch);

	spin_unlock_irqrestore(&mch_ptp_cap_lock, flags);

	return 0;
}
EXPORT_SYMBOL(mch_ptp_capture_start);

int mch_ptp_capture_stop(void *ptp_handle)
{
	struct ptp_device *p_dev = (struct ptp_device *)ptp_handle;
	struct mch_private *priv;
	unsigned long flags;
	void *timestamps;

	if (!ptp_handle)
		return -EINVAL;

	priv = p_dev->priv;
	if (!priv)
		return -ENODEV;

	if (!p_dev->que.timestamps)
		return -EPERM;

	/* move from active list to inactive list */
	spin_lock_irqsave(&mch_ptp_cap_lock, flags);
	list_move_tail(&p_dev->list, &priv->ptp_capture_inactive);
	spin_unlock_irqrestore(&mch_ptp_cap_lock, flags);

	/* capture stop */
	mch_ptp_capture_detach(priv, p_dev->capture_ch);

	/* clear ptp device */
	spin_lock_irqsave(&p_dev->qlock, flags);

	timestamps = p_dev->que.timestamps;
	p_dev->que.timestamps = NULL;
	p_dev->que.max_cnt = -1;
	p_dev->capture_ch = AVTP_CAP_CH_INVALID;

	spin_unlock_irqrestore(&p_dev->qlock, flags);

	/* free timestamp buffer */
	kfree(timestamps);

	return 0;
}
EXPORT_SYMBOL(mch_ptp_capture_stop);

int mch_ptp_get_timestamps(void *ptp_handle,
			   int req_count,
			   u64 *timestamps)
{
	struct mch_private *priv = mch_priv_ptr;
	struct ptp_device *p_dev = (struct ptp_device *)ptp_handle;
	int i;
	unsigned long flags;

	pr_debug("[%s] START\n", __func__);

	if (!priv)
		return -ENODEV;

	if (!p_dev)
		return -EINVAL;

	if (p_dev->capture_ch == AVTP_CAP_CH_INVALID)
		return -EPERM;

	spin_lock_irqsave(&p_dev->qlock, flags);

	for (i = 0; i < req_count; i++)
		if (dequeue(&p_dev->que, &timestamps[i]) < 0)
			break;

	spin_unlock_irqrestore(&p_dev->qlock, flags);

	pr_debug("[%s] END\n", __func__);

	return i;
}
EXPORT_SYMBOL(mch_ptp_get_timestamps);

int mch_ptp_capture_init(struct mch_private *priv)
{
	struct ptp_capture_device *cap;
	int err, i;

	err = mch_regist_interrupt(priv,
				   "ch22",
				   "multi_A",
				   mch_ptp_timestamp_interrupt,
				   mch_ptp_timestamp_interrupt_th);
	if (err < 0)
		return err;

	for (i = 0; i < AVTP_CAP_DEVICES; i++) {
		cap = &priv->cap_dev[i];
		INIT_LIST_HEAD(&cap->active);
	}
	INIT_LIST_HEAD(&priv->ptp_capture_inactive);

	return 0;
}

int mch_ptp_capture_cleanup(struct mch_private *priv)
{
	struct ptp_device *pd, *tmp;
	struct ptp_capture_device *cap;
	int i;

	for (i = 0; i < AVTP_CAP_DEVICES; i++) {
		cap = &priv->cap_dev[i];
		list_for_each_entry_safe(pd, tmp, &cap->active, list)
			mch_ptp_close(pd);
	}

	list_for_each_entry_safe(pd, tmp, &priv->ptp_capture_inactive, list)
		mch_ptp_close(pd);

	return 0;
}
