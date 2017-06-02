/*************************************************************************/ /*
 avb-mch

 Copyright (C) 2017 Renesas Electronics Corporation

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

#ifndef __MCH_CORE_H__
#define __MCH_CORE_H__

#include "ravb_mch.h"

#define MCH_DEVID_MAX     1
#define MCH_PTP_TIMER_MAX 7
#define ADG_AVB_OFFSET   0x0100
#define AVTP_CAP_DEVICES 12

#define GIS_PTMF1 0x00000008
#define GCCR_LI_1 0x00100000

#define q_next(n, max) (((n) + 1) % (max))

#define AVTP_CAP_CH_MIN     2
#define AVTP_CAP_CH_MAX     13
#define AVTP_CAP_CH_INVALID -1

#define PTP_TIMER_CH_MIN  1
#define PTP_TIMER_CH_MAX  7

#define NSEC 1000000000UL

enum ADG_AVB_REG {
	AVBCKR          = 0x0100 - ADG_AVB_OFFSET,
	AVB_SYNC_SEL0   = 0x0104 - ADG_AVB_OFFSET,
	AVB_SYNC_SEL1   = 0x010c - ADG_AVB_OFFSET,
	AVB_SYNC_SEL2   = 0x0110 - ADG_AVB_OFFSET,
	AVB_SYNC_DIV0   = 0x0114 - ADG_AVB_OFFSET,
	AVB_SYNC_DIV1   = 0x0118 - ADG_AVB_OFFSET,
	AVB_CLK_DIV0    = 0x011c - ADG_AVB_OFFSET,
	AVB_CLK_DIV1    = 0x0120 - ADG_AVB_OFFSET,
	AVB_CLK_DIV2    = 0x0124 - ADG_AVB_OFFSET,
	AVB_CLK_DIV3    = 0x0128 - ADG_AVB_OFFSET,
	AVB_CLK_DIV4    = 0x012c - ADG_AVB_OFFSET,
	AVB_CLK_DIV5    = 0x0130 - ADG_AVB_OFFSET,
	AVB_CLK_DIV6    = 0x0134 - ADG_AVB_OFFSET,
	AVB_CLK_DIV7    = 0x0138 - ADG_AVB_OFFSET,
	AVB_CLK_CONFIG  = 0x013c - ADG_AVB_OFFSET,
};

enum AVTP_CAP_STATE {
	AVTP_CAP_STATE_INIT = 0,
	AVTP_CAP_STATE_UNLOCK,
	AVTP_CAP_STATE_LOCK,
};

struct mch_param {
	int avtp_cap_ch;
	int avtp_cap_cycle;
	char avtp_clk_name[32];
	int avtp_clk_frq;
	u32 frq;
	int sample_rate;
};

struct mch_timestamps {
	struct list_head list;
	int count;
	struct mch_timestamp *ts;
};

struct mch_device {
	struct mch_private	*priv;

	spinlock_t		qlock; /* for timestamp queue */

	u32			pending_events;
	wait_queue_head_t	wait_event;
	struct task_struct	*task;
	int			correct;
	u32			interval;
	struct list_head	timestamps;
};

struct ptp_queue {
	int head;
	int tail;
	int max_cnt;
	u64 *timestamps;
};

struct ptp_device {
	struct mch_private	*priv;

	int			capture_ch;
	struct ptp_queue	que;
	spinlock_t		qlock; /* for timestamp queue */
	struct list_head	list;
};

struct ptp_timer_device {
	struct mch_private      *priv;

	int			status; /* 0:inactive, 1:active */
	int			ch;
	u32			time;
	void			*arg;
	u32			(*func)(void *);
};

struct ptp_capture_device {
	enum AVTP_CAP_STATE status;
	u32 timestamp;
	u32 timestamp_pre;
	u64 timestamp_cap_pre;
	u64 timestamp_diff;
	atomic_t attached;
	struct list_head active;
};

struct mch_private {
	/* MCH Control */
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *adg_avb_addr;
	struct i2c_client *client;
	struct clk *clk;
	struct net_device *ndev;
	int irq;

	struct mch_device *m_dev[MCH_DEVID_MAX];

	DECLARE_BITMAP(timer_map, MCH_PTP_TIMER_MAX);
	struct ptp_timer_device *tim_dev[MCH_PTP_TIMER_MAX];

	DECLARE_BITMAP(timestamp_irqf, AVTP_CAP_DEVICES);
	struct ptp_capture_device cap_dev[AVTP_CAP_DEVICES];
	struct list_head ptp_capture_inactive;
	u64 timestamp_diff_init;
	u64 pre_ptp_timestamp_u32;
	u64 pre_ptp_timestamp_l32;

	struct mch_param param;

	struct ravb_ptp *ptp;
};

/* RAVB */
int mch_regist_interrupt(struct mch_private *priv,
			 const char *ch, const char *name,
			 irqreturn_t (*func_i)(int, void *),
			 irqreturn_t (*func_t)(int, void *));
int mch_regist_ravb(struct mch_private *priv, int freq,
		    int cap_cycle, int ch);

/* PTP Capture */
int mch_ptp_capture_init(struct mch_private *priv);
int mch_ptp_capture_cleanup(struct mch_private *priv);

/* PTP Timer */
int mch_ptp_timer_init(struct mch_private *priv);
int mch_ptp_timer_cleanup(struct mch_private *priv);

/* ADG */
int mch_set_adg_avb_sync_sel(struct mch_private *priv, int ch,
			     char *clk_name);
static inline u32 mch_adg_avb_read(struct mch_private *priv,
				   enum ADG_AVB_REG reg)
{
	return ioread32(priv->adg_avb_addr + reg);
}

static inline void mch_adg_avb_write(struct mch_private *priv,
				     enum ADG_AVB_REG reg, u32 data)
{
	iowrite32(data, priv->adg_avb_addr + reg);
}

#endif /* __MCH_CORE_H__ */
