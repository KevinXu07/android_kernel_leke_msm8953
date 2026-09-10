/* SPDX-License-Identifier: GPL-2.0-only */
/* L05 external audio enables, recovered from the original board kernel.
 * Kept separate from the generic MBHC detection algorithm. GPIO38 is the
 * physical socket contact: high selects codec, low selects headphone gate.
 */
#ifndef WCD_MBHC_L05_GATES_H
#define WCD_MBHC_L05_GATES_H

#include <linux/interrupt.h>
#include <linux/workqueue.h>

struct l05_audio_gates {
	struct device *dev;
	struct mutex lock;
	struct delayed_work work;
	int detect;
	int codec;
	int headphone;
	int irq;
};

static void l05_audio_gates_select(struct work_struct *work)
{
	struct l05_audio_gates *g = container_of(work,
		struct l05_audio_gates, work.work);
	int absent;

	mutex_lock(&g->lock);
	absent = gpio_get_value_cansleep(g->detect);
	if (absent) {
		gpio_set_value_cansleep(g->headphone, 0);
		/* Allow the external path to settle; never busy-wait in IRQ. */
		msleep(500);
		/* A new insertion during settling must not enable the wrong path. */
		absent = gpio_get_value_cansleep(g->detect);
		gpio_set_value_cansleep(g->codec, !!absent);
		gpio_set_value_cansleep(g->headphone, !absent);
	} else {
		gpio_set_value_cansleep(g->codec, 0);
		gpio_set_value_cansleep(g->headphone, 1);
	}
	dev_info(g->dev, "L05 audio gates: socket=%d codec=%d headphone=%d\n",
		absent, gpio_get_value_cansleep(g->codec),
		gpio_get_value_cansleep(g->headphone));
	mutex_unlock(&g->lock);
}

static irqreturn_t l05_audio_gates_irq(int irq, void *data)
{
	struct l05_audio_gates *g = data;

	mod_delayed_work(system_wq, &g->work, msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static void l05_audio_gates_event(struct l05_audio_gates *g,
				enum wcd_notify_event event)
{
	if (!g)
		return;
	if (event != WCD_EVENT_PRE_HPHL_PA_ON &&
	    event != WCD_EVENT_PRE_HPHR_PA_ON &&
	    event != WCD_EVENT_POST_HPHL_PA_OFF)
		return;
	mutex_lock(&g->lock);
	if (event == WCD_EVENT_POST_HPHL_PA_OFF)
		gpio_set_value_cansleep(g->codec, 0);
	else if (gpio_get_value_cansleep(g->detect))
		gpio_set_value_cansleep(g->codec, 1);
	mutex_unlock(&g->lock);
}

static int l05_audio_gates_init(struct wcd_mbhc *mbhc)
{
	struct device *dev = mbhc->codec->component.card->dev;
	struct device_node *np = dev->of_node;
	struct l05_audio_gates *g;
	int ret;

	mbhc->l05_gates = NULL;
	if (!of_property_read_bool(np, "leke,l05-audio-gates"))
		return 0;
	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	g->detect = of_get_named_gpio(np, "leke,jack-detect-gpios", 0);
	g->codec = of_get_named_gpio(np, "leke,codec-enable-gpios", 0);
	g->headphone = of_get_named_gpio(np, "leke,headphone-enable-gpios", 0);
	ret = g->detect < 0 ? g->detect :
		(g->codec < 0 ? g->codec : (g->headphone < 0 ? g->headphone : 0));
	if (ret)
		goto free_state;
	ret = gpio_request_one(g->detect, GPIOF_IN, "l05-jack-detect");
	if (ret)
		goto free_state;
	ret = gpio_request_one(g->codec, GPIOF_OUT_INIT_LOW, "l05-codec-enable");
	if (ret)
		goto free_detect;
	ret = gpio_request_one(g->headphone, GPIOF_OUT_INIT_LOW,
			       "l05-headphone-enable");
	if (ret)
		goto free_codec;
	mutex_init(&g->lock);
	INIT_DELAYED_WORK(&g->work, l05_audio_gates_select);
	g->irq = gpio_to_irq(g->detect);
	if (g->irq < 0) {
		ret = g->irq;
		goto free_headphone;
	}
	ret = request_irq(g->irq, l05_audio_gates_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"l05-audio-jack", g);
	if (ret)
		goto free_headphone;
	mbhc->l05_gates = g;
	/* Match stock startup settling, with later edges overriding this delay. */
	schedule_delayed_work(&g->work, msecs_to_jiffies(10000));
	dev_info(dev, "L05 audio gates: GPIOs %d/%d/%d acquired\n",
		g->detect, g->codec, g->headphone);
	return 0;

free_headphone:
	mutex_destroy(&g->lock);
	gpio_free(g->headphone);
free_codec:
	gpio_free(g->codec);
free_detect:
	gpio_free(g->detect);
free_state:
	dev_err(dev, "L05 audio gate setup failed: %d\n", ret);
	kfree(g);
	return ret;
}

/* Caller first unregisters the codec notifier, then tears down gate state. */
static void l05_audio_gates_exit(struct wcd_mbhc *mbhc)
{
	struct l05_audio_gates *g = mbhc->l05_gates;

	if (!g)
		return;
	free_irq(g->irq, g);
	cancel_delayed_work_sync(&g->work);
	gpio_set_value_cansleep(g->codec, 0);
	gpio_set_value_cansleep(g->headphone, 0);
	gpio_free(g->headphone);
	gpio_free(g->codec);
	gpio_free(g->detect);
	mutex_destroy(&g->lock);
	mbhc->l05_gates = NULL;
	kfree(g);
}

#endif
