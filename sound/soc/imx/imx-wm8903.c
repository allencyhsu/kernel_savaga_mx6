/*
 * imx-wm8903.c
 *
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2013 Poslab co. Ltd. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/kthread.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <mach/audmux.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>

#include "imx-ssi.h"
#include "../codecs/wm8903.h"

struct imx_priv {
	int sysclk;         /*mclk from the outside*/
	int codec_sysclk;
	int dai_hifi;
	struct platform_device *pdev;
	struct snd_pcm_substream *first_stream;
	struct snd_pcm_substream *second_stream;
};
unsigned int sample_format = SNDRV_PCM_FMTBIT_S16_LE;
static struct imx_priv card_priv;
static struct snd_soc_card snd_soc_card_imx;
static struct snd_soc_codec *gcodec;

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat = priv->pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(1);

	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat = priv->pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(0);

	return;
}

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	unsigned int channels = params_channels(params);
	int ret = 0;
	u32 dai_format;

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;
        
	ret = snd_soc_dai_set_sysclk(codec_dai,
					 0,
					 priv->sysclk,
					 SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_jack imx_wm8903_mic_jack;

static struct snd_soc_jack_pin imx_wm8903_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static const struct snd_kcontrol_new controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
};

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Mic Jack", NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headphone Jack", NULL, "HPOUTL" },
	{ "Headphone Jack", NULL, "HPOUTR" },
	{ "Mic Bias", NULL, "Mic Jack"},
	{ "IN1L", NULL, "Mic Jack"},
};

static int imx_wm8903_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int ret = 0;

	gcodec = rtd->codec;

	/* Add imx specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_dapm_widgets,
				  ARRAY_SIZE(imx_dapm_widgets));

	/* Set up imx specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");

	snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
			 &imx_wm8903_mic_jack);
	snd_soc_jack_add_pins(&imx_wm8903_mic_jack,
			      ARRAY_SIZE(imx_wm8903_mic_jack_pins),
			      imx_wm8903_mic_jack_pins);
	wm8903_mic_detect(codec, &imx_wm8903_mic_jack, 0, SND_JACK_MICROPHONE);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "Mic Bias");
	snd_soc_dapm_sync(&codec->dapm);

	return 0;
}

static struct snd_soc_ops imx_hifi_ops = {
	.startup = imx_hifi_startup,
	.shutdown = imx_hifi_shutdown,
	.hw_params = imx_hifi_hw_params,
};

static struct snd_soc_dai_link imx_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name	= "wm8903-hifi",
		.codec_name	= "wm8903.0-001a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_wm8903_init,
		.ops		= &imx_hifi_ops,
	},
};

static struct snd_soc_card snd_soc_card_imx = {
	.name		= "wm8903-audio",
	.dai_link	= imx_dai,
	.num_links	= ARRAY_SIZE(imx_dai),
	.controls = controls,
	.num_controls = ARRAY_SIZE(controls),
	.dapm_widgets = imx_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(imx_dapm_widgets),
};

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_wm8903_probe(struct platform_device *pdev)
{

	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_priv *priv = &card_priv;
	int ret = 0;

	priv->pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	if (plat->init && plat->init()) {
		ret = -EINVAL;
		return ret;
	}

	priv->sysclk = plat->sysclk;

	priv->first_stream = NULL;
	priv->second_stream = NULL;

	return ret;
}

static int __devexit imx_wm8903_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_priv *priv = &card_priv;

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_wm8903_driver = {
	.probe = imx_wm8903_probe,
	.remove = imx_wm8903_remove,
	.driver = {
		   .name = "imx-wm8903",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_snd_device;

static int __init imx_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm8903_driver);
	if (ret < 0)
		goto exit;

	imx_snd_device = platform_device_alloc("soc-audio", 5);
	if (!imx_snd_device)
		goto err_device_alloc;

	platform_set_drvdata(imx_snd_device, &snd_soc_card_imx);

	ret = platform_device_add(imx_snd_device);

	if (0 == ret)
		goto exit;

	platform_device_put(imx_snd_device);

err_device_alloc:
	platform_driver_unregister(&imx_wm8903_driver);
exit:
	return ret;
}

static void __exit imx_asoc_exit(void)
{
	platform_driver_unregister(&imx_wm8903_driver);
	platform_device_unregister(imx_snd_device);
}

module_init(imx_asoc_init);
module_exit(imx_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC imx wm8903");
MODULE_LICENSE("GPL");
