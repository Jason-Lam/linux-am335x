/*
 * ASoC driver for TI DAVINCI EVM platform
 *
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <asm/hardware/asp.h>
#include <mach/edma.h>
#ifdef CONFIG_MACH_AM335XEVM
#include <mach/board-am335xevm.h>
#endif
#ifdef CONFIG_MACH_IPC335X
#include <mach/board-ipc335x.h>
#include "../codecs/wm8960.h"
#endif

#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"

#ifdef  DEBUG
#define dprintk( argc, argv... )        printk( argc, ##argv )
#else
#define dprintk( argc, argv... )        
#endif

#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | \
		SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)
static int evm_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned sysclk;

	/* ASP1 on DM355 EVM is clocked by an external oscillator */
	if (machine_is_davinci_dm355_evm() || machine_is_davinci_dm6467_evm() ||
	    machine_is_davinci_dm365_evm())
		sysclk = 27000000;

	/* ASP0 in DM6446 EVM is clocked by U55, as configured by
	 * board-dm644x-evm.c using GPIOs from U18.  There are six
	 * options; here we "know" we use a 48 KHz sample rate.
	 */
	else if (machine_is_davinci_evm())
		sysclk = 12288000;

	else if (machine_is_davinci_da830_evm() ||
				machine_is_davinci_da850_evm())
		sysclk = 24576000;
	/* On AM335X, CODEC gets MCLK from external Xtal (12MHz). */
	else if (machine_is_am335xevm() || machine_is_ipc335x())
#ifdef CONFIG_MACH_AM335XEVM
		if (am335x_evm_get_id() == EVM_SK)
			sysclk = 24000000;
		else
#endif
			sysclk = 12000000;
	else
		return -EINVAL;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

#ifdef CONFIG_MACH_IPC335X
	if (ipc335x_dock_get_id() == HMI335X){
		ret = snd_soc_dai_set_pll(codec_dai, 0, 0, sysclk, 11289600);
		if (ret < 0)
			return ret;
	}
#endif

	return 0;
}

#ifdef CONFIG_MACH_IPC335X
static int hmi335x_wm8960_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int rate = params_rate(params);
	snd_pcm_format_t fmt = params_format( params );
	unsigned sysclk;
	int ret = 0;

	int bclk_div;
	int dacdiv;

	dprintk("+%s()\n", __FUNCTION__ );

	switch ( fmt ) {
	case SNDRV_PCM_FORMAT_S16:
		bclk_div = 32;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24:
		bclk_div = 48;
		break;
	default:
		dprintk("-%s(): PCM FMT ERROR\n", __FUNCTION__ );
		return -EINVAL;
	}
	
	switch ( rate ) {
	case 8018:
		dacdiv = WM8960_DAC_DIV_5_5;
		sysclk = 11289600;
		break;
	case 11025:
		dacdiv = WM8960_DAC_DIV_4;
		sysclk = 11289600;
		break;
	case 22050: 
		dacdiv = WM8960_DAC_DIV_2;
		sysclk = 11289600;
		break;
	case 44100:
		dacdiv = WM8960_DAC_DIV_1;
		sysclk = 11289600;
		break;	
	case 8000:
		dacdiv = WM8960_DAC_DIV_6;
		sysclk = 12288000;
		break;
	case 12000: 
		dacdiv = WM8960_DAC_DIV_4;
		sysclk = 12288000;
		break;
	case 16000: 
		dacdiv = WM8960_DAC_DIV_3;
		sysclk = 12288000;
		break;
	case 24000:
		dacdiv = WM8960_DAC_DIV_2;
		sysclk = 12288000;
		break;
	case 32000:
		dacdiv = WM8960_DAC_DIV_1_5;
		sysclk = 12288000;
		break;
	case 48000:
		dacdiv = WM8960_DAC_DIV_1;
		sysclk = 12288000;
		break;
	default:
		dprintk("-%s(): SND RATE ERROR (%d)\n", __FUNCTION__,rate );
		return -EINVAL;
	}
	dprintk("-%s(): rate is %d\n", __FUNCTION__,rate );
	ret = snd_soc_dai_set_clkdiv( codec_dai,  WM8960_DACDIV, dacdiv );
	if( ret < 0 ){
		dprintk( "-%s(): Codec SYSCLKDIV setting error, %d\n", __FUNCTION__, ret );
		return ret;
	}
	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if( ret < 0 ){
		dprintk( "-%s(): Codec DAI configuration error, %d\n", __FUNCTION__, ret );
		return ret;
	}
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if( ret < 0 ){
	    dprintk( "-%s(): AP DAI configuration error, %d\n", __FUNCTION__, ret );
	    return ret;
	}
	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, 12000000, sysclk);
	dprintk("-%s()\n", __FUNCTION__ );
	return 0;
}

static struct snd_soc_ops hmi335x_wm8960_ops = {
	.hw_params = hmi335x_wm8960_hw_params,
};
#endif

static int evm_spdif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	/* set cpu DAI configuration */
	return snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
}

static struct snd_soc_ops evm_ops = {
	.hw_params = evm_hw_params,
};

static struct snd_soc_ops evm_spdif_ops = {
	.hw_params = evm_spdif_hw_params,
};

/* davinci-evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

/* davinci-evm machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},
};

/* Logic for a aic3x as connected on a davinci-evm */
static int evm_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(dapm, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "MONO_LOUT");
	snd_soc_dapm_disable_pin(dapm, "HPLCOM");
	snd_soc_dapm_disable_pin(dapm, "HPRCOM");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");

	return 0;
}

#ifdef CONFIG_MACH_IPC335X
static const struct snd_soc_dapm_widget hmi335x_dapm_capture_widgets[] = {
	SND_SOC_DAPM_MIC(   "Mic Jack",         NULL ),
	SND_SOC_DAPM_LINE(  "Line Input 3 (FM)",NULL ),
};

static const struct snd_soc_dapm_widget hmi335x_dapm_playback_widgets[] = {
	SND_SOC_DAPM_HP(    "Headphone Jack",   NULL ),
	SND_SOC_DAPM_SPK(   "Speaker_L",        NULL ),
	SND_SOC_DAPM_SPK(   "Speaker_R",        NULL ),
};

static const struct snd_soc_dapm_route hmi335x_audio_map[] = {
	{ "Headphone Jack", NULL,   "HP_L"      },
	{ "Headphone Jack", NULL,   "HP_R"      },
	{ "Speaker_L",      NULL,   "SPK_LP"    }, 
	{ "Speaker_L",      NULL,   "SPK_LN"    }, 
	{ "Speaker_R",      NULL,   "SPK_RP"    }, 
	{ "Speaker_R",      NULL,   "SPK_RN"    }, 
	{ "LINPUT1",        NULL,   "MICB"      },
	{ "MICB",           NULL,   "Mic Jack"  },
};

static int hmi335x_wm8960_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	dprintk("+%s()\n", __FUNCTION__ );

	snd_soc_dapm_nc_pin(dapm, "RINPUT1");
	snd_soc_dapm_nc_pin(dapm, "LINPUT2");
	snd_soc_dapm_nc_pin(dapm, "RINPUT2");
	snd_soc_dapm_nc_pin(dapm, "OUT3");

	snd_soc_dapm_new_controls(dapm, hmi335x_dapm_capture_widgets,
			ARRAY_SIZE(hmi335x_dapm_capture_widgets ) );
	snd_soc_dapm_new_controls(dapm, hmi335x_dapm_playback_widgets,
			ARRAY_SIZE(hmi335x_dapm_playback_widgets ) );

	snd_soc_dapm_add_routes(dapm, hmi335x_audio_map, 
			ARRAY_SIZE(hmi335x_audio_map));

	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Speaker_L");
	snd_soc_dapm_enable_pin(dapm, "Speaker_R");

	snd_soc_dapm_disable_pin(dapm, "Line Input 3 (FM)");

	dprintk("*%s(): dapm sync start\n", __FUNCTION__ );
	snd_soc_dapm_sync( dapm );
	dprintk("*%s(): dapm sync end\n", __FUNCTION__ );

	dprintk("-%s()\n", __FUNCTION__ );
	return 0;
}
#endif

/* davinci-evm digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dm6446_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link dm355_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link dm365_evm_dai = {
#ifdef CONFIG_SND_DM365_AIC3X_CODEC
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp",
	.codec_dai_name = "tlv320aic3x-hifi",
	.init = evm_aic3x_init,
	.codec_name = "tlv320aic3x-codec.1-0018",
	.ops = &evm_ops,
#elif defined(CONFIG_SND_DM365_VOICE_CODEC)
	.name = "Voice Codec - CQ93VC",
	.stream_name = "CQ93",
	.cpu_dai_name = "davinci-vcif",
	.codec_dai_name = "cq93vc-hifi",
	.codec_name = "cq93vc-codec",
#endif
	.platform_name = "davinci-pcm-audio",
};

static struct snd_soc_dai_link dm6467_evm_dai[] = {
	{
		.name = "TLV320AIC3X",
		.stream_name = "AIC3X",
		.cpu_dai_name= "davinci-mcasp.0",
		.codec_dai_name = "tlv320aic3x-hifi",
		.platform_name ="davinci-pcm-audio",
		.codec_name = "tlv320aic3x-codec.0-001a",
		.init = evm_aic3x_init,
		.ops = &evm_ops,
	},
	{
		.name = "McASP",
		.stream_name = "spdif",
		.cpu_dai_name= "davinci-mcasp.1",
		.codec_dai_name = "dit-hifi",
		.codec_name = "spdif_dit",
		.platform_name = "davinci-pcm-audio",
		.ops = &evm_spdif_ops,
	},
};

static struct snd_soc_dai_link da830_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link da850_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name= "davinci-mcasp.0",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link am335x_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.2-001b",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link am335x_evm_sk_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

#ifdef CONFIG_MACH_IPC335X
static struct snd_soc_dai_link ipc335x_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.0",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.3-001b",
	.platform_name = "davinci-pcm-audio",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link hmi335x_dai = {
	.name = "WM8960",
	.stream_name = "8960",
	.cpu_dai_name = "davinci-mcasp.1",
	.codec_dai_name = "wm8960-hifi",
	.codec_name = "wm8960-codec.3-001a",
	.platform_name = "davinci-pcm-audio",
	.init = hmi335x_wm8960_init,
	.ops = &hmi335x_wm8960_ops,
};
#endif

/* davinci dm6446 evm audio machine driver */
static struct snd_soc_card dm6446_snd_soc_card_evm = {
	.name = "DaVinci DM6446 EVM",
	.dai_link = &dm6446_evm_dai,
	.num_links = 1,
};

/* davinci dm355 evm audio machine driver */
static struct snd_soc_card dm355_snd_soc_card_evm = {
	.name = "DaVinci DM355 EVM",
	.dai_link = &dm355_evm_dai,
	.num_links = 1,
};

/* davinci dm365 evm audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_evm = {
	.name = "DaVinci DM365 EVM",
	.dai_link = &dm365_evm_dai,
	.num_links = 1,
};

/* davinci dm6467 evm audio machine driver */
static struct snd_soc_card dm6467_snd_soc_card_evm = {
	.name = "DaVinci DM6467 EVM",
	.dai_link = dm6467_evm_dai,
	.num_links = ARRAY_SIZE(dm6467_evm_dai),
};

static struct snd_soc_card da830_snd_soc_card = {
	.name = "DA830/OMAP-L137 EVM",
	.dai_link = &da830_evm_dai,
	.num_links = 1,
};

static struct snd_soc_card da850_snd_soc_card = {
	.name = "DA850/OMAP-L138 EVM",
	.dai_link = &da850_evm_dai,
	.num_links = 1,
};

static struct snd_soc_card am335x_snd_soc_card = {
	.name = "AM335X EVM",
	.dai_link = &am335x_evm_dai,
	.num_links = 1,
};

static struct snd_soc_card am335x_evm_sk_snd_soc_card = {
	.name = "AM335X EVM",
	.dai_link = &am335x_evm_sk_dai,
	.num_links = 1,
};

#ifdef CONFIG_MACH_IPC335X
static struct snd_soc_card ipc335x_snd_soc_card = {
	.name = "IPC335X EVM",
	.dai_link = &ipc335x_evm_dai,
	.num_links = 1,
};

static struct snd_soc_card hmi335x_snd_soc_card = {
	.name = "HMI335X",
	.dai_link = &hmi335x_dai,
	.num_links = 1,
};
#endif

static struct platform_device *evm_snd_device;

static int __init evm_init(void)
{
	struct snd_soc_card *evm_snd_dev_data;
	int index;
	int ret;

	if (machine_is_davinci_evm()) {
		evm_snd_dev_data = &dm6446_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_dm355_evm()) {
		evm_snd_dev_data = &dm355_snd_soc_card_evm;
		index = 1;
	} else if (machine_is_davinci_dm365_evm()) {
		evm_snd_dev_data = &dm365_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_dm6467_evm()) {
		evm_snd_dev_data = &dm6467_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_da830_evm()) {
		evm_snd_dev_data = &da830_snd_soc_card;
		index = 1;
	} else if (machine_is_davinci_da850_evm()) {
		evm_snd_dev_data = &da850_snd_soc_card;
		index = 0;
	} else if (machine_is_am335xevm()) {
		evm_snd_dev_data = &am335x_snd_soc_card;
#ifdef CONFIG_MACH_AM335XEVM
		if (am335x_evm_get_id() == EVM_SK)
			evm_snd_dev_data = &am335x_evm_sk_snd_soc_card;
#endif
		index = 0;
	}
#ifdef CONFIG_MACH_IPC335X
	else if (machine_is_ipc335x()) {
		evm_snd_dev_data = &ipc335x_snd_soc_card;
	if (ipc335x_dock_get_id() == HMI335X)
		evm_snd_dev_data = &hmi335x_snd_soc_card;
		index = 0;
	}
#endif
	else
		return -EINVAL;

	evm_snd_device = platform_device_alloc("soc-audio", index);
	if (!evm_snd_device)
		return -ENOMEM;

	platform_set_drvdata(evm_snd_device, evm_snd_dev_data);
	ret = platform_device_add(evm_snd_device);
	if (ret)
		platform_device_put(evm_snd_device);

	return ret;
}

static void __exit evm_exit(void)
{
	platform_device_unregister(evm_snd_device);
}

module_init(evm_init);
module_exit(evm_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("TI DAVINCI EVM ASoC driver");
MODULE_LICENSE("GPL");
