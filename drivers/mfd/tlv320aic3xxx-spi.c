
#include <linux/err.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/mfd/tlv320aic3xxx-core.h>

struct regmap_config tlv320aic3xxx_spi_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0x1,
};

static int __devinit tlv320aic3xxx_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct aic3xxx *tlv320aic3xxx;
	const struct regmap_config *regmap_config;
	int ret;

	switch (id->driver_data) {
#ifdef CONFIG_SND_SOC_AIC3262
	case TLV320AIC3262:
		regmap_config = &tlv320aic3xxx_spi_regmap;
		break;
#endif
#ifdef CONFIG_MFD_AIC3285
	case TLV320AIC3285:
		regmap_config = &tlv320aic3285_spi_regmap;
		break;
#endif
	default:
		dev_err(&spi->dev, "Unknown device type %ld\n",
			id->driver_data);
		return -EINVAL;
	}

	tlv320aic3xxx =
		devm_kzalloc(&spi->dev, sizeof(struct aic3xxx), GFP_KERNEL);
	if (tlv320aic3xxx == NULL)
		return -ENOMEM;

	tlv320aic3xxx->regmap = devm_regmap_init_spi(spi, regmap_config);
	if (IS_ERR(tlv320aic3xxx->regmap)) {
		ret = PTR_ERR(tlv320aic3xxx->regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	tlv320aic3xxx->type = id->driver_data;
	tlv320aic3xxx->dev = &spi->dev;
	tlv320aic3xxx->irq = spi->irq;

	return aic3xxx_device_init(tlv320aic3xxx, tlv320aic3xxx->irq);
}

static int __devexit tlv320aic3xxx_spi_remove(struct spi_device *spi)
{
	struct aic3xxx *tlv320aic3xxx = dev_get_drvdata(&spi->dev);
	aic3xxx_device_exit(tlv320aic3xxx);
	return 0;
}

static const struct spi_device_id aic3xxx_spi_ids[] = {
	{"tlv320aic3262", TLV320AIC3262},
	{"tlv320aic3285", TLV320AIC3285},
	{ }
};
MODULE_DEVICE_TABLE(spi, aic3xxx_spi_ids);

static struct spi_driver tlv320aic3xxx_spi_driver = {
	.driver = {
		.name	= "tlv320aic3xxx",
		.owner	= THIS_MODULE,
	},
	.probe		= tlv320aic3xxx_spi_probe,
	.remove		= __devexit_p(tlv320aic3xxx_spi_remove),
	.id_table	= aic3xxx_spi_ids,
};

module_spi_driver(tlv320aic3xxx_spi_driver);

MODULE_DESCRIPTION("TLV320AIC3XXX SPI bus interface");
MODULE_AUTHOR("Mukund Navada <navada@ti.com>");
MODULE_LICENSE("GPL");
