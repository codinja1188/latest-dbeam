#ifndef _DAC_H_
#define _DAC_H_


struct dac_data {
	dev_t devt;
	struct device 	*dev;
	struct class	*dac_class;
};

#endif
