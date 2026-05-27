
#include "common.h"

HAL_StatusTypeDef status;



static uint8_t dev_addr;

BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	(void)intf_ptr;

	status = HAL_I2C_Master_Transmit(&hi2c1, BMM350_ADDRESS, &reg_addr, 1, 100);
	if (status != HAL_OK) {
		//printf("BMM350 I2C read fail\n");
	}
	//uint32_t err = HAL_I2C_GetError(&hi2c1);

	status = HAL_I2C_Master_Receive(&hi2c1, BMM350_ADDRESS | 0x01, reg_data, length, 100);
	if (status != HAL_OK) {
		//printf("BMM350 I2C read fail\n");
	}
	return BMM350_OK;

}

BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	(void)intf_ptr;
    uint8_t tx_buf[length + 1];

    /* First byte is register address */
    tx_buf[0] = reg_addr;

    /* Copy data bytes */
    for (uint32_t i = 0; i < length; i++)
    {
        tx_buf[i + 1] = reg_data[i];
    }
	status = HAL_I2C_Master_Transmit(&hi2c1, BMM350_ADDRESS, tx_buf, length + 1, 100);
	if (status != HAL_OK) {
		//printf("BMM350 I2C write fail\n");
	}
	return BMM350_OK;

}

void bmm350_delay(uint32_t period, void *intf_ptr)
{
	(void)intf_ptr;
	uint32_t delay_ms = period / 1000;
	HAL_Delay(delay_ms);
}



int8_t bmm350_interface_init(struct bmm350_dev *dev)
{
	int8_t rslt = BMM350_OK;
	dev_addr = BMM350_I2C_ADSEL_SET_LOW;
	dev->read = bmm350_i2c_read;
	dev->write = bmm350_i2c_write;
	dev->delay_us = bmm350_delay;
	return rslt;
}



