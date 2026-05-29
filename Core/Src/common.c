
#include "common.h"
#include "i2c.h"

/* Consecutive I2C failure counter. Reset on any successful transaction.
 * Application code may poll this and call bmm350_i2c_bus_recover() when it grows. */
volatile uint32_t bmm350_i2c_fail_count = 0;

BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	(void)intf_ptr;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Master_Transmit(&hi2c1, BMM350_ADDRESS, &reg_addr, 1, 100);
	if (status != HAL_OK) {
		bmm350_i2c_fail_count++;
		return BMM350_E_COM_FAIL;
	}

	/* HAL handles the R/W bit internally; pass the same (8-bit) address as the write. */
	status = HAL_I2C_Master_Receive(&hi2c1, BMM350_ADDRESS, reg_data, length, 100);
	if (status != HAL_OK) {
		bmm350_i2c_fail_count++;
		return BMM350_E_COM_FAIL;
	}

	bmm350_i2c_fail_count = 0;
	return BMM350_OK;
}

BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	(void)intf_ptr;
	HAL_StatusTypeDef status;
    uint8_t tx_buf[length + 1];

    tx_buf[0] = reg_addr;
    for (uint32_t i = 0; i < length; i++)
    {
        tx_buf[i + 1] = reg_data[i];
    }
	status = HAL_I2C_Master_Transmit(&hi2c1, BMM350_ADDRESS, tx_buf, length + 1, 100);
	if (status != HAL_OK) {
		bmm350_i2c_fail_count++;
		return BMM350_E_COM_FAIL;
	}

	bmm350_i2c_fail_count = 0;
	return BMM350_OK;
}

void bmm350_delay(uint32_t period, void *intf_ptr)
{
	(void)intf_ptr;
	uint32_t delay_ms = period / 1000;
	if (delay_ms == 0) delay_ms = 1;
	HAL_Delay(delay_ms);
}

int8_t bmm350_interface_init(struct bmm350_dev *dev)
{
	int8_t rslt = BMM350_OK;
	dev->read = bmm350_i2c_read;
	dev->write = bmm350_i2c_write;
	dev->delay_us = bmm350_delay;
	return rslt;
}

/* Bus recovery: 9 SCL pulses to release a slave that is holding SDA low,
 * then a manual STOP, then re-init the I2C peripheral.
 * Returns HAL_OK if SDA is high (bus free) after the sequence. */
HAL_StatusTypeDef bmm350_i2c_bus_recover(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_I2C_DeInit(&hi2c1);
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB6 = SCL as open-drain output, PB7 = SDA as input. */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);

	for (int i = 0; i < 9; i++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(1);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) break;
	}

	/* Manual STOP: SDA low→high while SCL high. */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_StatusTypeDef sda_ok = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? HAL_OK : HAL_ERROR;

	__HAL_RCC_I2C1_CLK_ENABLE();
	MX_I2C1_Init();

	bmm350_i2c_fail_count = 0;
	return sda_ok;
}
