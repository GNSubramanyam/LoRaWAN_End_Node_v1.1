#ifndef COMMON_H
#define COMMON_H

#include "bmm350_defs.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define BMM350_ADDRESS (0x14 << 1)


BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bmm350_delay(uint32_t period, void *intf_ptr);
int8_t bmm350_interface_init(struct bmm350_dev *dev);

#endif
