---
name: build-procedure
description: How to compile this STM32WL firmware from the CLI to verify changes
metadata:
  type: reference
---

This project (STM32CubeIDE, target STM32WLE5CCUX) has no `arm-none-eabi-gcc` on PATH. Use the IDE-bundled toolchain:

`PATH=/c/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin:$PATH`

Then build the Debug config: `cd Debug && make all -j4`. A clean build links `LoRaWAN_End_Node_v1.1.elf` (~123 KB flash) and exits 0. There is also a `Release/` config dir. Two warnings are pre-existing/benign: `common.c` incompatible write fn-pointer (const mismatch) and `set_int_ctrl` unused in `BMM350_Init`.
