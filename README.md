# Description

This repository contains the **FXLS89xxxx** accelerometer sensor driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **fxls89xxxx-driver** | **embedded-utils** |
|:---:|:---:|
| [sw1.2](https://github.com/Ludovic-Lesur/fxls89xxxx-driver/releases/tag/sw1.2) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw1.1](https://github.com/Ludovic-Lesur/fxls89xxxx-driver/releases/tag/sw1.1) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw1.0](https://github.com/Ludovic-Lesur/fxls89xxxx-driver/releases/tag/sw1.0) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `FXLS89XXXX_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `fxls89xxxx_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `FXLS89XXXX_DRIVER_DISABLE` | `defined` / `undefined` | Disable the FXLS89xxxx driver. |
| `FXLS89XXXX_DRIVER_I2C_ERROR_BASE_LAST` | `<value>` | Last error base of the low level I2C driver. |
