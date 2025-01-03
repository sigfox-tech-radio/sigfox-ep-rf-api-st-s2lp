# S2LP RF API implementation example

## Description

This **S2LP RF API** is a low level implementation example of the [Sigfox EP library](https://github.com/sigfox-tech-radio/sigfox-ep-lib), showing the `manuf/rf_api.c` file implementation for the [S2LP](https://www.st.com/en/wireless-connectivity/s2-lp.html) transceiver from ST-Microelectronics.

> [!WARNING]  
> The resulting radio performances of your device strongly depends on your **hardware design** (schematic, PCB routing, crystal oscillator placement, good RF practices, etc...). **Sigfox certification remains mandatory** whatever the software embedded in your device (including the Sigfox End-Point library and its implementation examples).

The table below shows the versions compatibility between this radio example and the Sigfox End-Point library.

| **S2LP_RF_API** | **EP_LIB** |
|:---:|:---:|
| [v4.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v4.0) | >= [v4.0](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v4.0) |
| [v3.1](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v3.1) | [v3.6](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.6) |
| [v3.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v3.0) | [v3.5](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.5) |
| [v2.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v2.0) | [v3.5](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.5) |
| [v1.3](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.3) | [v3.4](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.4) |
| [v1.2](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.2) | [v3.2](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.2) to [v3.3](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.3) |
| [v1.1](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.1) | [v3.2](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.2) to [v3.3](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.3) |
| [v1.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.0) | [v3.2](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.2) to [v3.3](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.3) |

## Architecture

<p align="center">
<img src="https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/wiki/images/sigfox_ep_rf_api_st_s2lp_architecture.drawio.png" width="600"/>
</p>

## Compilation flags for optimization

This radio example inherits all the [Sigfox End-Point library flags](https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki/compilation-flags-for-optimization) and can be optimized accordingly.

The `SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE` flag must be enabled to use this example.

## How to add S2LP RF API example to your project

### Dependencies

The **S2LP RF API** is based on the [official S2LP driver](https://github.com/STMicroelectronics/x-cube-subg2/tree/main/Drivers/BSP/Components/S2LP/S2LP_Library) from ST-Microelectronics. This driver exposes a functional interface to configure the chip registers, which is used to perform the Sigfox uplink modulation and optional downlink reception. It relies on **low level functions** (called board drivers) which need to be implemented to run on your specific hardware. There are divided in 2 groups:

* **S2LP_CORE_SPI** : low level SPI access functions of the S2LP driver from ST.
* **S2LP_HW_API** : additional hardware dependent functions required to get a generic RF API implementation of the S2LP.

The templates are located in the `src/board` folder.

### Submodule

The best way to embed the S2LP RF API example into your project is to use a [Git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules), in a similar way to the library. The radio driver will be seen as a sub-repository with independant history. It will be much easier to **upgrade the radio driver** or to **switch between versions** when necessary, by using the common `git pull` and `git checkout` commands within the `sigfox-ep-rf-api-st-s2lp` folder.

To add the S2LP RF API submodule, go to your project location and run the following commands:

```bash
mkdir lib
cd lib/
git submodule add https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp.git
```

This will clone the S2LP RF API repository. At project level, you can commit the submodule creation with the following commands:

```bash
git commit --message "Add Sigfox S2LP RF API submodule."
git push
```

With the submodule, you can easily:

* Update the radio driver to the **latest version**:

```bash
cd lib/sigfox-ep-rf-api-st-s2lp/
git pull
git checkout master
```

* Use a **specific release**:

```bash
cd lib/sigfox-ep-rf-api-st-s2lp/
git pull
git checkout <tag>
```

### Raw source code

You can [download](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases) or clone any release of the S2LP RF API example and copy all files into your project.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp.git
```

### Precompiled source code

You can [download](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases) or clone any release of the S2LP RF API example and copy all files into your project. If you do not plan to change your compilation flags in the future, you can perform a **precompilation step** before copying the file in your project. The precompilation will **remove all preprocessor directives** according to your flags selection, in order to produce a more **readable code**. Then you can copy the new files into your project.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp.git
```

To perform the precompilation, you have to install `cmake` and `unifdef` tools, and run the following commands:

```bash
cd sigfox-ep-rf-api-st-s2lp/
mkdir build
cd build/

cmake -DSIGFOX_EP_LIB_DIR=<sigfox-ep-lib path> \
      -DSIGFOX_EP_RC1_ZONE=ON \
      -DSIGFOX_EP_RC2_ZONE=ON \
      -DSIGFOX_EP_RC3_LBT_ZONE=ON \
      -DSIGFOX_EP_RC3_LDC_ZONE=ON \
      -DSIGFOX_EP_RC4_ZONE=ON \
      -DSIGFOX_EP_RC5_ZONE=ON \
      -DSIGFOX_EP_RC6_ZONE=ON \
      -DSIGFOX_EP_RC7_ZONE=ON \
      -DSIGFOX_EP_APPLICATION_MESSAGES=ON \
      -DSIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DSIGFOX_EP_BIDIRECTIONAL=ON \
      -DSIGFOX_EP_ASYNCHRONOUS=ON \
      -DSIGFOX_EP_LOW_LEVEL_OPEN_CLOSE=ON \
      -DSIGFOX_EP_REGULATORY=ON \
      -DSIGFOX_EP_LATENCY_COMPENSATION=ON \
      -DSIGFOX_EP_SINGLE_FRAME=ON \
      -DSIGFOX_EP_UL_BIT_RATE_BPS=OFF \
      -DSIGFOX_EP_TX_POWER_DBM_EIRP=OFF \
      -DSIGFOX_EP_T_IFU_MS=OFF \
      -DSIGFOX_EP_T_CONF_MS=OFF \
      -DSIGFOX_EP_UL_PAYLOAD_SIZE=OFF \
      -DSIGFOX_EP_AES_HW=ON \
      -DSIGFOX_EP_CRC_HW=OFF \
      -DSIGFOX_EP_MESSAGE_COUNTER_ROLLOVER=OFF \
      -DSIGFOX_EP_PARAMETERS_CHECK=ON \
      -DSIGFOX_EP_CERTIFICATION=ON \
      -DSIGFOX_EP_PUBLIC_KEY_CAPABLE=ON \
      -DSIGFOX_EP_VERBOSE=ON \
      -DSIGFOX_EP_ERROR_CODES=ON \
      -DSIGFOX_EP_ERROR_STACK=12 ..

make precompil_s2lp_rf_api
```

The new files will be generated in the `build/precompil` folder.

### Static library

You can also [download](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases) or clone any release of the S2LP RF API example and build a **static library**.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp.git
```

To build a static library, you have to install `cmake` tool and run the following commands:

```bash
cd sigfox-ep-rf-api-st-s2lp/
mkdir build
cd build/

cmake -DSIGFOX_EP_LIB_DIR=<sigfox-ep-lib path> \
      -DSIGFOX_EP_RC1_ZONE=ON \
      -DSIGFOX_EP_RC2_ZONE=ON \
      -DSIGFOX_EP_RC3_LBT_ZONE=ON \
      -DSIGFOX_EP_RC3_LDC_ZONE=ON \
      -DSIGFOX_EP_RC4_ZONE=ON \
      -DSIGFOX_EP_RC5_ZONE=ON \
      -DSIGFOX_EP_RC6_ZONE=ON \
      -DSIGFOX_EP_RC7_ZONE=ON \
      -DSIGFOX_EP_APPLICATION_MESSAGES=ON \
      -DSIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DSIGFOX_EP_BIDIRECTIONAL=ON \
      -DSIGFOX_EP_ASYNCHRONOUS=ON \
      -DSIGFOX_EP_LOW_LEVEL_OPEN_CLOSE=ON \
      -DSIGFOX_EP_REGULATORY=ON \
      -DSIGFOX_EP_LATENCY_COMPENSATION=ON \
      -DSIGFOX_EP_SINGLE_FRAME=ON \
      -DSIGFOX_EP_UL_BIT_RATE_BPS=OFF \
      -DSIGFOX_EP_TX_POWER_DBM_EIRP=OFF \
      -DSIGFOX_EP_T_IFU_MS=OFF \
      -DSIGFOX_EP_T_CONF_MS=OFF \
      -DSIGFOX_EP_UL_PAYLOAD_SIZE=OFF \
      -DSIGFOX_EP_AES_HW=ON \
      -DSIGFOX_EP_CRC_HW=OFF \
      -DSIGFOX_EP_MESSAGE_COUNTER_ROLLOVER=OFF \
      -DSIGFOX_EP_PARAMETERS_CHECK=ON \
      -DSIGFOX_EP_CERTIFICATION=ON \
      -DSIGFOX_EP_PUBLIC_KEY_CAPABLE=ON \
      -DSIGFOX_EP_VERBOSE=ON \
      -DSIGFOX_EP_ERROR_CODES=ON \
      -DSIGFOX_EP_ERROR_STACK=12 ..

make s2lp_rf_api
```

The archive will be generated in the `build/lib` folder.
