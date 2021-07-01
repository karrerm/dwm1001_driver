# DWM1001 ROS driver
This repository contains a simple ROS-driver to use the DWM1001 dev-kit using the Decawave examples.
The driver is intended to be used with the two way example (i.e. two modules measuring the pairwise distance)

## Prepare the UWB modules
Download and the uwb-apps repository from Decawave:
```bash
git clone git@github.com:Decawave/uwb-apps.git
```
Follow the instructions and install the required dependencies.
Note: in my case (Ubuntu 18.04), the gcc-arm-none-eabi package does not contain the gdb anymore which is required by the uwb-apps examples. So I downloaded the tool-chain directly from [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) and add the bin folder of the unpacked package to your system path.
Then carry out the instructions until 5 on every board.
On one board (tag) run the following (same as in app repository):
```bash
newt target create twr_tag_tdma
newt target set twr_tag_tdma app=apps/twr_tag_tdma
newt target set twr_tag_tdma bsp=@decawave-uwb-core/hw/bsp/dwm1001
newt target set twr_tag_tdma build_profile=optimized
newt run twr_tag_tdma 0
```
On the second board (node) run:
```bash
newt target create twr_node_tdma
newt target set twr_node_tdma app=apps/twr_node_tdma
newt target set twr_node_tdma bsp=@decawave-uwb-core/hw/bsp/dwm1001
newt target set twr_node_tdma build_profile=debug
newt target amend twr_node_tdma syscfg=LOG_LEVEL=1:UWBCFG_DEF_ROLE='"0x1"':CONSOLE_RTT=0:CONSOLE_UART=1:CONSOLE_UART_BAUD=460800
newt run twr_node_tdma 0
``` 
With that the boards are setup.
Note: this setup was carried out using the commit 9b935a9d2fb27352ff6be14489817287fa6bd088 of the uwb-apps repository.

## Running the node
As a first step, the parameter serial_port needs to be set to the corresponding serial device that is connected. Note that only the modules configured to run as a node will communicate the corresponding data. 
In order to compensate for inaccuracies, a linear correction model is implemented allowing to scale and offset the raw measurements. To set these values, one can collect a set of measurements at known distances (e.g. using a tape-measurement) and fit the model to the data. 