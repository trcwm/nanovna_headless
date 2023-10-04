# NanoVNA Headless - Firmware for a NanoVNA without a display

# About

This firmware streamlines the USB serial I/O and removes all user interface functions.
A Raspberry Pi can then be used to build a VNA with a bigger display.

This firmware is based on edy555's original NanoVNA firmware.

## Prepare ARM Cross Tools

### Linux (ubuntu)

Download arm cross tools from [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

    $ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
    $ sudo tar xfj gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 -C /usr/local
    $ PATH=/usr/local/gcc-arm-none-eabi-8-2018-q4-major/bin:$PATH
    $ sudo apt install -y dfu-util

## Fetch source code

Fetch source and submodule.

    $ git clone https://github.com/trcwm/nanovna_headless.git
    $ cd nanovna_headless
    $ git submodule update --init --recursive

## Build

Just make in the directory.

    $ make

## Flash firmware

First, make device enter DFU mode by one of following methods.

* Jumper BOOT0 pin at powering device
* Select menu Config->DFU (needs recent firmware)

Then, flash firmware using dfu-util via USB.

    $ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin

Or simply use make.

    $ make flash

## Reference

* [Schematics](/doc/nanovna-sch.pdf)
* [PCB Photo](/doc/nanovna-pcb-photo.jpg)
* [Block Diagram](/doc/nanovna-blockdiagram.png)

## Credits

* [@trcwm](https://github.com/trcwm)
* [@edy555](https://github.com/edy555)

### Contributors

* [@hugen79](https://github.com/hugen79)
* [@cho45](https://github.com/cho45)
* [@DiSlord](https://github.com/DiSlord/)
