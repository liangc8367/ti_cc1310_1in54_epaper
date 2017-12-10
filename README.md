# ti_cc1310_1in54_epaper

Demo code to drive the 1.54 inch epaper module by TI CC1310 LaunchPad.

The module was manufactured by Waveshare https://www.waveshare.com/wiki/1.54inch_e-Paper_Module_(B).

The driver code was also modified slightly based on the demo code from Waveshare.

TI CC1310 Launchpad: http://www.ti.com/tool/LAUNCHXL-CC1310

Interface:
- SPI0
- GPIO:
    # SPI CS: DIO15 (output)
    # RST   : DIO01 (output)
    # D/C   : DIO11 (output)
    # BUSY  : DIO13 (input)
