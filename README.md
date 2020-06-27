# PedalShield-Nucleo
A digital programmable guitar pedal based on NucleoF446RE.
Based on the Original Project- https://github.com/Guitarman9119/Nucleo_Guitar_Effects_Pedal

Changes from the original project-
1) All programming is now done in a single IDE- STM32CubeIDE
2) Uses an I2C OLED screen
3) ADC and DAC read write are double buffered at 60 Khz sampling rate via DMA

As of now, CLEAN and DISTORTION work superb. Next step is to implement digital filters and Potentiometers. As of nw components are still on a breadboard.
