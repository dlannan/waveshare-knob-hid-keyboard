# waveshare-knob-hid-keyboard

A simple Arduino BLE hid keyboard for the waveshare 1.8" knob. 

The library is based on a number of examples:

- Knob Demo from Waveshare: https://www.waveshare.com/wiki/ESP32-S3-Knob-Touch-LCD-1.8

- Knob Demo from Volos Projects: https://www.youtube.com/watch?v=8pHF0OAG2TI

- BLE HID Keyboard from T-vK: https://github.com/T-vK/ESP32-BLE-Keyboard

## Required Arduino libraries

SensorLib by Lewis He - version 0.3.1

lvgl by kisvegabor - version 8.4.0

![alt text](https://github.com/dlannan/waveshare-knob-hid-keyboard/blob/main/media/device_screenshot.jpg)

## Notes

This is the new device I decided to use as a replacement to my previous mini-keyboard that was USB driven, not BT.

https://github.com/dlannan/mini-keyboard

## Main Features

The sketch is currently very messy, and I will get to it and clean it up over time.

The core features are:

- Touch display using lvgl to select a dial/object and then use the encoder to control selected object.

- Currently volume, and macro controls are available in the HID control. Will add more macros.

- Sleep mode is 20 seconds. Note: It can take a few seconds for BlueTooth to "wake up and connect" coming out of sleep. So if you need fast responses for BT, then disable the sleep code.

The right and bottom dials do nothing at the moment. The current plan is to change these to cpu, network, and other stats (maybe configurable).

# License

MIT License... do what you want with it :)

