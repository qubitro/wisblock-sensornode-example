# WisBlock Sensor Node Example
![Qubitro](https://img.shields.io/badge/Developed%20by-Qubitro-orange?logo=data:image/png;base64,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&link=http://www.qubitro.com)


This repo contains an example from [Qubitro](https://www.qubitro.com/) that allows an out of the box experience with RAK [WisBlocks](https://store.rakwireless.com/pages/wisblock) where the users can create a sensor node by selecting the sensors chosen in the `main.h`. 

## Usage

Instructions for using the example as well adding to The Things Stack can be found [in the Qubitro documentation](https://docs.qubitro.com/client-guides/connect-device/lorawan-devices/wisblock#learning-by-example). All you need to get started with the project is platformio, WisBlock support, and editing `main.h` if you have the RAK WisBlock starter kit.

## TODO WIP

* Add theft detection with accelerometer
* Add GPS
* Deep sleep, investigating issues with RAK example


