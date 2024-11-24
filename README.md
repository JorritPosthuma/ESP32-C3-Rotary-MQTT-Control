# ESP32-C3 Rotary Encoder MQTT publisher

The goal of this code is simple. Expose the values of a rotary encoder via MQTT so that Home Assistant can respond to it. Part of little project to learn OnShape. Design can be found on [Printables Design](https://www.printables.com/model/1079386-wireless-big-round-rotary-button).

![](https://media.printables.com/media/prints/1079386/images/8162594_0e0c6d5d-6caa-4589-a98b-1c3578e633c3_6d4e8c19-58a0-4041-bdb4-3e4dee450cc3/thumbs/inside/1600x1200/jpg/img_0792.webp)

## Features

 * Publish rotary values to HA compatible MQTT topic
 * Publish button presses to HA compatible MQTT topic
 * Easy Wifi & MQTT configuration on first boot (via ESP32 AP mode)
 * Deep sleep mode for energy saving on battery power
 * Throttle rotary pushes to max 1 p/s
 * WebSerial interface for debugging remote
 * Reset configuration when rotary encoder pressed + reset button

## Usage

Just open using Platform.io and upload to ESP32-C3.