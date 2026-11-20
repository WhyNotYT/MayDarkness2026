# MayDarkness2026

This project is a project testing how far we can send messages from an Arduino Nano using it's GPIO pins connected to a WiFi antenna from a router and a coper plate, not impedance matching has been performed.
The idea:

Arduino Nano has no WiFi or Bluetooth but is a perfectly good microcontroller, could we have a nano in end of a table send messages to an ESP32 or a Raspberry Pi Pico W on the other end of the room, which can then forward it over the internet.

The setup:
- You have access to a arduino nano over USB Serial.
- Pin D10 of the nano is connected to a WiFi antenna from a router
- Pin D11 is connected to a 4cmx3cm copper plate

- You also have access to a raspberry pi pico W over USB
- It's GPIO28 is connected to a similar copper plate
- It's GPIO27 is connected to a WiFi antenna from a router

- You also have access to an ESP32 over USB
- GPIO34 is connected to a long jumper wire ~20cm which can act as an antenna
- There is the esp-sdr/ project which lets you get the raw samples from the WiFi radio on the ESP

What has been done:
Under NanoWireless/chirp-sender runs on the Arduino Nano and sends LoRa like chirp signals using the GPIO
chirp-sdr and chirp-sdr-esp run a simple SDR that plots the readings from the GPIO pin
chirp-receiver tries to decode the chirp
The setup works for up to about 20cm distance between the Pico W and Nano.


The setup:
Right now the Nano is placed 30cm away from the ESP32 and Pico W, come up with a method to transmit data from the Nano to either or both of the other microcontollers (ESP32/Pico W). The Pico W has a better ADC so the antenna, and copper plate is connected to it. While the ESP has access to the raw WiFi radio samples (this goes down to 1 Ghz). Try things out until you manage to sucesfully transmit some bytes between them. Utilize all the different physical properties of RF waves. For example, if we send a square wave using the Nano, it's harmonics might land in the 1Ghz range and the esp might be able to find it, or if we can get the radio to find a lower harmonic of the 2.4Ghz antennas. Or using "capacitance" using the copper plates.

Port the esp-sdr to platformio if it makes your life easier. Install whatever toolchain you want. You have access to platformIO to reprogram and monitor all the microcontrollers.

Goodluck!
