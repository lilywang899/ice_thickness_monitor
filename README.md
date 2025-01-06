# ice_thickness_monitor

## Project Overview 

A device designed to monitor winter temperatures and estimate ice thickness of nearby lakes, helping prevent accidents caused by venturing onto unsafe ice surfaces.

## Current Capabilities

- Temperature Measurement: Captures temperature readings using a DHT11 temperature sensor.

- Ice Thickness Communication: Calculates ice thickness and transmits the data to a second STM32 board via UART communication protocol.

- Visual Display: Displays ice thickness levels using an LED with the following indicators:

  - Green: Safe ice thickness

  - Blue: Cautionary ice thickness

  - Red: Unsafe ice thickness

This project enhances safety by providing a clear and intuitive indication of ice stability based on real-time temperature data.
