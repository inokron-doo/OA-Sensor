# SFS LoRa Sensor Node Firmware

Open-source firmware for the **SFS LoRa sensor node**. It reads distance
(ToF), temperature and humidity sensors, applies filtering, and transmits
encoded readings to the concentrator node over LoRa at 10-second intervals.

Part of the **SmartFeed Feed Monitoring System (SFS)** by **Inokron d.o.o.**,
developed under the OpenAgri EU project (SIP13).

- **Version:** v1.0.0
- **Release date:** 2026-06-17
- **Target:** STM32F051K8T7 (ARM Cortex-M0), bare metal — no RTOS
- **Language:** C (STM32 HAL, C99)
- **Licence:** [GPL-3.0](LICENSE)

## Hardware

Runs on the SFS LoRa Sensor Network Node PCB (see the
[OA-HWcomponents](https://github.com/inokron-doo/OA-HWcomponents) repository).
The same PCB runs the [concentrator firmware](https://github.com/inokron-doo/OA-Concentrator)
in the other role.


## Build and flash

Requires **STM32CubeIDE** and an **ST-Link V2** programmer.

1. Import the project into STM32CubeIDE.
2. Set the node's unique ID in `Core/Src/main.c`:
   `#define SENSOR_DEVICE_ID <1–31>` (each node on a concentrator must be unique).
3. Build the **Release** configuration.
4. Flash via ST-Link.

On startup the active sensor set is logged over USART2 (115200 baud). The node
then transmits autonomously — no user interaction required.


## Licence

GPL-3.0 — see [LICENSE](LICENSE). Derivative works must be released under the
same licence with source code made available. Bundled ST HAL/CMSIS drivers
retain their own licences.

## Contact

Inokron d.o.o. — info@inokron.com

---

*Funded by the European Union under the OpenAgri project (SIP13).*
