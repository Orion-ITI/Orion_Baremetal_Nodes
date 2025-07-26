# Orion_Baremetal_Nodes

## Overview
This repository contains firmware projects for automotive Electronic Control Units (ECUs) implemented on STM32F103C8T6 "Blue Pill" development boards, running **FreeRTOS** for real-time task management. The projects include firmware for an **AC Control ECU**, **Seat Angle Control ECU**, and **Blinker Control ECU**, all connected via a **Controller Area Network (CAN)** for communication. Each ECU leverages the STM32 Blue Pill's Cortex-M3 processor and FreeRTOS to handle real-time control tasks efficiently.

## Project Structure
The repository is organized into two main directories:
- **Example**: Contains three test example projects to demonstrate basic functionality on the STM32 Blue Pill.
- **Features**: Contains three projects, each implementing the core logic for one ECU (AC Control, Seat Angle Control, Blinker Control), including FreeRTOS porting and CAN communication.