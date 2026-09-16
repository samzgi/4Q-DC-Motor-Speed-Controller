# 4Q DC Motor Speed Controller

A four-quadrant DC motor speed controller developed as an application project for **Power Electronics I** at Ege University.

The project combines an H-bridge power stage, MOSFET gate driving, STM32-based digital control, current sensing and protection, LTspice simulation, and experimental hardware implementation.

## Project Overview

The controller is designed for bidirectional speed and torque control of a DC motor through four-quadrant operation:

- Forward motoring
- Forward braking
- Reverse motoring
- Reverse braking

The system uses an H-bridge topology to control motor voltage and current in both directions. PWM-based control is implemented on an STM32 microcontroller.

## Main Features

- **Four-quadrant DC motor control**
- H-bridge power stage
- **IRF540N** power MOSFETs
- **IR2104** high/low-side gate driver
- **21 kHz PWM** switching frequency
- STM32-based digital control
- Dead-time implementation for shoot-through prevention
- **ACS712** current sensing
- **1 A current-limit protection**
- Soft-start functionality
- Encoder-based speed feedback
- OLED display for system information
- CW/CCW direction control
- Automatic direction-change operation
- LTspice power-stage simulation
- Physical prototype and experimental testing

## System Architecture

```text
                 +----------------------+
                 |      STM32 MCU       |
                 | PWM / Control Logic  |
                 | Speed Feedback       |
                 | Protection           |
                 +----------+-----------+
                            |
                       PWM + Direction
                            |
                 +----------v-----------+
                 |    IR2104 Drivers    |
                 +----------+-----------+
                            |
                 +----------v-----------+
                 |      H-Bridge        |
                 |   IRF540N MOSFETs    |
                 +----------+-----------+
                            |
                       DC Motor
                            |
                 +----------v-----------+
                 |   ACS712 / Encoder   |
                 | Current / Speed FB   |
                 +----------------------+
```

## Design Workflow

1. Analytical design of the four-quadrant converter
2. Selection of the power MOSFETs and gate-driver circuit
3. LTspice simulation and verification
4. STM32 control firmware development
5. Current sensing and protection implementation
6. Hardware assembly and prototype testing
7. Experimental verification of controller operation

## Hardware

| Component | Role |
|---|---|
| STM32 | Digital controller and PWM generation |
| IRF540N | H-bridge power MOSFETs |
| IR2104 | High/low-side MOSFET gate driver |
| ACS712 | Motor current measurement |
| Encoder | Motor speed feedback |
| OLED | User/system information display |
| DC Motor | Controlled load |

## Control and Protection

The STM32 generates the PWM signals required to operate the H-bridge. Dead-time is incorporated into the switching sequence to reduce the risk of simultaneous conduction of the high- and low-side MOSFETs.

An ACS712 current-sensing stage monitors motor current. The controller incorporates current limiting and soft-start functionality to improve operating safety during startup and load changes.

The controller also supports clockwise/counter-clockwise operation and automatic direction changes.

## Simulation

The power stage and relevant electrical behavior were investigated using **LTspice** before hardware implementation. Simulation was used to examine switching and motor-control behavior and to support component and circuit design decisions.

## Repository Structure

```text
4Q-DC-Motor-Speed-Controller/
├── README.md
├── documentation/
│   └── Project_Report_Gharehbagh_Zoghi.pdf
├── firmware/
│   └── STM32/
├── simulation/
│   └── LTspice/
└── figures/
```

## Academic Context

**Course:** Power Electronics I  
**Institution:** Ege University  
**Project:** Design of 4Q DC Motor Speed Controller  
**Authors:** Sam Zoghi, Amirkia Mahd Gharehbagh

## Technologies & Tools

- STM32
- Embedded C
- LTspice
- Power Electronics
- PWM Motor Control
- H-Bridge Converters
- Current Sensing
- Encoder Feedback

## Repository Status

This repository documents an academic power-electronics project, including the design approach, simulation work, embedded control, and hardware implementation.

> The repository is being organized to separate documentation, firmware, simulation files, and supporting project figures.