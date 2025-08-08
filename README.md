![Logo Institucional](https://github.com/JonatanBogadoUNLZ/PPS-Jonatan-Bogado/blob/9952aac097aca83a1aadfc26679fc7ec57369d82/LOGO%20AZUL%20HORIZONTAL%20-%20fondo%20transparente.png)

# PPS - Hydraulic Cutting Machine Automation

This repository contains the report and source code of my **Supervised Professional Practice (PPS)** carried out at **A.D. Barbieri S.A.**, Burzaco, Argentina, as part of the Mechatronics Engineering degree at the **National University of Lomas de Zamora (UNLZ)**.

## Project Description

The project consisted of the upgrade and automation of a hydraulic cutting machine for metal profiles. A system was implemented based on:
- **Microcontroller**: ESP32-WROOM-32.
- **Motors**: Two NEMA 17 stepper motors for traction and one with an encoder for positioning.
- **Sensors**: NPN inductive sensors for profile detection.
- **Interface**: ILI9341 display with menus for configuration and monitoring.
- **Software**: Programmed in the Arduino IDE with libraries such as Adafruit_ILI9341 and AccelStepper.

The system allows for configuring cutting parameters (profile type, length, quantity) and operates autonomously, with safety features such as emergency pushbuttons and sensor validation.

## Repository Structure

- **docs/**: Full PDF report and sections in Markdown.
- **src/**: Source code (`csf_mk2.ino`) and required libraries.
- **images/**: System diagrams and screenshots (if available).

## Instructions for Using the Code

1. Install the **Arduino IDE**.
2. Download the libraries listed in `src/README.md`.
3. Connect an ESP32-WROOM-32 with the pins configured according to the report (see `docs/sections/04_Project_Resolution.md`).
4. Upload the `src/c_sf_mk2.ino` file to the ESP32.

## Author
- Jonatan Bogado.
- Engineering Student – UNLZ.
- Academic Advisor: Cristian Lukaszewicz.
