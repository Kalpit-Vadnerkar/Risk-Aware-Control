# Risk-Aware Control System (WIP)

## Overview
This project aims to develop a risk-aware control system that monitors vehicle behavior and intervenes when necessary.
Currently, this repository contains the initial setup for **Autoware** and **AWSIM Labs**.

## Setup

### Prerequisites
1.  **Autoware**: Follow the installation guide [here](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/).
2.  **AWSIM Labs**: Download the binary from [Release v1.6.1](https://github.com/autowarefoundation/AWSIM-Labs/releases/tag/v1.6.1).

### Repository Structure
- `autoware/`: Contains the Autoware source code.
- `awsim_labs_v1.6.1/`: (Ignored) Contains the AWSIM Labs simulation binary.
- `Shinjuku-Map/`: (Ignored) Contains map data.
- `Run_Autoware.sh`: Script to launch Autoware.
- `Run_AWSIM.sh`: Script to launch the AWSIM simulation.

## How to Run

### 1. Start AWSIM Simulation
In a separate terminal, run the following script to launch the simulator:
```bash
./Run_AWSIM.sh
```

### 2. Start Autoware
Run the following script to create the container and launch Autoware:
```bash
./Run_Autoware.sh
```

> **Note:** Ensure you have the necessary dependencies installed as per the Autoware documentation.