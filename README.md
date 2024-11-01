# Potrov2 - ROV Control Package

Potrov2 is the control package for the Remotely Operated Underwater Vehicle (ROV) developed by Potential Robotics. It provides both simulation and hardware interfacing, allowing for comprehensive testing and deployment of control algorithms in underwater environments.

## Features

- **Simulation Environment**: Test and validate control algorithms in simulated underwater scenarios.
- **Hardware Interfacing**: Seamless integration with ROV hardware for real-world operation.
- **Control Algorithms**: Pre-built algorithms for controlling depth, orientation, and positioning.
- **Modular Design**: Easily extend and modify components for different mission needs.

## Getting Started

### Prerequisites

- **Python 3.8+**
- **ROS** (Robot Operating System)
- Necessary hardware components for real ROV deployment

### Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/potential-robotics/potrov2.git
    cd potrov2
    ```

2. Install the required Python dependencies:
    ```bash
    pip install -r requirements.txt
    ```

### Usage

#### Running the Simulator

To test control algorithms in a simulation environment, run:

```bash
python simulator.py
