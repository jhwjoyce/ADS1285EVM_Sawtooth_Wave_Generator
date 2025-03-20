# ADS1285EVM Sawtooth Wave Generator

## Overview
The ADS1285EVM Sawtooth Wave Generator is a Python project designed to generate a continuous sawtooth waveform using the Digital-to-Analog Converter (DAC) on the ADS1285EVM. This project is particularly useful for testing and simulating various applications that require a sawtooth waveform.

## Features
- Generates a configurable sawtooth waveform.
- Utilizes SPI communication for interfacing with the ADS1285EVM.
- Supports adjustable frequency and amplitude settings.

## Requirements
To run this project, you need the following dependencies:
- `spidev`: For SPI communication with the ADS1285EVM.
- `RPi.GPIO`: For GPIO control on Raspberry Pi.

You can install the required packages using pip:

```
pip install spidev RPi.GPIO
```

## Usage
1. Clone the repository to your local machine.
2. Navigate to the project directory.
3. Run the `sawtooth.py` script:

```
python3 sawtooth.py
```

4. The script will start generating a sawtooth wave. You can stop the generation by pressing `Ctrl+C`.

## Configuration
You can adjust the following parameters in the `sawtooth.py` file:
- `SAWTOOTH_FREQ_HZ`: Frequency of the sawtooth wave in Hertz.
- `SAWTOOTH_MIN_VAL`: Minimum DAC value.
- `SAWTOOTH_MAX_VAL`: Maximum DAC value for the 12-bit DAC.
- `STEP_SIZE`: Increment step size for the sawtooth waveform.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.