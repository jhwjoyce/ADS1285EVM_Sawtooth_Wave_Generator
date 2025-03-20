#!/usr/bin/env python3
"""
ADS1285EVM Sawtooth Wave Generator

This Python script generates a sawtooth wave using the DAC on the ADS1285EVM.
It configures the required registers and creates a continuous sawtooth
waveform with configurable frequency and amplitude.

Requirements:
- spidev (for SPI communication)
- RPi.GPIO (Raspberry Pi as host)
"""

import time
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("RPi.GPIO module not found. Ensure you are running on a Raspberry Pi.")
    print("Using Mock.GPIO for simulation purposes.")
    import Mock.GPIO as GPIO  # Use a mock GPIO library for non-RPi environments
    from unittest.mock import MagicMock
import spidev
from threading import Thread
import signal
import sys

# Configuration parameters
SAWTOOTH_FREQ_HZ = 100    # Frequency of sawtooth wave in Hz
SAWTOOTH_MIN_VAL = 0      # Minimum DAC value
SAWTOOTH_MAX_VAL = 4095   # Maximum DAC value for 12-bit DAC
STEP_SIZE = 10            # Increment step size for sawtooth

# GPIO pin definitions (adjust according to your connections)
CS_PIN = 8               # Chip select pin for SPI
RESET_PIN = 10           # Reset pin for ADS1285EVM
START_PIN = 12           # START pin for ADS1285

# SPI configuration
SPI_BUS = 0              # SPI bus number
SPI_DEVICE = 0           # SPI device number
SPI_SPEED = 1000000      # SPI speed in Hz (1 MHz)

class ADS1285EVMController:
    """Controller class for the ADS1285EVM"""
    
    def __init__(self):
        """Initialize the ADS1285EVM controller"""
        # Initialize SPI
        self.spi = getSpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        self.spi.mode = 1  # SPI mode 1 (CPOL=0, CPHA=1)
        
        # Initialize GPIO
        if is_gpio_module_imported():
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(CS_PIN, GPIO.OUT)
            GPIO.setup(RESET_PIN, GPIO.OUT)
            GPIO.setup(START_PIN, GPIO.OUT)
            
            # Set initial pin states
            GPIO.output(CS_PIN, GPIO.HIGH)     # CS inactive
            GPIO.output(RESET_PIN, GPIO.HIGH)  # Reset inactive
            GPIO.output(START_PIN, GPIO.LOW)   # START inactive
            
        # Current DAC value
        self.current_dac_value = SAWTOOTH_MIN_VAL
        
        # Flag for running state
        self.running = False
        
        # Reset the device
        self.reset_device()
        
        # Initialize the DAC
        self.init_dac()
    
    def reset_device(self):
        """Reset the ADS1285EVM"""
        if is_gpio_module_imported():
            GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.1)  # 100ms reset pulse
        if is_gpio_module_imported():
            GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(0.1)  # Wait for device to stabilize
    
    def init_dac(self):
        """Initialize the DAC on the ADS1285EVM"""
        # Send initialization commands to DAC
        # Example: Set power mode, reference, etc.
        self.write_register(0x01, 0x10)  # Example: Power control register
        self.write_register(0x02, 0x20)  # Example: Configuration register
    
    def write_register(self, addr, value):
        """Write to a register on the ADS1285"""
        if is_gpio_module_imported():
            GPIO.output(CS_PIN, GPIO.LOW)  # Assert CS
        
        # Send command byte (write command: 0x5X where X is the register address)
        self.spi.xfer2([0x50 | (addr & 0x0F)])
        
        # Send data byte
        self.spi.xfer2([value])
        
        if is_gpio_module_imported():
            GPIO.output(CS_PIN, GPIO.HIGH)  # Deassert CS
    
    def write_dac(self, value):
        """Write value to the DAC"""
        # For a 12-bit DAC, format the data for SPI transfer
        msb = (value >> 8) & 0xFF  # Extract MSB (bits 8-15)
        lsb = value & 0xFF         # Extract LSB (bits 0-7)
        
        if is_gpio_module_imported():
            GPIO.output(CS_PIN, GPIO.LOW)  # Assert CS
        
        # Send command byte for DAC write (assumed to be 0x70)
        # Then send 16-bit data (MSB first)
        self.spi.xfer2([0x70, msb, lsb])
        
        if is_gpio_module_imported():
            GPIO.output(CS_PIN, GPIO.HIGH)  # Deassert CS
    
    def generate_sawtooth(self):
        """Generate a single sawtooth cycle"""
        # Increment or reset DAC value to create sawtooth pattern
        if self.current_dac_value >= SAWTOOTH_MAX_VAL:
            # Reset to minimum when we reach maximum (creates sawtooth falling edge)
            self.current_dac_value = SAWTOOTH_MIN_VAL
        else:
            # Otherwise increment by step size
            self.current_dac_value += STEP_SIZE
        
        # Write value to DAC
        self.write_dac(self.current_dac_value)
        print(f"DAC Value: {self.current_dac_value}")  # Debug output
    
    def start_sawtooth_wave(self):
        """Start generating a continuous sawtooth wave"""
        self.running = True
        
        # Calculate delay between updates based on frequency
        delay = 1.0 / (SAWTOOTH_FREQ_HZ * (SAWTOOTH_MAX_VAL / STEP_SIZE))
        
        # Start the waveform generation thread
        self.wave_thread = Thread(target=self._generate_continuous_sawtooth, args=(delay,))
        self.wave_thread.daemon = True
        self.wave_thread.start()
        
        print(f"Sawtooth wave started at {SAWTOOTH_FREQ_HZ} Hz")
    
    def _generate_continuous_sawtooth(self, delay):
        """Thread function for continuous sawtooth generation"""
        while self.running:
            self.generate_sawtooth()
            time.sleep(delay)
    
    def stop_sawtooth_wave(self):
        """Stop the sawtooth wave generation"""
        self.running = False
        if hasattr(self, 'wave_thread'):
            self.wave_thread.join(timeout=1.0)
        print("Sawtooth wave stopped")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_sawtooth_wave()
        self.spi.close()
        GPIO.cleanup()

def signal_handler(sig, frame):
    """Handle Ctrl+C for clean shutdown"""
    print("\nShutting down...")
    if 'controller' in globals():
        controller.cleanup()
    sys.exit(0)

def is_gpio_module_imported():
    return 'GPIO' in sys.modules

# Ensure spidev is available for testing on non-RPi systems
def getSpiDev():
    if 'Mock' in sys.modules:
        # Create a mock spidev object
        mock_spi = MagicMock(spec=spidev.SpiDev)

        # replace SpiDev with the mock
        spidev.SpiDev = MagicMock(return_value=mock_spi)
    return spidev.SpiDev()

if __name__ == "__main__":
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Create ADS1285EVM controller
        controller = ADS1285EVMController()
        
        # Start sawtooth wave generation
        controller.start_sawtooth_wave()
        
        # Keep the main thread alive
        print("Press Ctrl+C to stop the sawtooth wave")
        while True:
            time.sleep(1)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Ensure cleanup happens
        if 'controller' in globals():
            controller.cleanup()