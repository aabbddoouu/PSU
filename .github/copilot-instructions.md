# STM32F411 PSU Project Instructions

This is a Variable Power Supply project based on STM32F411 microcontroller with dual outputs, OLED displays, and serial communication capabilities.

## Project Architecture

### Key Components
- Dual PSU channels (PSU1 & PSU2) with independent voltage control
- OLED displays for voltage/current monitoring
- Serial communication interfaces (PC, BT)
- I2C DAC control for voltage setting
- ADC sampling for current monitoring

### Core Files
- `Core/Src/main.c`: Main control logic and state machine
- `Core/Src/dac_i2c.c`: DAC control functions
- `Core/Src/usart_pc.c`: PC communication interface
- `SSD1306/`: OLED display drivers and fonts

## Development Workflows

### Building and Flashing
```bash
make build      # Compile the project
make clean      # Clean build artifacts
make flash      # Flash compiled binary to STM32
```

### State Machine States
- `DEFAULT_STATE`: Main loop, LED heartbeat, ADC sampling
- `RX_RCV_STATE`: Handle received commands
- `SEND_DAC_STATE`: Update DAC voltage
- `UPDATE_OLED`: Refresh display values
- `TX_SEND_STATE`: Send acknowledgments

### Communication Protocol
UART message format (5 bytes):
1. Channel select ('1'/'2' for PSU1/PSU2)
2. Second voltage digit
3. First voltage digit
4. First decimal digit
5. Output state ('1' = ON, '0' = OFF)

Example: "12340" = PSU1, 23.4V, OFF

### Hardware Conventions
- ADC values use median filtering over 9 samples
- Voltage commands use millivolts internally
- Current measurements use `VREF/CURR_GAIN/ADC_RES` scaling
- OLED displays use I2C with addresses `SSD1306_I2C_ADDR` and `SSD1306_I2C_ADDR_2`

## Common Tasks
- To update voltage: Use `dac_set_voltage(channel, voltage_mV)`
- To enable/disable outputs: Use `enable_channel(&PSU)`
- To read current: Check `ADC_Table` after `start_dma_adc_convertion()`

## Error Handling
- Hardware faults trigger MCU reset via `hard_fault_handler()`
- I2C timeouts handled in `tim5_isr()`
- ADC overruns detected in `adc_isr()`