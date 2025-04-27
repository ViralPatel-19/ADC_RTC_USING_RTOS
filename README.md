
---

# ADC and RTC Integration Using RTOS

## Overview

This project demonstrates the integration of Analog-to-Digital Converter (ADC) readings with a Real-Time Clock (RTC) module in a real-time operating system (RTOS) environment. The system reads analog voltage values, timestamps them using the RTC, and displays the information on an OLED screen. The implementation is done in C and is compatible with Infineon's PSoC™ microcontrollers using the ModusToolbox™ development environment.

## Features

- **ADC Reading**: Continuously reads analog voltage values from a specified channel.
- **RTC Integration**: Retrieves the current time and date from the RTC module.
- **OLED Display**: Displays the voltage readings along with their corresponding timestamps on a 0.96-inch OLED screen using the SSD1306 driver.
- **RTOS Implementation**: Utilizes FreeRTOS to manage tasks for ADC reading, RTC communication, and OLED display updates.
- **Modular Code Structure**: Organized code with separate files for OLED handling, RTC commands, and main application logic.

## Project Structure

```
ADC_RTC_USING_RTOS/
├── main.c             // Main application code
├── rtc_commands/
│   ├── rtc_commands.c // RTC command implementations
│   └── rtc_commands.h // RTC command definitions
├── ssd1306.c          // OLED display driver implementation
├── ssd1306.h          // OLED display driver header
├── ssd1306_conf.h     // OLED display configuration
├── ssd1306_fonts.c    // Font data for OLED display
├── ssd1306_fonts.h    // Font header for OLED display
├── ssd1306_port.c     // Hardware abstraction layer for OLED
├── ssd1306_port.h     // Header for hardware abstraction
└── README.md          // Project documentation
```

## Requirements

- **Hardware**:
  - Infineon PSoC™ microcontroller development board (e.g., CY8CKIT-062-BLE)
  - 0.96-inch OLED display module with SSD1306 driver
  - DS1307 RTC module connected via I2C
  - Analog voltage source (e.g., potentiometer or sensor)
  - USB cable for programming and power

- **Software**:
  - [ModusToolbox™](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/) installed on your development machine
  - Serial terminal application (e.g., PuTTY, Tera Term) for UART communication (optional)

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/ViralPatel-19/ADC_RTC_USING_RTOS.git
```

### 2. Open the Project in ModusToolbox™

- Launch **ModusToolbox™**.
- Click on **"Import Application"**.
- Navigate to the cloned repository and select the project.

### 3. Configure the ADC, I2C, and RTOS Components

- Open the Device Configurator in ModusToolbox™.
- **ADC Configuration**:
  - Add and configure an ADC channel to read the analog voltage.
  - Assign the analog input pin connected to your voltage source.
- **I2C Configuration**:
  - Add and configure an I2C master component to communicate with the RTC and OLED display.
  - Assign the appropriate SDA and SCL pins connected to the RTC and OLED.
- **RTOS Configuration**:
  - Enable FreeRTOS and configure tasks for ADC reading, RTC communication, and OLED display updates.
- Save and generate the configuration.

### 4. Build and Program

- Build the project using the ModusToolbox™ IDE.
- Connect your development board via USB.
- Program the board using the **"Program"** option.

### 5. Observe the OLED Display

- After programming, the OLED display should show the current voltage reading along with the corresponding timestamp.
- Adjust the input voltage to see real-time updates on the display.

## Understanding the Code

- **main.c**: Initializes the RTOS, creates tasks for ADC reading, RTC communication, and OLED display updates.
- **rtc_commands.c / rtc_commands.h**: Implements functions to communicate with the DS1307 RTC module over I2C, including reading and setting time and date.
- **ssd1306.c / ssd1306.h**: Implements functions to control the SSD1306 OLED display, including initialization, clearing the screen, setting cursor positions, and displaying text.
- **ssd1306_conf.h**: Contains configuration settings for the OLED display, such as screen dimensions and I2C address.
- **ssd1306_fonts.c / ssd1306_fonts.h**: Provides font data and definitions used for displaying characters on the OLED.
- **ssd1306_port.c / ssd1306_port.h**: Abstracts the hardware-specific I2C communication functions, allowing the OLED driver to interface with the microcontroller's I2C peripheral.

## Customization

- **Multiple ADC Channels**: Extend the project to read multiple analog inputs and display their values.
- **Data Logging**: Implement functionality to log the voltage readings and timestamps to an external storage device or transmit them via UART.
- **Alarm Functionality**: Add features to trigger alarms or notifications based on specific voltage thresholds or time events.

## License

This project is open-source and available for educational and personal development use. Please refer to the `LICENSE` file for more information.

## Acknowledgments

- **Developed by**: Viral Patel
- **Tools Used**: ModusToolbox™, Infineon PSoC™ microcontrollers.

Feel free to contribute to this project by submitting issues or pull requests. Your feedback and improvements are welcome!

--- 
