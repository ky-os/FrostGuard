# FrostGuard: Cooling System for RNA Extracted from Breast Cancer Cell Line

The FrostGuard system is designed to cool RNA extracted from a breast cancer cell line to maintain its integrity during transportation and storage. The system uses a Peltier element connected to an Arduino board, along with a DS18B20 temperature sensor and an I2C LCD display, to maintain a temperature of 4 degrees Celsius.

## System Setup

To set up the "FrostGuard" system for cooling RNA extracted from a breast cancer cell line, follow these steps:

1. Connect the Peltier element to the BTS7960 H-bridge and the Arduino board. The H-bridge has two inputs that control the direction of the current flow and two inputs that control the current magnitude. Connect the direction inputs to digital pins on the Arduino board, and connect the current magnitude inputs to PWM pins.

2. Connect the DS18B20 temperature sensor to the Arduino board. The sensor has three pins: ground, power, and data. Connect the ground and power pins to the corresponding pins on the Arduino board, and connect the data pin to a digital pin on the board.

3. Connect the I2C LCD display to the Arduino board. The display has four pins: ground, power, SDA, and SCL. Connect the ground and power pins to the corresponding pins on the Arduino board, and connect the SDA and SCL pins to the corresponding pins on the board.

4. Upload the "FrostGuard" code to the Arduino board using the Arduino IDE.

5. Power on the system and verify that the LCD display shows the current temperature and the PID output.

## Operation

To operate the "FrostGuard" system, follow these steps:

1. Ensure that the system is set up as described in the previous section.

2. Power on the system by connecting it to a power source.

3. The system will automatically start controlling the temperature of the Peltier element to maintain a temperature of 4 degrees Celsius.

4. The LCD display will show the current temperature and the PID output, which represents the amount of power being supplied to the Peltier element.

5. To change the PID parameters, open the serial monitor in the Arduino IDE and send the following commands:

   - `setKP <value>`: Set the proportional gain of the PID controller to `<value>`.
   - `setKI <value>`: Set the integral gain of the PID controller to `<value>`.
   - `setKD <value>`: Set the derivative gain of the PID controller to `<value>`.

   Replace `<value>` with the desired value for each parameter.

6. To view the up-time of the system, type the command `uptime` in the serial monitor.

7. To stop the system, disconnect the power source.

With the "FrostGuard" system, you can be sure that the RNA extracted from a breast cancer cell line will remain cool and intact during transportation and storage, helping to preserve its quality for further research.
