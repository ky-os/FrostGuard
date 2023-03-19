#include <OneWire.h>           //Library for communicating with DS18B20 temperature sensor
#include <DallasTemperature.h> //Library for reading temperature data from DS18B20; DallasTemperature by Miles Burton
#include <LiquidCrystal_I2C.h> //Library for communicating with I2C LCD; LiquidCrystal_I2C by Frank de Brabander
#include <PID_v1.h>            //Library for PID control; PID by Brett Beauregard
#include <SerialCommand.h>     //Library for serial communication with the Arduino; SerialCommand by Stefan Rado
#include <EEPROM.h>

#define ONE_WIRE_BUS 2      // Pin connected to DS18B20 data line
#define COOLING_PIN_A 3     // Pin A of H-bridge connected to cooling element
#define COOLING_PIN_B 4     // Pin B of H-bridge connected to cooling element
#define COOLING_SPEED 200   // PWM value (0-255) for controlling cooling element speed
#define TARGET_TEMP 4       // Target temperature in Celsius
#define CHECK_INTERVAL 1000 // Time between temperature checks in milliseconds
#define KP 5.0              // Proportional constant for PID control
#define KI 0.1              // Integral constant for PID control
#define KD 1.0              // Derivative constant for PID control

// Define the EEPROM addresses for the PID values
#define PID_P_ADDRESS 0
#define PID_I_ADDRESS 2
#define PID_D_ADDRESS 4

// Create a oneWire instance to communicate with DS18B20
OneWire oneWire(ONE_WIRE_BUS);
// Create a DallasTemperature instance to read temperature data from DS18B20
DallasTemperature sensors(&oneWire);
// Create a LiquidCrystal_I2C instance to communicate with I2C LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Create a SerialCommand instance to read serial commands from Arduino
SerialCommand sCmd;

// Variables for storing time information
unsigned long previousMillis = 0;
unsigned long uptimeSeconds = 0;

// Variables for PID control
double input, output, setpoint;
PID pid(&input, &output, &setpoint, KP, KI, KD, DIRECT);

void setKP()
{
  // Read the first argument (PID constant) from serial communication
  double kp = atof(sCmd.next());

  // Set the proportional constant for PID control
  pid.SetTunings(kp, KI, KD);

  Serial.print(F("Proportional constant set to: "));
  Serial.println(kp);
}

void setKI()
{
  // Read the first argument (PID constant) from serial communication
  double ki = atof(sCmd.next());

  // Set the integral constant for PID control
  pid.SetTunings(KP, ki, KD);

  Serial.print(F("Integral constant set to: "));
  Serial.println(ki);
}

void setKD()
{
  // Read the first argument (PID constant) from serial communication
  double kd = atof(sCmd.next());

  // Set the derivative constant for PID control
  pid.SetTunings(KP, KI, kd);

  Serial.print(F("Derivative constant set to: "));
  Serial.println(kd);
}

void getpidCommand()
{
  char output[50];
  snprintf(output, sizeof(output), "Current PID values: KP=%.2f, KI=%.2f, KD=%.2f", pid.GetKp(), pid.GetKi(), pid.GetKd());
  Serial.println(output);
}

// Function to save the current PID values to EEPROM
void savePID()
{
  EEPROM.put(PID_P_ADDRESS, pid.GetKp());
  EEPROM.put(PID_I_ADDRESS, pid.GetKi());
  EEPROM.put(PID_D_ADDRESS, pid.GetKd());
}

void initPID()
{

  double kp, ki, kd;

  // Read PID values from EEPROM
  EEPROM.get(PID_P_ADDRESS, kp);
  EEPROM.get(PID_I_ADDRESS, ki);
  EEPROM.get(PID_D_ADDRESS, kd);

  // If PID values are not set in EEPROM, set default values
  if (isnan(kp) || isnan(ki) || isnan(kd))
  {
    pid.SetTunings(kp, ki, kd);
    savePID(); // Save default PID values to EEPROM
  }
}

void setup()
{

  initPID();

  // Set cooling element pins as outputs
  pinMode(COOLING_PIN_A, OUTPUT);
  pinMode(COOLING_PIN_B, OUTPUT);
  // Initialize DS18B20 temperature sensor
  sensors.begin();
  // Initialize I2C LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("FrostGuard v1.0");
  // Set the target temperature for PID control
  setpoint = TARGET_TEMP;
  // Set the PID mode to automatic
  pid.SetMode(AUTOMATIC);

  // Initialize serial communication with a baud rate of 9600
  Serial.begin(9600);
  // Add commands to set PID constants via serial communication
  sCmd.addCommand("setKP", setKP);
  sCmd.addCommand("setKI", setKI);
  sCmd.addCommand("setKD", setKD);
  sCmd.addCommand("getPID", getpidCommand);
  sCmd.addCommand("savePID", savePID);

  delay(3000);
  lcd.clear();
}

void loop()
{
  // Read serial commands from Arduino
  sCmd.readSerial();

  // Record current time in milliseconds
  unsigned long currentMillis = millis();
  // Calculate elapsed time since last temperature check
  unsigned long elapsedMillis = currentMillis - previousMillis;
  previousMillis = currentMillis;
  // Update uptime in seconds
  uptimeSeconds += elapsedMillis / 1000;

  // Request temperature data from DS18B20
  sensors.requestTemperatures();
  // Read temperature data from the first sensor (there is only one in this example)
  input = sensors.getTempCByIndex(0);

  // Calculate PID output based on current temperature and target temperature
  pid.Compute();

  // Display temperature and uptime on the first line of the LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(input);
  lcd.print((char)223); // Degree symbol
  lcd.print("C   Uptime: ");
  lcd.print(uptimeSeconds);
  lcd.print("s");

  // Display PID output on the second line of the LCD
  lcd.setCursor(0, 1);
  lcd.print("PWM: ");
  lcd.print(output);
  lcd.print("  ");

  // Set cooling element speed based on PID output
  if (output >= 0)
  {
    // Cooling element is turned on in forward direction
    analogWrite(COOLING_PIN_A, COOLING_SPEED);
    analogWrite(COOLING_PIN_B, 0);
  }
  else
  {
    // Cooling element is turned on in reverse direction
    analogWrite(COOLING_PIN_A, 0);
    analogWrite(COOLING_PIN_B, COOLING_SPEED);
  }

  // Wait for the specified interval before checking temperature again
  delay(CHECK_INTERVAL);
}