#include <MPU9250.h>
#include <MadgwickAHRS.h>

// IMU sensor objects
MPU9250 mpu;
Madgwick filter;

// Gyroscope calibration offsets
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// Initial pitch angle and debug flag
float angle = 45;  // Initial guess for the pitch angle
float pitch = 0.0; // Real-time pitch value
bool debug = false;

// PID controller parameters
const double Kp = 140;    // Proportional gain
const double Ki = 0.0008; // Integral gain
const double Kd = 10.1;   // Derivative gain

// Alpha for complementary filter (unused in this implementation)
const float alpha = 0.99;

// PID control variables
double setpoint = 100.0; // Target pitch value
double output = 0.0;     // PID output
double prevError = 0.0;  // Previous error for derivative calculation
double integral = 0.0;   // Accumulated integral term

// Timing variables
unsigned long lastTime = 0;
unsigned long currentTime;

// Motor control pins
const int pwmPin = 9;        // PWM output pin
const int directionPin = 6;  // Direction control pin
const int startstopPin = 12; // Motor enable/disable pin

// Function prototypes
void calibrateGyro();
void processSerialInput();
void parseCommand(String command);

// Setup function executed once at the start
void setup()
{
  // Initialize I2C communication
  Wire.begin();

  // Initialize the MPU9250 sensor
  if (!mpu.setup(0x68))
  {
    while (1)
      ; // Halt execution if the IMU cannot be initialized
  }

  // Calibrate the gyroscope
  calibrateGyro();

  // Initialize the Madgwick filter
  filter.begin(100); // Sampling rate: 100 Hz

  // Configure motor control pins
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(startstopPin, OUTPUT);

  // Enable the motor
  digitalWrite(startstopPin, HIGH);

  // Initialize serial communication for debugging
  Serial.begin(115200);
}

// Main loop executed continuously
void loop()
{
  // Update the current time
  currentTime = millis();

  // Perform IMU updates and PID calculations every 10ms (100 Hz)
  if (currentTime - lastTime >= 10)
  {
    lastTime = currentTime; // Update last execution time

    // Update MPU9250 readings
    if (mpu.update())
    {
      // Apply gyroscope calibration offsets
      float gyroX = mpu.getGyroX() - gyroXOffset;
      float gyroY = mpu.getGyroY() - gyroYOffset;
      float gyroZ = mpu.getGyroZ() - gyroZOffset;

      // Read accelerometer data
      float accX = mpu.getAccX();
      float accY = mpu.getAccY();
      float accZ = mpu.getAccZ();

      // Update Madgwick filter with IMU data
      filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

      // Retrieve the calculated pitch angle
      pitch = filter.getPitch();
    }

    // PID control logic
    double error = pitch;             // Calculate error
    double proportional = Kp * error; // Proportional term

    integral += error * 0.005; // Update integral term (5ms interval)
    double integralTerm = Ki * integral;

    double derivative = (error - prevError) / 0.005; // Derivative term
    double derivativeTerm = Kd * derivative;

    // Compute total PID output
    output = proportional + integralTerm + derivativeTerm;

    // Constrain output to motor's input range
    output = constrain(output, -250, 250);

    // Store current error for next derivative calculation
    prevError = error;

    // Control motor speed and direction based on PID output
    if (output > 0)
    {
      digitalWrite(directionPin, HIGH); // Set direction to reverse
      analogWrite(pwmPin, 255 - int(abs(output)));
    }
    else
    {
      digitalWrite(directionPin, LOW); // Set direction to forward
      analogWrite(pwmPin, 255 - int(abs(output)));
    }
  }

  // Process serial input for real-time PID tuning
  if (Serial.available())
  {
    processSerialInput();
  }
}

// Function to handle serial input for PID parameter tuning
void processSerialInput()
{
  static String inputString = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    if (inChar == '\n')
    {
      parseCommand(inputString);
      inputString = ""; // Clear the input buffer
    }
    else
    {
      inputString += inChar; // Append character to the input string
    }
  }
}

// Parse and execute tuning commands
void parseCommand(String command)
{
  if (command.startsWith("Kp "))
  {
    Kp = command.substring(3).toFloat();
    Serial.print("Kp updated to: ");
    Serial.println(Kp);
  }
  else if (command.startsWith("Ki "))
  {
    Ki = command.substring(3).toFloat();
    Serial.print("Ki updated to: ");
    Serial.println(Ki);
  }
  else if (command.startsWith("Kd "))
  {
    Kd = command.substring(3).toFloat();
    Serial.print("Kd updated to: ");
    Serial.println(Kd);
  }
  else
  {
    Serial.println("Unknown command. Use Kp, Ki, or Kd followed by a value.");
  }
}

// Calibrate the gyroscope by averaging readings over a set number of samples
void calibrateGyro()
{
  const int numSamples = 2000; // Number of samples to average
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++)
  {
    mpu.update();
    sumX += mpu.getGyroX();
    sumY += mpu.getGyroY();
    sumZ += mpu.getGyroZ();
    delay(1); // Small delay to stabilize readings
  }

  // Calculate average offsets
  gyroXOffset = sumX / numSamples;
  gyroYOffset = sumY / numSamples;
  gyroZOffset = sumZ / numSamples;

  Serial.println("Gyroscope calibrated!");
  Serial.print("Offsets - X: ");
  Serial.print(gyroXOffset);
  Serial.print(", Y: ");
  Serial.print(gyroYOffset);
  Serial.print(", Z: ");
  Serial.println(gyroZOffset);
}
