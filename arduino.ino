#include <Servo.h>

// Constants for servo center positions and movement limits
const int SERVO_CENTER_X = 81;
const int SERVO_CENTER_Y = 90;
const int LIM = 9;         // Limit for servo movement
const int XLIM = 150;
const int YLIM = 150;

// Servo objects for X-axis and Y-axis
Servo s1;
Servo s2;

// PID gain constants
float kpX = 0.65;  // Proportional gain for X-axis
float kpY = 0.65;  // Proportional gain for Y-axis
float ki = 0.1;    // Integral gain
float kd = 0.06;   // Derivative gain

// PID control variables for X-axis and Y-axis
int error_priorX = 0;
float integral_X = 0;
int error_priorY = 0;
float integral_Y = 0;

// Time tracking variable for PID calculations
unsigned long lastTime = 0;  

// Function to constrain the servo position within safe limits
int constrainServoPosition(int pos) {
    return constrain(pos, 60, 110);
}

// Function to set the fail state and print error message
void setFailState(const String& errorMessage) {
    Serial.print("Error: ");
    Serial.println(errorMessage);
}

// Setup function to initialize the Arduino
void setup() {
    Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
    Serial.println("Arduino is ready");

    s1.attach(9);  // Attach servo for X-axis to pin 9
    s2.attach(10); // Attach servo for Y-axis to pin 10

    s1.write(SERVO_CENTER_X); // Center the X-axis servo
    s2.write(SERVO_CENTER_Y); // Center the Y-axis servo
    delay(5000);  // Delay for servo initialization

    lastTime = millis();  // Initialize lastTime for PID calculations
}

// Loop function to read data and control servos
void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');  // Read incoming data from serial

        if (data.startsWith("#")) {
            // Process PID gain adjustments based on received commands
            if (data.startsWith("#PX")) {
                kpX = data.substring(3).toFloat();  // Adjust proportional gain for X-axis
                Serial.print("Set P for X to ");
                Serial.println(kpX, 4);  // Print the updated gain value
            } else if (data.startsWith("#PY")) {
                kpY = data.substring(3).toFloat();  // Adjust proportional gain for Y-axis
                Serial.print("Set P for Y to ");
                Serial.println(kpY, 4);  // Print the updated gain value
            } else if (data.startsWith("#I")) {
                ki = data.substring(2).toFloat();  // Adjust integral gain
                Serial.print("Set I to ");
                Serial.println(ki, 4);  // Print the updated gain value
            } else if (data.startsWith("#D")) {
                kd = data.substring(2).toFloat();  // Adjust derivative gain
                Serial.print("Set D to ");
                Serial.println(kd, 4);  // Print the updated gain value
            }
        } else {
            // Parse the received data for servo movement
            int commaIndex = data.indexOf(',');
            if (commaIndex != -1) {
                int x = data.substring(0, commaIndex).toInt();
                int y = data.substring(commaIndex + 1).toInt();

                // Filter out small noise
                if (abs(x) < 30) x = 0;
                if (abs(y) < 30) y = 0;

                // Map and constrain the received values to the servo limits
                int mapped_X = constrain(map(x, -XLIM, XLIM, -LIM, LIM), -LIM, LIM);
                int mapped_Y = constrain(map(y, -YLIM, YLIM, -LIM, LIM), -LIM, LIM);

                // Calculate the time period for PID calculations
                unsigned long currentTime = millis();
                float period = (currentTime - lastTime) / 1000.0;  // Time period in seconds
                lastTime = currentTime;

                // Calculate the PID errors for X and Y axes
                int errorX = mapped_X;
                int errorY = mapped_Y;

                // Compute the integral term with anti-windup
                integral_X = constrain(integral_X + ki * errorX * period, -1000, 1000);
                integral_Y = constrain(integral_Y + ki * errorY * period, -1000, 1000);

                // Compute the derivative term
                float derivative_X = kd * (errorX - error_priorX) / period;
                float derivative_Y = kd * (errorY - error_priorY) / period;

                // Compute the PID output
                float outputX = kpX * errorX + integral_X + derivative_X;
                float outputY = kpY * errorY + integral_Y + derivative_Y;

                // Constrain and move the servos based on the PID output
                int posX = constrainServoPosition(SERVO_CENTER_X + outputX);
                int posY = constrainServoPosition(SERVO_CENTER_Y + outputY);

                s1.write(posX);  // Move the X-axis servo
                s2.write(posY);  // Move the Y-axis servo

                // Print the PID output values
                Serial.print("PID Output - X: ");
                Serial.print(outputX);
                Serial.print(", Y: ");
                Serial.println(outputY);

                // Update the previous errors for the next iteration
                error_priorX = errorX;
                error_priorY = errorY;

                delay(10);  // Short delay to allow for servo movement
            } else {
                setFailState("Invalid data format received.");  // Handle invalid data format
            }
        }
    }
}

