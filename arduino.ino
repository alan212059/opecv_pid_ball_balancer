#include <Servo.h>

int lim = 20;  // Limit for servo movement
int xlim = 150;
int ylim=150;


Servo s1;  // Servo for X-axis
Servo s2;  // Servo for Y-axis

float kp = 0.5;  // Proportional gain
float ki = 0.0;  // Integral gain
float kd = 0.0;  // Derivative gain

int error_priorX = 0;
int integral_X = 0;
int error_priorY = 0;
int integral_Y = 0;

void setup() {
    Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
    Serial.println("Arduino is ready");
    s1.attach(9);  // Attach servo to pin 9
    s2.attach(10);  // Attach servo to pin 10
    s1.write(81);
    s2.write(87);// Center the servo initially (adjust as needed)
    delay(5000);  // Delay for servo initializatio
    if(Serial.available()){
      String data = Serial.readStringUntil('\n'); 
      s1.write(100);
      s2.write(100);
      delay(1000);
      int commaIndex_init = data.indexOf(',');
        if (commaIndex_init != -1) {
            String x_init = data.substring(0, commaIndex_init);
            String y_init = data.substring(commaIndex_init + 1);
            Serial.println(x_init);
            Serial.println(y_init);
            
      }
      
      }
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');  // Read data from serial until newline character
        
        // Check if the command starts with #
        if (data.startsWith("#")) {
            // Process PID gain adjustments
            if (data.startsWith("#P")) {
                String p_str = data.substring(2);
                kp = p_str.toFloat();
                Serial.print("Set P to ");
                Serial.println(kp, 4);  // Print with four decimal places
            } else if (data.startsWith("#I")) {
                String i_str = data.substring(2);
                ki = i_str.toFloat();
                Serial.print("Set I to ");
                Serial.println(ki, 4);  // Print with four decimal places
            } else if (data.startsWith("#D")) {
                String d_str = data.substring(2);
                kd = d_str.toFloat();
                Serial.print("Set D to ");
                Serial.println(kd, 4);  // Print with four decimal places
            }
        } else {
            // Parse the received data for servo movement
            int commaIndex = data.indexOf(',');
            if (commaIndex != -1) {
                String x_str = data.substring(0, commaIndex);
                String y_str = data.substring(commaIndex + 1);

                int x = x_str.toInt();
                int y = y_str.toInt();
                
                if (abs(x) < 50) {
                    x = 0; // Adjust for small movements
                }
                if(abs(y)<30){
                  y=0;
                  }
                
                int mapped_X = constrain(map(x, -xlim, xlim, -lim, lim), -lim, lim);
                int mapped_Y = constrain(map(y, -ylim, ylim, -lim, lim), -lim, lim);

                int errorX = mapped_X - 0;  // Setpoint is 0 in this example
                int errorY = mapped_Y - 0;
                integral_X = integral_X + errorX;
                integral_Y = integral_Y + errorY;
                int derivative_X = errorX - error_priorX;
                int derivative_Y = errorY - error_priorY;
                
                float outputX = kp * errorX + ki * integral_X + kd * derivative_X;
                float outputY = kp * errorY + ki * integral_Y + kd * derivative_Y;
                
                int posX = 81 + outputX;  // Adjust for servo center position
                int posY = 87 + outputY;
                
                s1.write(posX);  // Move the servo based on the PID output
                s2.write(posY);
                
                delay(10);  // Small delay to stabilize the loop
                
                error_priorX = errorX;
                error_priorY = errorY;
            }
        }
    }
}
