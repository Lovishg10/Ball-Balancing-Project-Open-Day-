#include <Arduino.h>
#include "InverseKinematics.h"
#include "MotorControl.h"
#include "Screen.h"
#include "PIDControllers.h"

// Global variable declaration
bool enable_serial_output = true;

//Movement pattern mode
int currentMode = 0 ;//0= Balance, 1 = Point, 2 = Line, 3 = Ellipse, 4 = Square, 5 = Figure8, 6 = Spiral, 7 = Star, 8 = Heart
double p1 = 0, p2 = 0, p3 = 0; // Parameters for setMode

// Function definitions
void muteAllSerialOutput() {
  enable_serial_output = false;
}

void enableAllSerialOutput() {
  enable_serial_output = true;
}


void setup() {
  // Initialization functions
  Serial.begin(115200);
  muteAllSerialOutput();
  if (enable_serial_output) Serial.println("----------------NEW RUN-----------------"); 
  screen_init();
  motor_init();
  home_motors();
  go_home();
  delay(1000);

  // Headers for CSV file reading
  //Serial.println("Time(ms),Ball_X(mm),Ball_Y(mm),Target_X(mm),Target_Y(mm)");
}

void checkSerial() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the whole line
    if (command.length() < 2) return;

    char type = command.charAt(0);
    char subtype = command.charAt(1);
    float value = command.substring(1).toFloat(); // For single letter commands
    float value2 = command.substring(2).toFloat(); // For two letter commands

    switch (type) {
      case 'P': kp = value; Serial.print("Kp: "); break;
      case 'I': ki = value; Serial.print("Ki: "); break;
      case 'D': kd = value; Serial.print("Kd: "); break;
      
      case 'A': // Adjusted Gains Prefix (e.g., "AP0.5")
        if (subtype == 'P') { kp_adj = value2; Serial.print("Adj Kp: "); }
        if (subtype == 'I') { ki_adj = value2; Serial.print("Adj Ki: "); }
        if (subtype == 'D') { kd_adj = value2; Serial.print("Adj Kd: "); }
        if (subtype == 'T') { adj_threshold = value2; Serial.print("Adj Threshold: "); }
        break;

      case 'L': // Limits (e.g., "LI60" for Integral Limit)
        if (subtype == 'I') { integ_limit = value2; Serial.print("Integ Limit: "); }
        if (subtype == 'M') { max_angle = value2; Serial.print("Max Angle: "); }
        break;
      
      case 'M': {
        // String looks like "M3,40,20"
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);

        // Get the Mode (number between 'M' and first comma)
        currentMode = command.substring(1, firstComma).toInt();
    
        // Get Parameter 1 (Side/Radius)
        if (firstComma != -1) p1 = command.substring(firstComma + 1, secondComma).toFloat();
    
        // Get Parameter 2 (Speed)
        if (secondComma != -1) p2 = command.substring(secondComma + 1, thirdComma).toFloat();

        //Get Parameter 3 (Speed for some)
        if (thirdComma != -1) p3 = command.substring(thirdComma+1).toFloat();

        Serial.print("Running Mode "); Serial.print(currentMode);
        Serial.print(" with P1="); Serial.print(p1);
        Serial.print(", P2="); Serial.println(p2);
        Serial.print(", P3="); Serial.println(p3);
        break;
      }
    }
    Serial.println(value2 > 0 ? value2 : value);
  }
}

void loop() {
  checkSerial();

 switch (currentMode) {
    case 0:
      pid_balance(0, 0); // Standard balancing at center
      break;
    case 1:
      move_to_point(p1, p2, 1); // Executes 1 to_point cycles
      break;
    case 2:
      move_line(p1, p2, p3, 1); // Executes 1 line cycles
      break;
    case 3:
      move_ellipse(p1, p2, p3, 1); // Executes 1 ellipse cycles
      break;
    case 4:
      move_square(p1, p2, 1); // Executes 1 square cycles
      break;
    case 5:
      move_figure8(p1, p2, 1); //Executes 1 figure8 cycles
      break;
    case 6:
      move_spiral(p1, p2, 1); // Executes 1 spiral cycles
      break;
    case 7:
      move_star(p1, p2, 1); // Executes 1 star cycles
      break;
    case 8:
      move_heart(p1, p2, 1); // Executes 1 heart cycles
      break;
 
  }
}
 