# Gotu_Obstacle_Avoidance_Robot
Gotu is an Arduino-based Obstacle Avoidance Robot that uses ultrasonic sensors for obstacle detection and clap-based commands for movement and speed control. The system is designed to autonomously navigate its environment, detect obstacles, and respond to user inputs through sound commands.

Project Overview :
The Gotu Obstacle Avoidance Robot integrates hardware components such as ultrasonic sensors, a motor driver, DC motors, and a servo motor to enable smooth and dynamic movement. The project also features a clap detection circuit to allow user commands like starting, stopping, and adjusting speed. Real-time processing is carried out using an Arduino Uno microcontroller, which makes quick decisions for navigation.

Features:
Obstacle Avoidance:
Detects obstacles using the HC-SR04 ultrasonic sensor and navigates in a direction with a clear path.
Clap Detection for Control:
1 clap → Start
2 claps → Stop
Additional 1 clap → Adjust speed (slow or fast).
Real-time Decision Making:
Processes sensor data to determine the direction of movement.
Speed Control:
Adjusts the speed dynamically based on commands and proximity to obstacles.
Servo Motor Rotation:
Enables the ultrasonic sensor to rotate and achieve a wider obstacle detection range.
Reverse Alert:
A buzzer activates when the robot moves in reverse.
Hardware Components
Arduino Uno: Microcontroller to process sensor data and control motors.
HC-SR04 Ultrasonic Sensor: Measures distance to obstacles.
Servo Motor: Rotates the ultrasonic sensor for wider detection coverage.
L293D Motor Driver Shield: Drives the DC motors for motion.
DC Motors: Enable the movement of the robot chassis.
Clap Detection Circuit: Detects sound signals (claps) to toggle commands.
Buzzer: Provides alerts during reverse motion.
Power Supply: Two 9V batteries (one for Arduino and one for motors).
Chassis: Houses all components and provides physical structure.
Software Development
The robot was programmed using the Arduino IDE with the following libraries:

AFMotor.h: For controlling DC motors via the motor driver shield.
Servo.h: For rotating the servo motor.
NewPing.h: For interfacing with the ultrasonic sensor.
Logic and Functionality
The ultrasonic sensor measures the distance to obstacles.
The Arduino processes the sensor data and decides the robot's direction (forward, left, right, or stop).
Clap detection triggers commands for start, stop, and speed adjustments.
Servo motor rotates the ultrasonic sensor for a broader detection range.
How It Works
Obstacle Detection:
The HC-SR04 sensor emits ultrasonic waves.
The time taken for the echo to return is processed to calculate the distance.
Decision Making:
If an obstacle is detected within a certain range, the robot stops and decides to turn left or right.
Clap Control:
The sound sensor detects claps, and predefined actions (start, stop, or speed control) are executed.
Servo Motor:
The servo motor rotates the ultrasonic sensor to scan the surroundings and determine the obstacle-free direction.
Arduino Code
cpp
Copy code
#include <AFMotor.h>       // Motor control library
#include <Servo.h>         // Servo motor library
#include <NewPing.h>       // Ultrasonic sensor library

#define TRIG 7            // Trigger pin for HC-SR04
#define ECHO 6            // Echo pin for HC-SR04
#define MAX_DISTANCE 200  // Maximum distance for detection (in cm)

AF_DCMotor motor1(1);     // Left motor
AF_DCMotor motor2(2);     // Right motor
Servo myServo;

NewPing sonar(TRIG, ECHO, MAX_DISTANCE);
int distance;

void setup() {
  Serial.begin(9600);
  motor1.setSpeed(200);   // Set initial motor speed
  motor2.setSpeed(200);
  myServo.attach(9);      // Servo connected to pin 9
  myServo.write(90);      // Center position
}

void loop() {
  distance = sonar.ping_cm();  // Get distance
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < 20) {
    stopMotors();        // Stop if an obstacle is near
    delay(1000);
    turnRight();         // Turn right to avoid obstacle
  } else {
    moveForward();       // Move forward if no obstacle
  }
}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  delay(500);
  stopMotors();
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}
Applications
Autonomous Navigation: Used in robotics for navigating dynamic environments.
Industrial Automation: Material movement in controlled areas.
Educational Projects: Teaching robotics, Arduino programming, and sensor-based navigation.
Future Scope
Bluetooth Integration: Control the robot remotely via a mobile app.
AI-based Person Following: Advanced tracking capabilities using AI algorithms.
Enhanced Obstacle Detection: Using LiDAR or multiple ultrasonic sensors for better navigation.
Conclusion
The Gotu Obstacle Avoidance Robot demonstrates autonomous navigation capabilities through the integration of sensors and sound-based commands. This project showcases the practical applications of Arduino programming, motor control, and real-time decision-making, serving as a stepping stone for advanced robotics projects.

