#include <Servo.h>

#define WAIST_PIN 3
#define SHOULDER_PIN 5
#define ELBOW_PIN 6
#define WRIST_PIN 9
#define WRIST_TWIST_PIN 10
#define GRIPPER_TWIST_PIN 11
#define GRIPPER_PIN 12

Servo waist_servo;
Servo shoulder_servo;
Servo elbow_servo;
Servo wrist_servo;
Servo wrist_twist_servo;
Servo gripper_twist_servo;


void setup() {
  waist_servo.attach(WAIST_PIN);
  shoulder_servo.attach(SHOULDER_PIN);
  elbow_servo.attach(ELBOW_PIN);
  wrist_servo.attach(WRIST_PIN);
  wrist_twist_servo.attach(WRIST_TWIST_PIN);
  gripper_twist_servo.attach(GRIPPER_TWIST_PIN);
  
  waist_servo.write(0); 
  shoulder_servo.write(0); 
  elbow_servo.write(0);
  wrist_servo.write(0);
  wrist_twist_servo.write(0);
  gripper_twist_servo.write(0);
  
}

void loop() {

}
