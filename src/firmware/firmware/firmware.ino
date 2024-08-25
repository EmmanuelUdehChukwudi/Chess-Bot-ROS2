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
Servo gripper_servo;

char received_chars[32];
int char_pos = 0;

void setup() {
  waist_servo.attach(WAIST_PIN);
  shoulder_servo.attach(SHOULDER_PIN);
  elbow_servo.attach(ELBOW_PIN);
  wrist_servo.attach(WRIST_PIN);
  wrist_twist_servo.attach(WRIST_TWIST_PIN);
  gripper_twist_servo.attach(GRIPPER_TWIST_PIN);
  gripper_servo.attach(GRIPPER_PIN);
 
  waist_servo.write(0); 
  shoulder_servo.write(0); 
  elbow_servo.write(0);
  wrist_servo.write(0);
  wrist_twist_servo.write(0);
  gripper_twist_servo.write(0);
  gripper_servo.write(35);
  Serial.begin(115200);
  Serial.setTimeout(1);
  delay(3000);
}

void loop() {
  while (Serial.available()) {
    char chr = Serial.read();
    if (chr == ',') {
      received_chars[char_pos] = '\0';
      // Process the command
      process_command(received_chars);
      char_pos = 0;
    } else {
      received_chars[char_pos] = chr;
      char_pos++;
      if (char_pos >= sizeof(received_chars) - 1) {
        char_pos = 0;  
      }
    }
  }
}

void process_command(const char *cmd) {
  char command_type = cmd[0];
  int angle = atoi(&cmd[1]);

  if (angle >= 0 && angle <= 180) { 
    switch (command_type) {
      case 'b': reach_goal(waist_servo, angle); break;
      case 's': reach_goal(shoulder_servo, angle); break;
      case 'e': reach_goal(elbow_servo, angle); break;
      case 'w': reach_goal(wrist_servo, angle); break;
      case 't': reach_goal(wrist_twist_servo, angle); break;
      case 'j': reach_goal(gripper_twist_servo, angle); break;
      case 'g': reach_goal(gripper_servo, angle); break;
      default: break; 
    }
  }
}

void reach_goal(Servo& motor, int goal) {
  int currentPos = motor.read();
  if (goal >= currentPos) {
    for (int pos = currentPos; pos <= goal; pos += 1) {
      motor.write(pos);
      delayMicroseconds(1);
    }
  } else {
    for (int pos = currentPos; pos >= goal; pos -= 1) {
      motor.write(pos);
      delayMicroseconds(1);
    }
  }
}
