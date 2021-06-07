#include <Servo.h>
#include <stdio.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#define STEER_CENTER 1555 // 0 deg
#define STEER_MIN 1360 // -13 deg 
#define STEER_MAX 1750 // +13 deg

#define SPEED_ZERO 1500
#define SPEED_MIN 1250
#define SPEED_MAX 1750

Servo steer_servo;
Servo dc_servo;

ros::NodeHandle nh_;

void cmd_cb(const geometry_msgs::Twist& cmd) {

  dc_servo.write(cmd.linear.x);
  steer_servo.writeMicroseconds(cmd.angular.z);
  printf("1");
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub_("cmd", cmd_cb);

void setup() {
  Serial.begin(57600);
  steer_servo.attach(3);
  dc_servo.attach(9);

  nh_.initNode();
  nh_.subscribe(cmd_sub_);
}

void loop() {
  /*static float tx_steer_;
  if(Serial.available() > 0) { // tx_throttle_
    tx_steer_ = Serial.parseFloat();
    Serial.println(tx_steer_);
    steer_servo.writeMicroseconds(tx_steer_);
    }*/
  nh_.spinOnce();
  delay(100);
}
