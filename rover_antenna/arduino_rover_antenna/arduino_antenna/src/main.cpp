/* #include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>

void callback_toggle_led(const std_msgs::Empty& toggle_msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Empty> sub_toggle_led("toggle_LED", &callback_toggle_led,1);

void callback_toggle_led(const std_msgs::Empty& toggle_msg)
{
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
}

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  nh.initNode();
  nh.subscribe(sub_toggle_led);
  while (true)
  {
    nh.spinOnce();
    delay(10);
  }
}

void loop()
{

} */

#include <Arduino.h>
#include <ros.h>

#define POT A8

void setup(){
  pinMode(POT, INPUT);
  Serial.begin(9600);
  float previous_input = 0;
  while(true){
    // Serial.println(analogRead(POT)); //0 a 1023
    // 360/1023 => (input-previous_input)*360/1023
    float pot_input = analogRead(POT)*360/1023;
    float pos_desired = (pot_input-previous_input);
    Serial.print(analogRead(POT));
    Serial.print("  =  ");
    Serial.println(pot_input);
    // Serial.print("  =  ");
    // Serial.println(pos_desired);
    previous_input = pot_input;
  }
}

void loop(){
  
}