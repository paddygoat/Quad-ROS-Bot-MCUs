#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

//Set up the ros node and publisher
std_msgs::Float32 throttle_msg;
std_msgs::Float32 ArduinoGPSx_msg;
std_msgs::Float32 ArduinoGPSy_msg;
std_msgs::Float32 act_steer_ang_msg;
std_msgs::Float32 throttControl_msg;

ros::Publisher pub_throttle("throttle", &throttle_msg);
ros::Publisher pub_ArduinoGPSx("ArduinoGPSx", &ArduinoGPSx_msg);
ros::Publisher pub_ArduinoGPSy("ArduinoGPSy", &ArduinoGPSy_msg);
ros::Publisher pub_act_steer_ang("act_steer_ang", &act_steer_ang_msg);
ros::NodeHandle nh;

// Testing:
// roscore
// rostopic echo temperature

const int ledPin1 =  13;
const int ledPin0 =  12;
int ledState = LOW;
unsigned long prevMillis_01 = 0;
unsigned long prevMillis_02 = 0;
unsigned long prevMillis_03 = 0;
const long interval_01 = 50;
const long interval_02 = 250;
const long interval_03 = 250;

int ledState_02 = LOW;
int ledState_03 = LOW;

int sensorPin_1 = A1;  // Steering control
int sensorPin_2 = A2;  // Throttle
int sensorPin_3 = A3;  // Not used
int sensorPin_4 = A4;  // Not used
int sensorPin_5 = A5;  // Actual mechanical steering angle

int steerControlVal = 0;  // Steering control
int throttleVal = 0;  // Throttle
int sensorValue_3 = 0;  // Not used
int sensorValue_4 = 0;  // Not used
int sensorValue_5 = 0;  // Actual mechanical steering angle

int speed = 0;

/////////////////////////////////////////////////////////////////////////////////////////////
// Subscriber call back setup:
void messageCb1( const std_msgs::Float32 dogx1_msg )
{
  ArduinoGPSx_msg = dogx1_msg;
  // blinkLED_12();
}
ros::Subscriber<std_msgs::Float32> sub1("dogx1_msg", &messageCb1 );

void messageCb2( const std_msgs::Float32 dogy1_msg )
{
  ArduinoGPSy_msg = dogy1_msg;
  // blinkLED_13();
}
ros::Subscriber<std_msgs::Float32> sub2("dogy1_msg", &messageCb2 );

void messageCb3( const std_msgs::Float32 throttControl_msg )
{
  blinkLED_12();
  speed = (int)throttControl_msg.data;
}
ros::Subscriber<std_msgs::Float32> sub3("throttControl_msg", &messageCb3 );

////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  nh.initNode();

//////////////////////////////////////////////
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
/////////////////////////////////////////////

  nh.advertise(pub_throttle);
  nh.advertise(pub_ArduinoGPSx);
  nh.advertise(pub_ArduinoGPSy);
  nh.advertise(pub_act_steer_ang);
  
  // Serial.begin(9600);
  pinMode(ledPin0, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(11, OUTPUT);        // THROTTLE PWM

  digitalWrite(ledPin0, HIGH);  // LED 12
  digitalWrite(ledPin1, HIGH);  // LED 13
  delay(1000);
  digitalWrite(ledPin0, LOW);
  digitalWrite(ledPin1, LOW);
}

void loop() 
{
  int temperature = 47;
  unsigned long currMillis = millis();

  if (currMillis - prevMillis_01 >= interval_01) 
  {
    prevMillis_01 = currMillis;

    steerControlVal = analogRead(sensorPin_1);  // Steering control
    throttleVal = analogRead(sensorPin_2);  // Throttle
    sensorValue_3 = analogRead(sensorPin_3);  // Not used
    sensorValue_4 = analogRead(sensorPin_4);  // Not used
    sensorValue_5 = analogRead(sensorPin_5);  // Actual steering angle. 575 to 408 to 276. Centre is 408.

    // Serial.print("steerControlVal: ");Serial.println(steerControlVal);
    // Serial.print("throttleVal: ");Serial.println(throttleVal);
    // Serial.print("sensorValue_3: ");Serial.println(sensorValue_3);
    // Serial.print("sensorValue_4: ");Serial.println(sensorValue_4);
    // Serial.println("");

    // STEERING CONTROL IS ON ANALOG 1:
    if ((steerControlVal > 300)&&(steerControlVal < 750)) // Deadspot between 300 and 750.
    {
      digitalWrite(49, LOW);
      delay(50);
      digitalWrite(51, LOW);    // Stop the actuator
    }

    if (steerControlVal > 750)
    {
      digitalWrite(49, HIGH);    // Turn right
      delay(50);
      digitalWrite(51, HIGH);    // Move the actuator
    }

    if (steerControlVal < 300)
    {
      digitalWrite(49, LOW);     // Turn left
      delay(50);
      digitalWrite(51, HIGH);    // Move the actuator
    }

    // THROTTLE IS ON ANALOG 2. Mid point on joystick is about = 488.
    if(throttleVal < 480)
    {
      blinkLED_13();  // Blink LED 13 at 2Hz.
      speed = (450 - throttleVal) /3.5;
      if (speed < 0)
      {
        speed = 0;
      }
    }

    analogWrite(11, speed);
    // Serial.print("speed: ");Serial.println(speed);

    throttle_msg.data = speed;
    act_steer_ang_msg.data = sensorValue_5;  // Actual mechanical steering angle

    pub_throttle.publish(&throttle_msg);
    pub_ArduinoGPSx.publish(&ArduinoGPSx_msg);
    pub_ArduinoGPSy.publish(&ArduinoGPSy_msg);
    pub_act_steer_ang.publish(&act_steer_ang_msg);
  }
  delay(1);
  nh.spinOnce();
}

void blinkLED_13() 
{
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis_03 >= interval_03) 
  {
    prevMillis_03 = currentMillis;
    if (ledState_03 == LOW) {
      ledState_03 = HIGH;
    } else {
      ledState_03 = LOW;
    }
    digitalWrite(ledPin1, ledState_03);
  }
}

void blinkLED_12() 
{
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis_02 >= interval_02) 
  {
    prevMillis_02 = currentMillis;
    if (ledState_02 == LOW) {
      ledState_02 = HIGH;
    } else {
      ledState_02 = LOW;
    }
    digitalWrite(ledPin0, ledState_02);
  }
}
