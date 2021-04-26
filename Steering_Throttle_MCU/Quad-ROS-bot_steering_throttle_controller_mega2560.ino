#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

//Set up the ros node and publisher
std_msgs::Float32 throttle_msg;
std_msgs::Float32 ArduinoGPSx_msg;
std_msgs::Float32 ArduinoGPSy_msg;
std_msgs::Float32 act_steer_ang_msg;

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
const long interval_01 = 50;

int sensorPin_4 = A4;
int sensorPin_1 = A1;
int sensorPin_2 = A2;
int sensorPin_3 = A3;
int sensorPin_5 = A5;    // Actual steering angle.

int sensorValue_4 = 0;
int sensorValue_1 = 0;
int sensorValue_2 = 0;
int sensorValue_3 = 0;
int sensorValue_5 = 0;

int speed = 0;

void messageCb( const std_msgs::Float32 dogx1_msg , const std_msgs::Float32 dogy1_msg)
{
  digitalWrite(12, HIGH-digitalRead(12));   // blink the led 12
  ArduinoGPSx_msg = dogx1_msg;
  ArduinoGPSy_msg = dogy1_msg;
}
ros::Subscriber<std_msgs::Float32> sub0("dogx1_msg", &messageCb );
ros::Subscriber<std_msgs::Float32> sub1("dogy1_msg", &messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub0);
  nh.subscribe(sub1);
  nh.advertise(pub_throttle);
  nh.advertise(pub_ArduinoGPSx);
  nh.advertise(pub_ArduinoGPSy);
  nh.advertise(pub_act_steer_ang);
  
  // Serial.begin(9600);
  pinMode(ledPin0, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(11, OUTPUT);        // THROTTLE PWM

  digitalWrite(ledPin0, HIGH);
  digitalWrite(ledPin1, HIGH);
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


    sensorValue_1 = analogRead(sensorPin_1);
    sensorValue_2 = analogRead(sensorPin_2);
    sensorValue_3 = analogRead(sensorPin_3);
    sensorValue_4 = analogRead(sensorPin_4);
    sensorValue_5 = analogRead(sensorPin_5);   // Actual steering angle. 575 to 408 to 276. Centre is 408.

 
    // Serial.print("sensorValue_1: ");Serial.println(sensorValue_1);
    // Serial.print("sensorValue_2: ");Serial.println(sensorValue_2);
    // Serial.print("sensorValue_3: ");Serial.println(sensorValue_3);
    // Serial.print("sensorValue_4: ");Serial.println(sensorValue_4);
    // Serial.println("");

    // STEERING IS ON ANALOG 1:
    if ((sensorValue_1 > 300)&&(sensorValue_1 < 750))
    {
      digitalWrite(49, LOW);
      delay(50);
      digitalWrite(51, LOW);    // Stop the actuator
    }

    if (sensorValue_1 > 750)
    {
      digitalWrite(49, HIGH);    // Turn right
      delay(50);
      digitalWrite(51, HIGH);    // Move the actuator
    }

    if (sensorValue_1 < 300)
    {
      digitalWrite(49, LOW);     // Turn left
      delay(50);
      digitalWrite(51, HIGH);    // Move the actuator
    }

    // THROTTLE IS ON ANALOG 2:

    speed = (450 - (sensorValue_2)) /4;
    if (speed < 0)
    {
      speed = 0;
    }
    
    analogWrite(11, speed);
    // Serial.print("speed: ");Serial.println(speed);

    
    //if (ledState == LOW) 
    //{
    //  ledState = HIGH;
    //} else 
    //{
    //  ledState = LOW;
    //}
    // digitalWrite(ledPin1, ledState);

    throttle_msg.data = speed;
    act_steer_ang_msg.data = sensorValue_5;
    pub_throttle.publish(&throttle_msg);
    pub_ArduinoGPSx.publish(&ArduinoGPSx_msg);
    pub_ArduinoGPSy.publish(&ArduinoGPSy_msg);
    pub_act_steer_ang.publish(&act_steer_ang_msg);
  }
  delay(1);
  nh.spinOnce();
}
