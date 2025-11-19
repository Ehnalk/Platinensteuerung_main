#include <BuggyControl.h>
#include <LEDManager.h>
#include <Motor.h>
#include <SteeringServo.h>

#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>

#include <dummy.h>

#include <ESP32Servo.h>
#include <Arduino.h>
#include <BLESecurity.h>
#include <Ticker.h>
#include <vector>
#include <algorithm>
#include <string>
#include <math.h>

const char* mac = "60:5b:b4:b2:90:b6"

//Klassen für die Motorsteuerung global initialisieren
Motor* motor = nullptr;
SteeringServo steering;
LEDManager* leftIndicator = nullptr;
LEDManager* rightIndicator = nullptr;
LEDManager* frontLights = nullptr;
LEDManager* rearLights = nullptr;
LEDManager* bremslicht = nullptr;


std::vector<LEDManager*> allLeds;


// Alle Klassen der Motor Steuerug setzen.
void motorSteuerungSetup(){

  pinMode(16, OUTPUT); // Blinker links
  pinMode(19, OUTPUT); // Frontlicht
  pinMode(5, OUTPUT);  // Blinker rechts
  pinMode(18, OUTPUT); // Rücklicht
  pinMode(20, OUTPUT); // Bremslicht
  pinMode(23, OUTPUT); // Servo
  
  // Motor initialisieren
  int motor_pin_front = 13;
  int motor_pin_back = 12;
  int motor_pin_dauerhigh_front = 14;
  int motor_pin_dauerhigh_back = 27;
  int motor_max_duty = 100;
  int motor_min_duty = 30;
  int motor_safety_delay = 1 * 1; 
  int motor_freq = 100000;
  motor = new Motor(motor_pin_front, motor_pin_back, motor_pin_dauerhigh_front, motor_pin_dauerhigh_back,
                     motor_max_duty, motor_min_duty, motor_safety_delay, motor_freq);

  setZero();

  //Servo initialisieren
  int servo_pin = 23;
  int power_pin = -1;
  int servo_rest_position = 90;
  int servo_max_steering_degree = 20; 
  int servo_deadzone = 6;
  steering = SteeringServo(servo_pin, power_pin, servo_rest_position, servo_max_steering_degree, servo_deadzone);

  //LEDs initialisieren
  int indicator_brightness = 100;
  int light_brightness = 100;

  int indicator_freq = 1000;
  int light_freq = 1000;

  int indicator_rest_state = 0;
  int light_rest_state = 1;

  std::vector<int> left_indicators = {16};
  std::vector<int> right_indicators = {5};
  std::vector<int> front_lights = {19};
  std::vector<int> rear_lights = {18};
  std::vector<int> bremslicht = {20}
   
  leftIndicator = new LEDManager(left_indicators, indicator_rest_state, indicator_brightness, indicator_freq);
  rightIndicator = new LEDManager(right_indicators, indicator_rest_state, indicator_brightness, indicator_freq);
  frontLights = new LEDManager(front_lights, light_rest_state, light_brightness, light_freq);
  rearLights = new LEDManager(rear_lights, light_rest_state, light_brightness, light_freq);
  bremslicht = new LEDManager(bremslicht, light_rest_state, light_brightness, light_freq);
}


void config()   //Config-Klasse, hier können alle Werte angepasst werden.
{
  Serial.begin(115200);
  Serial.println("Start");

  motorSteuerungSetup();  //  Setup der Motorsteuerung
  setZero();

  // Callback-Loop des PS4 Controllers Initialisieren
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  PS4.attach(onIncommingPS4Data);

  // Callback-Loop des PS4 Controllers starten
  beginPS4Connection(NULL);

}

//  Licht Animationsfunktion, aka eine funktion um die LED an und aus zu machen für 'blink_amount' mal.
//  verwendung von Delays kann evtl blocking sein.
void lightAnimation(int blink_amount)  
{ 
  for (int j = 0; j < blink_amount; j++) {
  
    for(LEDManager* i : allLeds)
    {
      i->turnOn(100);
    }
    delay(500);
    for(LEDManager* i : allLeds)
    {
      i->turnOff();
    }
      delay(500);

  }

  delay(500);
  for(LEDManager* i : allLeds)
  {
    i->rest();
  }
}


// Beginnt die Bluetooth verbindung mit dem PS4 Controller bzw die Bluetooth suche.
// Die Paramater '*pvParamaeters* ist für eine Spätere verwendung von Multithreading.
void beginPS4Connection(void *pvParameters) {
  PS4.begin("60:5b:b4:b2:90:b6");

  Serial.println("Waiting for Controller");
  Serial.println("PS4 Controller searching in Thread: ");
  Serial.println(xPortGetCoreID());

}

// Variable um nur jeden dritten R2/L2 Wert zu verwerten, optimierungsversuch
uint8_t ignoreDataCounter = 0;

// Funktion die Bei eingehenden PS4 Daten aufgerufen wird, diese funktion ist auch für die verarbeitung 
// /verwertung der Daten zuständig.
void onIncommingPS4Data() { 

  int8_t combinedR2L2Buttons = PS4.R2Value() + PS4.L2Value();

  if(ignoreDataCounter == 3){
    ignoreDataCounter = 0;
    return;
  }
  if(ignoreDataCounter != 0){
    ignoreDataCounter++;
    return;
  }
  ignoreDataCounter++;

  //  Auf Dreieck soll man kann man alle Lichter an machen können.
  if(PS4.Triangle())  {
    Serial.print("Triangle, ");
    lightAnimation(1);
  }

  // Mit dem rechts Pfeil kann man nach rechts Blinken
  if(PS4.Right()) {
    Serial.print("Rechts, ");
    rightIndicator->startIndicating();
    delay(100);
    rightIndicator->stopIndicating();
  }

  //mit dem links Pfeil kann man nach links Blinken
  if(PS4.Left())  {
    Serial.print("Links, ");
    leftIndicator->startIndicating();
    delay(100);
    leftIndicator->stopIndicating();
  }

  // Der Input des Rechten Joystick ist -127 zu 127.
  if(PS4.RStickX() != steering.getCurrentSteeringDegree()) {  

    // Convertieren zu %
    int8_t r_Joystick_x_Value = (int)round(float(float(PS4.RStickX()) / 127) * 100);
    Serial.print("Lenken: ");
    Serial.println(r_Joystick_x_Value);

    // Der Servo Klasse wird ein Wert von -100% bis 100% gegeben.
    steering.steerAbsolute(r_Joystick_x_Value);
  }

  // Wenn der eingehende R2/L2 reduntant ist, wird das programm hier beendet.
  if(combinedR2L2Buttons == motor->getCurrentDuty()){
    return;
  }
  
  Serial.print(" R2: ");
  Serial.print(PS4.R2Value());
  Serial.print(" L2: ");
  Serial.println(PS4.L2Value());
  motor->changeSpeedAbsolute(
                          (int)round(float((float(PS4.R2Value() - PS4.L2Value()) / 127)) * 100));


}


void onConnect()  {
  Serial.println("Controller Connected!");
  lightAnimation(2);
}


void onDisconnect() {
  Serial.println("Controller Disconnected!");
  lightAnimation(1);
} 


void setup() {
  // put your setup code here, to run once:
  setZero();
  config();
  setZero();
  steering.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
}

void setZero()
{
  Serial.println(("Null"));
  ledcWrite(12, 0);
  ledcWrite(13, 0);

  digitalWrite(14, LOW);
  digitalWrite(27, LOW);
}