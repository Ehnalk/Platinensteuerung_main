#include <BuggyControl.h>
#include <LEDManager.h>
#include <Motor.h>
#include <SteeringServo.h>

#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>

#include <dummy.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <BLESecurity.h>
#include <Ticker.h>
#include <vector>
#include <algorithm>
#include <string>
#include <math.h>


// UUID's für die Dabble app, es gibt eine library, diese wird jedoch nichtmehr gepflegt
#define SERVICE_UUID         "06b298f7-60df-4f9c-9cee-4d7c72dd1863"
#define CHARACTERISTIC_UUID  "d136d35f-a5d8-4ac6-986a-3f7aa9025484"

#define DABBLE_MODULE_ID_GAMEPAD  0x01
#define GAMEPAD_DATA_LENGTH       0x06

#define PI 3.14159


//Klassen für die Motorsteuerung global initialisieren
Motor motor(0, 0,0, 0, 0, 0, 0, 0);

SteeringServo steering(0, 0, 0, 0);

LEDManager leftIndicator({0}, 0, 0, 0);
LEDManager rightIndicator({0}, 0, 0, 0);
LEDManager frontLights({0}, 0, 0, 0);
LEDManager rearLights({0}, 0, 0, 0);

std::vector<LEDManager*> allLeds = {&leftIndicator, &rightIndicator, &frontLights, &rearLights};

// Klasse für Die Dabble App global initialisieren
BLESecurity *pSecurity = new BLESecurity();



void motorSteuerungSetup(){

  pinMode(16, OUTPUT); // Blinker links
  pinMode(19, OUTPUT); // Frontlicht
  pinMode(5, OUTPUT);  // Blinker rechts
  pinMode(18, OUTPUT); // Rücklicht
  pinMode(23, OUTPUT); // Servo
  
  // Motor initialisieren
  int motor_pin_front = 13;
  int motor_pin_back = 12;
  int motor_pin_dauerhigh_front = 14 ;
  int motor_pin_dauerhigh_back = 27;
  int motor_max_duty = 100;
  int motor_min_duty = 30;
  int motor_safety_delay = 1 * 1000; 
  int motor_freq = 10000;
  motor = Motor(motor_pin_front, motor_pin_back, motor_pin_dauerhigh_front, motor_pin_dauerhigh_back, motor_max_duty, motor_min_duty, motor_safety_delay, motor_freq);

  //Servo initialisieren
  int servo_pin = 23;
  int servo_rest_position = 90;
  int servo_max_steering_degree = 20; 
  int servo_deadzone = 6;
  steering = SteeringServo(servo_pin, servo_rest_position, servo_max_steering_degree, servo_deadzone);

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
   
  leftIndicator = LEDManager(left_indicators, indicator_rest_state, indicator_brightness, indicator_freq);
  rightIndicator = LEDManager(right_indicators, indicator_rest_state, indicator_brightness, indicator_freq);
  frontLights = LEDManager(front_lights, light_rest_state, light_brightness, light_freq);
  rearLights = LEDManager(rear_lights, light_rest_state, light_brightness, light_freq);

}
/*
class AppCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    delay(10);
    String rxValue =  pCharacteristic->getValue();
    const uint8_t* data = (const uint8_t*)rxValue.data();
    if(rxValue.length() >= 3)  {

      uint16_t xAxis = (rxValue[2] <<8) | (uint8_t)rxValue[1];
      uint16_t yAxis = (rxValue[4] <<8) | (uint8_t)rxValue[3];
      uint8_t buttons = (uint8_t)rxValue[4];


      Serial.printf("Joystick: x=%d, y=%d\n");

      Serial.println(" Wert von Handy empfangen:");
      Serial.print(sum);
      oldsum = sum + 40;
      Serial.println(oldsum);
      sum = 0;

    }

  }
};
*/


void config()   //Config-Klasse, hier können alle Werte angepasst werden.
{

  Serial.begin(115200);
  Serial.println("Start");

  motorSteuerungSetup();

  // Callback-Loop des PS4 Controllers Initialisieren
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  PS4.attach(onIncommingPS4Data);

  // Callback-Loop des PS4 Controllers starten
//  xTaskCreatePinnedToCore(beginPS4Connection, "PS4ControllerTask", 4096, NULL, 1, NULL, 0);
//  xTaskCreatePinnedToCore(beginBLEConnection, "BLETask", 4096, NULL, 1, NULL, 1);
//  beginBLEConnection(NULL);
  beginPS4Connection(NULL);

}


void lightAnimation(int blink_amount)   //Animation, die abgespielt werden kann. Schaltet die LEDs zweimal an und aus und kehrt dann auf rest zurück
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


void beginPS4Connection(void *pvParameters) {
  PS4.begin("60:5b:b4:b2:90:b6");
//  for (;;)  {
    Serial.println("Waiting for Controller");
    Serial.println("PS4 Controller searching in Thread: ");
    Serial.println(xPortGetCoreID());
//    vTaskDelay(200 / portTICK_PERIOD_MS);
//  }

}

/*
void beginBLEConnection(void *pvParameters) {
  Serial.println("Starting BLE Connecton.");
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_NO_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  BLEDevice::init("Buggy von der Ausbildung");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, 
                                                                      BLECharacteristic::PROPERTY_READ | 
                                                                      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new AppCallbacks());
  pService->start();

  /*  Funktionen zum Einstellen der BLE (Bluetooth low energey) verbindung
      Die angegeben funktionen sind die standarts, also einfach so lassen außer man braucht was spezielles
  *//*

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();        // Der esp wird nun unter Bluetooth verbindungen angezeigt und kann gekoppelt werden

    Serial.println("Waiting for BLE App");
    Serial.println("BLE searching in Thread: ");
    Serial.println(xPortGetCoreID());
//    vTaskDelay(200 / portTICK_PERIOD_MS);
  
}
*/

void onIncommingPS4Data() { 
  delay(10);
/*  Serial.printf("RX: %4d | R2: %4d | L2: %4d\n",
              PS4.RStickX(),
              PS4.R2Value(),
              PS4.L2Value());
*/
  Serial.print("Reciveing Data: ");
  if(PS4.Triangle())  {
    Serial.print("Triangle, ");
    lightAnimation(1);
  }

  if(PS4.Right()) {
    Serial.print("Rechts, ");
    rightIndicator.startIndicating();
    delay(100);
    rightIndicator.stopIndicating();
  }

  if(PS4.Left())  {
    Serial.print("Links, ");
    leftIndicator.startIndicating();
    delay(100);
    leftIndicator.stopIndicating();
  }

  //  Wenn der Rechte Joystick eine X position größer als 10 hat, bzw mehr als 10/255 oder 4% von seinem Ursprung bewegt 
  //  wird, wird gelenkt.
  if(PS4.RStickX() > 10) {  

    Serial.print(PS4.RStickX());

    // Der Rechte Joystick gibt einen Wert auf der X-Achse von 0 bis 255 aus
    steering.steerAbsolut(
                        (int)round(float((float(PS4.RStickX()) / 127.5) - 1 ) * 100)
                        );
  }

  Serial.print(" R2: ");
  Serial.print(PS4.R2Value());

  //
  motor.changeSpeedAbsolut(
                          (int)round(float((float(PS4.R2Value() - PS4.L2Value()) / 127)) * 100)
                          );


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
  config();
}

void loop() {
  // put your main code here, to run repeatedly:
}
