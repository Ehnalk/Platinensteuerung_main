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
#include <BLE2902.h>

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
Motor* motor = nullptr;
SteeringServo* steering = nullptr;
LEDManager* leftIndicator = nullptr;
LEDManager* rightIndicator = nullptr;
LEDManager* frontLights = nullptr;
LEDManager* rearLights = nullptr;


std::vector<LEDManager*> allLeds;

// Klasse für Die Dabble App global initialisieren
BLESecurity *pSecurity = new BLESecurity();
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;



void motorSteuerungSetup(){

  pinMode(16, OUTPUT); // Blinker links
  pinMode(19, OUTPUT); // Frontlicht
  pinMode(5, OUTPUT);  // Blinker rechts
  pinMode(18, OUTPUT); // Rücklicht
  pinMode(23, OUTPUT); // Servo
  
  // Motor initialisieren
  int motor_pin_front = 13;
  int motor_pin_back = 12;
  int motor_pin_dauerhigh_front = 14;
  int motor_pin_dauerhigh_back = 27;
  int motor_max_duty = 100;
  int motor_min_duty = 30;
  int motor_safety_delay = 1 * 1; 
  int motor_freq = 10000;
  motor = new Motor(motor_pin_front, motor_pin_back, motor_pin_dauerhigh_front, motor_pin_dauerhigh_back,
                     motor_max_duty, motor_min_duty, motor_safety_delay, motor_freq);

  //Servo initialisieren
  int servo_pin = 23;
  int power_pin = -1;
  int servo_rest_position = 90;
  int servo_max_steering_degree = 20; 
  int servo_deadzone = 6;
  steering = new SteeringServo(servo_pin, power_pin, servo_rest_position, servo_max_steering_degree, servo_deadzone);

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
   
  leftIndicator = new LEDManager(left_indicators, indicator_rest_state, indicator_brightness, indicator_freq);
  rightIndicator = new LEDManager(right_indicators, indicator_rest_state, indicator_brightness, indicator_freq);
  frontLights = new LEDManager(front_lights, light_rest_state, light_brightness, light_freq);
  rearLights = new LEDManager(rear_lights, light_rest_state, light_brightness, light_freq);

  motor->changeSpeedAbsolute(0);
}


void config()   //Config-Klasse, hier können alle Werte angepasst werden.
{
  setZero();
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
  delay(1000);
  motor->changeSpeedAbsolute(0);
//  initBLE();

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

/*
struct GamepadData {
  float xAxis;        // -7 bis +7
  float yAxis;        // -7 bis +7
  int radius;         // 0 bis 7
  int angle;          // 0 bis 360
  uint8_t buttons;    // Button-Status als Bitmaske
};

// Globale Variablen
GamepadData gamepad = {0, 0, 0, 0, 0};
int motorSpeed = 127;      // 0-255 (127 = Stopp)
int steeringValue = 127;   // 0-255 (127 = Mitte)
bool BLEDeviceConnected = false;

const float JOYSTICK_DEADZONE = 1.0;



// ========== BLE CALLBACK KLASSEN ==========

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLEDeviceConnected = true;
    Serial.println("✓ Dabble App verbunden!");
  }

  void onDisconnect(BLEServer* pServer) {
    BLEDeviceConnected = false;
    Serial.println("✗ Dabble App getrennt!");
    
    // Automatisch wieder Advertising starten
    delay(500);
    pServer->startAdvertising();
    Serial.println("Warte auf neue Verbindung...");
  }
};


class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      parseDabbleData((uint8_t*)rxValue.c_str(), rxValue.length());
    }
  }
};


// ========== DABBLE PROTOKOLL PARSER ==========

void parseDabbleData(uint8_t* data, size_t length) {
  
  // Minimale Länge prüfen
  if (length < 4) return;
  
  // Dabble Protokoll Format:
  // Byte 0: Start Marker (0xFF)
  // Byte 1: Module ID (0x01 für Gamepad)
  // Byte 2: Data Length
  // Byte 3+: Daten
  
  if (data[0] == 0xFF && data[1] == DABBLE_MODULE_ID_GAMEPAD) {
    
    uint8_t dataLen = data[2];
    
    if (length >= 3 + dataLen) {
      parseGamepadData(&data[3], dataLen);
    }
  }
}


void parseGamepadData(uint8_t* data, uint8_t length) {
  
  // Dabble Gamepad Datenformat (vereinfacht):
  // Byte 0: X-Achse (signed byte, -7 bis +7)
  // Byte 1: Y-Achse (signed byte, -7 bis +7)
  // Byte 2: Radius (0-7)
  // Byte 3-4: Winkel (0-360) als 16-bit
  // Byte 5+: Button Status
  
  if (length >= 6) {
    
    // X und Y Achse als signed bytes
    gamepad.xAxis = (int8_t)data[0];
    gamepad.yAxis = (int8_t)data[1];
    gamepad.radius = data[2];
    
    // Winkel als 16-bit Wert
    gamepad.angle = (data[3] << 8) | data[4];
    
    // Button Status
    gamepad.buttons = data[5];
    
    // Konvertiere zu Motor-Werten
    motorSpeed = convertToMotorSpeed(gamepad.yAxis, gamepad.radius);
    steeringValue = convertToSteering(gamepad.xAxis);
    
    // Debug Ausgabe
    printGamepadData();
  }
}


// ========== KONVERTIERUNGS-FUNKTIONEN ==========

int convertToMotorSpeed(float yValue, int radiusValue) {
  
  // Deadzone anwenden
  if (abs(yValue) < JOYSTICK_DEADZONE) {
    return 127;  // Neutral Position (Stopp)
  }
  
  // Normalisiere Y-Wert auf -1.0 bis +1.0
  float normalized = yValue / 7.0;
  
  // Begrenze auf -1.0 bis +1.0
  normalized = constrain(normalized, -1.0, 1.0);
  
  // Konvertiere zu 0-255 Bereich
  // -1.0 => 0 (Volle Rückwärts)
  // 0.0 => 127 (Stop)
  // +1.0 => 255 (Volle Vorwärts)
  int speed = (int)((normalized + 1.0) * 127.5);
  
  return constrain(speed, 0, 255);
}


int convertToSteering(float xValue) {
  
  // Deadzone anwenden
  if (abs(xValue) < JOYSTICK_DEADZONE) {
    return 127;  // Neutral Position (Geradeaus)
  }
  
  // Normalisiere X-Wert auf -1.0 bis +1.0
  float normalized = xValue / 7.0;
  
  // Begrenze auf -1.0 bis +1.0
  normalized = constrain(normalized, -1.0, 1.0);
  
  // Konvertiere zu 0-255 Bereich
  // -1.0 => 0 (Voll links)
  // 0.0 => 127 (Geradeaus)
  // +1.0 => 255 (Voll rechts)
  int steering = (int)((normalized + 1.0) * 127.5);
  
  return constrain(steering, 0, 255);
}


// ========== BUTTON FUNKTIONEN ==========

bool isButtonPressed(uint8_t buttonBit) {
  return (gamepad.buttons & (1 << buttonBit)) != 0;
}

bool isUpPressed()       { return isButtonPressed(0); }
bool isDownPressed()     { return isButtonPressed(1); }
bool isLeftPressed()     { return isButtonPressed(2); }
bool isRightPressed()    { return isButtonPressed(3); }
bool isTrianglePressed() { return isButtonPressed(4); }
bool isCirclePressed()   { return isButtonPressed(5); }
bool isCrossPressed()    { return isButtonPressed(6); }
bool isSquarePressed()   { return isButtonPressed(7); }


// ========== BLE INITIALISIERUNG ==========

void initBLE() {
  Serial.println("Initialisiere BLE für Dabble App...");
  
  // BLE Device erstellen
  BLEDevice::init("ESP32_Buggy_Gamepad");
  
  // BLE Server erstellen
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // BLE Service erstellen
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // TX Characteristic (ESP32 -> App)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // RX Characteristic (App -> ESP32)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  // Service starten
  pService->start();
  
  // Advertising starten
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("✓ BLE gestartet!");
  Serial.println("Öffne Dabble App und verbinde mit 'ESP32_Buggy_Gamepad'");
  Serial.println("Wähle dann das Gamepad Modul");
}


// ========== DEBUG AUSGABE ==========

void printGamepadData() {
  Serial.println("========================================");
  Serial.println("RAW JOYSTICK WERTE:");
  Serial.printf("  X-Achse: %.1f | Y-Achse: %.1f\n", gamepad.xAxis, gamepad.yAxis);
  Serial.printf("  Radius: %d | Winkel: %d°\n", gamepad.radius, gamepad.angle);
  
  Serial.println("\nKONVERTIERTE WERTE (0-255):");
  Serial.printf("  Motor Speed: %d | Steering: %d\n", motorSpeed, steeringValue);
  
  Serial.print("BUTTONS: ");
  if (isUpPressed()) Serial.print("UP ");
  if (isDownPressed()) Serial.print("DOWN ");
  if (isLeftPressed()) Serial.print("LEFT ");
  if (isRightPressed()) Serial.print("RIGHT ");
  if (isTrianglePressed()) Serial.print("TRIANGLE ");
  if (isCirclePressed()) Serial.print("CIRCLE ");
  if (isCrossPressed()) Serial.print("CROSS ");
  if (isSquarePressed()) Serial.print("SQUARE ");
  Serial.println();
  Serial.println("========================================\n");
}
*/

void beginPS4Connection(void *pvParameters) {
  PS4.begin("60:5b:b4:b2:90:b6");

  Serial.println("Waiting for Controller");
  Serial.println("PS4 Controller searching in Thread: ");
  Serial.println(xPortGetCoreID());
  motor->changeSpeedAbsolute(0);

}


uint8_t ignoreDataCounter = 0;

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

  if(PS4.Triangle())  {
    Serial.print("Triangle, ");
    lightAnimation(1);
  }

  if(PS4.Right()) {
    Serial.print("Rechts, ");
    rightIndicator->startIndicating();
    delay(100);
    rightIndicator->stopIndicating();
  }

  if(PS4.Left())  {
    Serial.print("Links, ");
    leftIndicator->startIndicating();
    delay(100);
    leftIndicator->stopIndicating();
  }

  //  Wenn der Rechte Joystick eine X position größer als 10 hat, bzw mehr als 10/255 oder 4% von seinem Ursprung bewegt 
  //  wird, wird gelenkt.
  if(PS4.RStickX() > 10) {  
    Serial.print(PS4.RStickX());

    // Der Rechte Joystick gibt einen Wert auf der X-Achse von 0 bis 255 aus
    steering->steerAbsolute(
                        (int)round(float((float(PS4.RStickX()) / 127.5) - 1 ) * 100)
                        );
  }

  Serial.print(" R2: ");
  Serial.print(PS4.R2Value());
  Serial.print(" L2: ");
  Serial.println(PS4.L2Value());
  if(combinedR2L2Buttons == 0 && motor->getCurrentDuty() == 0){
    return;
  }

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
  motor->changeSpeedAbsolute(0);
  setZero();

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