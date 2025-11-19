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
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define DABBLE_MODULE_ID_GAMEPAD  0x01
#define GAMEPAD_DATA_LENGTH       0x06

#define PI 3.14159


//Klassen für die Motorsteuerung global initialisieren
Motor motor(13, 12, 14, 27, 100, 30, 1, 25000);  // direction_change_delay = 1 (NICHT 500!)
SteeringServo steering(23, -3, 90, 20, 6);
LEDManager leftIndicator({16}, 0, 100, 1000);
LEDManager rightIndicator({5}, 0, 100, 1000);
LEDManager frontLights({19}, 1, 100, 1000);
LEDManager rearLights({18}, 1, 100, 1000);

std::vector<LEDManager*> allLeds;


// Klasse für Die Dabble App global initialisieren
BLESecurity *pSecurity = new BLESecurity();
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;

// Gamepad Datenstruktur
struct GamepadData {
  float xAxis;        // -7 bis +7
  float yAxis;        // -7 bis +7
  int radius;         // 0 bis 7
  int angle;          // 0 bis 360
  uint8_t buttons;    // Button-Status als Bitmaske
};

GamepadData gamepad = {0, 0, 0, 0, 0};
bool deviceConnected = false;

const float JOYSTICK_DEADZONE = 1.0;

/*
// BLE Callback Klasse
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t* rxValue = pCharacteristic->getData();
    size_t rxLength = pCharacteristic->getValue().length();

    if (rxLength > 0) {
      parseDabbleData(rxValue, rxLength);
    }
  }
};
*/


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
    rightIndicator.startIndicating();
    delay(100);
    rightIndicator.stopIndicating();
  }

  //mit dem links Pfeil kann man nach links Blinken
  if(PS4.Left())  {
    Serial.print("Links, ");
    leftIndicator.startIndicating();
    delay(100);
    leftIndicator.stopIndicating();
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
  if(combinedR2L2Buttons == motor.getCurrentDuty()){
    return;
  }
  
  Serial.print(" R2: ");
  Serial.print(PS4.R2Value());
  Serial.print(" L2: ");
  Serial.println(PS4.L2Value());
  motor.changeSpeedAbsolute(
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
//  --------------------------------------------------------------
//  Funktionen für Dabble BLE verbindung
//  --------------------------------------------------------------

void parseDabbleData(uint8_t* data, size_t length) {
  
  // Minimale Länge prüfen
  if (length < 4) return;
  
  
  if (data[0] == 0xFF && data[1] == DABBLE_MODULE_ID_GAMEPAD) {
    
    uint8_t dataLen = data[2];
    
    if (length >= 3 + dataLen) {
      parseGamepadData(&data[3], dataLen);
    }
  }
}

void parseGamepadData(uint8_t* data, uint8_t length) {
  

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
    motor.changeSpeedAbsolute(convertToMotorSpeed(gamepad.yAxis));
    steering.steerAbsolute(convertToSteering(gamepad.xAxis));
    
  }
}

int convertToMotorSpeed(float yValue) {
  
  // Deadzone anwenden
  if (abs(yValue) < JOYSTICK_DEADZONE) {
    return 0;  // Neutral Position (Stopp)
  }
  
  // Normalisiere Y-Wert auf -1.0 bis +1.0
  float normalized = yValue / 7.0;
  
  // Begrenze auf -1.0 bis +1.0
  normalized = constrain(normalized, -1.0, 1.0);
  
  // Konvertiere zu -100 bis +100 Bereich
  int speed = (int)(normalized * 100.0);
  
  return constrain(speed, -100, 100);
}

int convertToSteering(float xValue) {
  
  // Deadzone anwenden
  if (abs(xValue) < JOYSTICK_DEADZONE) {
    return 0;  // Neutral Position (Geradeaus)
  }
  
  // Normalisiere X-Wert auf -1.0 bis +1.0
  float normalized = xValue / 7.0;
  
  // Begrenze auf -1.0 bis +1.0
  normalized = constrain(normalized, -1.0, 1.0);
  
  // Konvertiere zu -100 bis +100 Bereich
  int steering = (int)(normalized * 100.0);
  
  return constrain(steering, -100, 100);
}

void setup() {
  // put your setup code here, to run once:
  setZero();
  config();
  setZero();
  steering.begin();

}

void config()   //Config-Klasse, hier können alle Werte angepasst werden.
{
  Serial.begin(115200);
  Serial.println("Start");

  motorSteuerungSetup();
  setZero();
  
  // Callback-Loop des PS4 Controllers Initialisieren
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  PS4.attach(onIncommingPS4Data);

  // Callback-Loop des PS4 Controllers starten
//    xTaskCreatePinnedToCore(beginPS4Connection, "PS4ControllerTask", 4096, NULL, 1, NULL, 0);
//    xTaskCreatePinnedToCore(beginBLEConnection, "BLETask", 4096, NULL, 1, NULL, 1);
//  beginBLEConnection(NULL);
  beginPS4Connection(NULL);
//  initBLE();

}

// Setup funktion für die Motorstuerung, aus Übersichtlichkeitsgründen nicht in void config()
void motorSteuerungSetup(){

  pinMode(16, OUTPUT); // Blinker links
  pinMode(19, OUTPUT); // Frontlicht
  pinMode(5, OUTPUT);  // Blinker rechts
  pinMode(18, OUTPUT); // Rücklicht
  pinMode(20, OUTPUT); // Bremslicht
  pinMode(23, OUTPUT); // Servo
  
  delay(50);
}


void setZero()
{
  Serial.println(("Null"));
  ledcWrite(12, 0);
  ledcWrite(13, 0);

  digitalWrite(14, LOW);
  digitalWrite(27, LOW);
}