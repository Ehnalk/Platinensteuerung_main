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

#include <esp_task_wdt.h>

const char* mac = "60:5b:b4:b2:90:b6"

//Klassen für die Motorsteuerung global initialisieren
Motor motor(13, 12, 14, 27, 100, 30, 1, 25000);  // direction_change_delay = 1 (NICHT 500!)
SteeringServo steering(23, -3, 90, 20, 6);
LEDManager leftIndicator({16}, 0, 100, 1000);
LEDManager rightIndicator({5}, 0, 100, 1000);
LEDManager frontLights({19}, 1, 100, 1000);
LEDManager rearLights({18}, 1, 100, 1000);
LEDManager brakeLights({4}, 1, 100, 1000);


std::vector<LEDManager*> allLeds;


// Alle Klassen der Motor Steuerug setzen.
void motorSteuerungSetup(){
  // Alle Objekte sind bereits global erstellt!

  // pinMode() setzen (wie in TestServoInit)
  pinMode(13, OUTPUT);  // Motor vorwärts
  pinMode(12, OUTPUT);  // Motor rückwärts
  pinMode(16, OUTPUT);  // Blinker links
  pinMode(5, OUTPUT);   // Blinker rechts
  pinMode(19, OUTPUT);  // Frontlicht
  pinMode(18, OUTPUT);  // Rücklicht
  pinMode(23, OUTPUT);  // Servo

  delay(50);  // Kurze Pause damit Pin-Konfiguration wirksam wird

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
  motor.changeSpeedAbsolute(0);

}

// Variable um nur jeden dritten R2/L2 Wert zu verwerten, optimierungsversuch
uint8_t ignoreDataCounter = 0;

// Funktion die Bei eingehenden PS4 Daten aufgerufen wird, diese funktion ist auch für die verarbeitung 
// /verwertung der Daten zuständig.
void onIncommingPS4Data() { 


  if(ignoreDataCounter == 3){
    ignoreDataCounter = 0;
    return;
  }
  if(ignoreDataCounter != 0){
    ignoreDataCounter++;
    return;
  }
  ignoreDataCounter++;

  int8_t combinedR2L2Buttons = PS4.R2Value() + PS4.L2Value();
  int8_t R2L2_in_percentage = (int)round(float((float(PS4.R2Value() - PS4.L2Value()) / 127)) * 100);
  int8_t RX_percentage = (int)round(float((float(PS4.RStickX()) / 127 ) * 100));

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
  if(RX_percentage != steering.getCurrentSteeringPercent()) {

    if(abs(PS4.RStickX()) > 10 ){
    
    
      Serial.println(PS4.RStickX());

      // Der Rechte Joystick gibt einen Wert auf der X-Achse von 0 bis 255 aus
      steering.steerAbsolute(RX_percentage);
    }else {
      steering.steerAbsolute(0);
      Serial.println(0);
    }
  }

  // Wenn der eingehende R2/L2 reduntant ist, wird das programm hier beendet.
  if(R2L2_in_percentage == motor.getCurrentDuty()){
    return;
  }
  
  Serial.print(" R2: ");
  Serial.print(PS4.R2Value());
  Serial.print(" L2: ");
  Serial.println(PS4.L2Value());
  motor.changeSpeedAbsolute(R2L2_in_percentage);


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
  
  config();

  // KRITISCH: steering.begin() MUSS in setup() aufgerufen werden!
  // Nicht in einer Unterfunktion!
  steering.begin();
  motor.changeSpeedAbsolute(0);
  // setZero() wird bereits in config() aufgerufen
  esp_task_wdt_deinit();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

void setZero()
{
  Serial.println(("Null"));
  ledcWrite(12, 0);
  ledcWrite(13, 0);

  digitalWrite(14, LOW);
  digitalWrite(27, LOW);
}