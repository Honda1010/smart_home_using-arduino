#include <Arduino.h>
#include <LiquidCrystal.h> 
#include <Keypad.h>
#include <Servo.h>
#include "DHT.h"
#include<Adafruit_Sensor.h>

#define DHT_PIN 6 // The Arduino Nano pin connected to DHT11 sensor
#define DHT_TYPE DHT11

DHT dht11(DHT_PIN, DHT_TYPE);

const byte rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

Servo myservo;

const byte ROWS = 4;
const byte COLS = 3; 
char keys[ROWS][COLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'}
};

byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {4, 3, 2};
Keypad kpd = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const byte Kitchen = 7, Bathroom = A4, Gate = 5, Temperature = A6, GasSensor = A5, Fan = A7;
bool KitchenStatus = false, BathroomStatus = false, GateStatus = false;

static int TotalFailedTries = 0;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000;

bool PasswordBeingChecked();
void Control();
void PrintTemperature();
void GasDetection();
void FailedTriesCompleted();
void Status ();
void setup() {
  pinMode(Kitchen, OUTPUT);
  pinMode(Bathroom, OUTPUT);
  pinMode(Temperature, INPUT);
  pinMode(GasSensor, INPUT);
  pinMode(Fan, OUTPUT);

  myservo.attach(Gate);
  
  digitalWrite(Kitchen, LOW);
  digitalWrite(Bathroom, LOW);
  lcd.home();
  lcd.begin(16,2);
  Serial.begin(9600);
  dht11.begin(); 
  lcd.print("Home Dashboard");
  Serial.println("Home Dashboard");
  delay (750);
  lcd.clear();
}

void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  GasDetection();
  if (PasswordBeingChecked()){
    Serial.println("Controls are going manual");
    Serial.println("This is hand over to you");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("This is hand");
    lcd.setCursor(0, 1);
    lcd.print("over to you");
    delay(500);
    Control();
  }
}

bool PasswordBeingChecked(){
  Serial.println();
  Serial.println("System Lock");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Lock");
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);
  String CodeEntered = "";
  while (true){
      char key = kpd.getKey();
      if (key){
          if (key == '#' && CodeEntered == "2020"){
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Access Granted");
              Serial.println();
              Serial.println("Access Granted");
              delay(500);
              return true;
          }
          else if (key == '#' && CodeEntered == "0202"){
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Calling Police");
              Serial.println();
              Serial.println("Calling Police");
              delay(500);
              TotalFailedTries++;
              if (TotalFailedTries == 3){
                FailedTriesCompleted();
                TotalFailedTries = 0;
              }
              return false;                
          }
          else if (key == '#'){
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Unauthorized");
              Serial.println();
              Serial.println("Unauthorized");
              delay(500);
              TotalFailedTries++;
              if (TotalFailedTries == 3){
                FailedTriesCompleted();
                TotalFailedTries = 0;
              }
              return false;
          }
          else{
              lcd.print('*');
              Serial.print('*');
              CodeEntered+=key; 
          }
      }

  }
}

void Control()
{
  while (true){
  GasDetection();
  String EnteredText;
  if (Serial.available())
  {
    while (Serial.available()){EnteredText += Serial.readString();}
    if (EnteredText == "Status"){Status();}

    else if (EnteredText == "KitchenON"){
      if (KitchenStatus){
        Serial.println();
        Serial.println("Kitchen Light is already ON");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Kitchen was ON");
        delay(1000);
      }
      else{
        digitalWrite(Kitchen, HIGH);
        KitchenStatus = true;
        Serial.println();
        Serial.println("Kitchen Light is ON");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Kitchen ON");
        delay(1000);
      }
    }

    else if (EnteredText == "KitchenOFF"){
      if (KitchenStatus){
        digitalWrite(Kitchen, LOW);
        KitchenStatus = false;
        Serial.println();
        Serial.println("Kitchen Light is OFF");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Kitchen OFF");
        delay(1000);
      }
      else{
        Serial.println();
        Serial.println("Kitchen Light is already OFF");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Kitchen was OFF");
        delay(1000);
      }
    }
    
    else if (EnteredText == "BathroomON"){
      if (BathroomStatus){
        Serial.println();
        Serial.println("Bathroom Light is already ON");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Bathroom was ON");
        delay(1000);
      }
      else{
        digitalWrite(Bathroom, HIGH);
        BathroomStatus = true;
        Serial.println();
        Serial.println("Bathroom Light is ON");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Bathroom ON");
        delay(1000);
      }
    }

    else if (EnteredText == "BathroomOFF"){
      if (BathroomStatus){
        digitalWrite(Bathroom, LOW);
        BathroomStatus = false;
        Serial.println();
        Serial.println("Bathroom Light is OFF");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Bathroom OFF");
        delay(1000);
      }
      else{
        Serial.println();
        Serial.println("Bathroom Light is already OFF");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Bathroom was OFF");
        delay(1000);
      }
    }

    else if (EnteredText == "OpenGate"){
      if (GateStatus){
        Serial.println();
        Serial.println("Gate is already open");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Gate was ON");
        delay(1000);
      }
      else{
        myservo.write(180);
        GateStatus = true;
        Serial.println("Gate is Open");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Gate is Open");
        delay(1000);
      }
    }

    else if (EnteredText == "CloseGate"){
      if (GateStatus){
        myservo.write(0);
        GateStatus = false;
        Serial.println();
        Serial.println("Gate is closed");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Gate is closed");
        delay(1000);
      }
      else{
        Serial.println();
        Serial.println("Gate was already closed");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Gate was closed");
        delay(1000);
      }
    }
    else if (EnteredText == "LogOut"){
      loop();
    }
    else{
      Serial.println();
      Serial.println("Wrong Choice");
      lcd.clear();
      lcd.print("Wrong Choice");
      delay(750);
    }
  EnteredText = "";
  }
  }
}

void PrintTemperature(){
  float temp = analogRead(Temperature);
  temp *= 0.48875855327;
  if (temp < 40){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print(" C");
    Serial.print("Current Temperature is ");
    Serial.print (temp);
    Serial.println(" C");
    delay(500);
  }
  else{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print(" C");
    Serial.print("Current Temperature is ");
    Serial.print (temp);
    Serial.println(" C");
    delay(500);
  }
}
void PrintHumidty(){
  float humi  = dht11.readHumidity();
    if (isnan(humi)) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.println("Failed to read from DHT sensor!");
    Serial.println("Failed to read from DHT sensor!");
    delay(500);
  }
  else{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humidity: ");
    lcd.print(humi);
    lcd.print("%");
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");
    delay(500);
  }
}

void FailedTriesCompleted(){
  int Minutes = 10;
  int seconds = 0;
  startMillis = millis();
  while (Minutes>=0){
    GasDetection();
    currentMillis = millis();
    if (currentMillis - startMillis >= period){
      if (seconds == 0){
        Minutes--;
        seconds = 59;
      }
      else{
        seconds--;
      }
      startMillis = currentMillis;
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Try Again in:");
    Serial.print(" Try Again in:  00:");
    lcd.setCursor(1,4);
    lcd.print("   00:");
    if (Minutes < 10){
      lcd.print("0");
      lcd.print(Minutes);
      Serial.print("0");
      Serial.print(Minutes);
    }
    else{
      lcd.print(Minutes);
      Serial.print(Minutes);
    }
    lcd.print(":");
    Serial.print(":");
    if (seconds < 10){
      lcd.print("0");
      lcd.print(seconds);
      Serial.print("0");
      Serial.print(seconds);
    }
    else{
      lcd.print(seconds);
      Serial.println(seconds);
    }
    delay(500);
  }

}

void GasDetection(){
  float GasValue = analogRead(GasSensor);
  if (GasValue>300){
  // if (digitalRead(GasSensor)==HIGH){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Fire! Fire!");
  lcd.setCursor(0,1);
  lcd.print("Calling 123");
  Serial.println("Fire! Fire!");
  Serial.println("Calling 123");
  delay(1000);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
}

void Status(){
  Serial.println();
  Serial.println("Status is: ");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Status is:");
  delay(750);
  if (KitchenStatus){
    Serial.println("Kitchen Light is ON");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kitchen ON");
    delay(750);
  }
  else{
    Serial.println("Kitchen Light is OFF");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kitchen OFF");
    delay(750);
  }
  if (BathroomStatus){
    Serial.println("Bathroom Light is ON");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bathroom ON");
    delay(750);
  }
  else{
    Serial.println("Bathroom Light is OFF");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bathroom OFF");
    delay(750);
  }
  if (GateStatus){
    Serial.println("Gate is Open");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gate is Open");
    delay(750);
  }
  else{
    Serial.println("Gate is Closed");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gate is Closed");
    delay(750);
  }
  PrintTemperature();
  PrintHumidty();
}