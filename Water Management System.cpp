#include "DFRobot_PH.h"
#include <EEPROM.h>
#include "GravityTDS.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#define CAYENNE_DEBUG       // Uncomment to show debug messages
#define CAYENNE_PRINT Serial  // Comment this out to disable prints and save space
#include <CayenneMQTTESP8266Shield.h>

//for Ultrasonic Sensor
int trigPin = 5;
int echoPin = 4;
 int duration, distance, height;

//pH sensor
#define PH_PIN A1
float voltage, phValue, temperature = 25;
DFRobot_PH ph;

//Temperature. sensor
#define ONE_WIRE_BUS 3
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 float Celcius=0;
 float Fahrenheit=0;
 float temp;

 //for turbidity
 float turb;

// WiFi network info.
char ssid[] = "Moyin";
char wifiPassword[] = "1234567890";

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "fa41ee80-ea45-11eb-b767-3f1a8f1211ba";
char password[] = "5b1b558fc19b1e79aa845d1d032ae8d07b303b94";
char clientID[] = "beca9890-f3e9-11eb-883c-638d8ce4c23d";

// Set ESP8266 Serial object. In this example we use the Serial1 hardware serial which is available on boards like the Arduino Mega.
#define EspSerial Serial1
ESP8266 wifi(&EspSerial);

#define VIRTUAL_CHANNEL 7
#define ACTUATOR_PIN 9
int buttonSense = 11;
 int pump = 6;
 
 // constants won't change. Used here to set a pin number:
const int ledPin =  8;// the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000; 

//for flow meter
int flowPin = 2;
double flowRate;
volatile int count;
bool pumpState = 0;
int flow;


void setup() {
     ph.begin();
  sensors.begin();
   pinMode(flowPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), Flow, RISING);
  pinMode(pump, OUTPUT);
   pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(ACTUATOR_PIN, OUTPUT);
   pinMode(buttonSense, INPUT_PULLUP);

  digitalWrite(pump, LOW);

  Serial.begin(9600);
  delay(10);
  EspSerial.begin(9600);
  delay(10); 
   Cayenne.begin(username, password, clientID, wifi, ssid, wifiPassword);
}


void blinkk(){
   unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}

void uSensor(){
    digitalWrite (trigPin, HIGH);
  delayMicroseconds (1000);
  digitalWrite (trigPin, LOW);
  duration = pulseIn (echoPin, HIGH);
  distance = (duration/2) / 29.1;   
  distance = map(distance, 2.00, 15.00, 100.00, 0.00);
distance = constrain(distance, 0.00, 100.00);
}

void turbidity() {
int sensorValue = analogRead(A2);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); 
  turb = voltage; 
 // Serial.println(turb);
}

void flowMeter() {
  // put your main code here, to run repeatedly:
  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second
  noInterrupts(); //Disable the interrupts on the Arduino

  //Start the math
  //Take counted pulses in the last second and multiply by 2.25mL
  flowRate = (count * 2.25);
  //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate * 60;
  //Convert mL to Liters, giving you Liters / Minute
  flowRate = flowRate / 1000;
  flow = (int)(flowRate);
}

void pHSensor() {
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {            //time interval: 1s
    timepoint = millis();
    //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
    voltage = analogRead(PH_PIN) / 1024.0 * 5000; // read the voltage
    phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
  }
  ph.calibration(voltage, temperature);          // calibration process by Serail CMD
}




void loop() {
   Cayenne.loop();
   uSensor();
  turbidity();
  pHSensor();
  blinkk();
  flowMeter();

 sensors.requestTemperatures(); 
Celcius=sensors.getTempCByIndex(0);
Fahrenheit=sensors.toFahrenheit(Celcius);
 temp = Celcius;

int sensorVal = digitalRead(buttonSense);
 if (sensorVal == LOW) {
 if(distance <= 7){
    digitalWrite(pump, HIGH);
  //  pumpState = 1;
  }

   if(distance >= 87){
    digitalWrite(pump, LOW);
   // pumpState = 0;
  } 
 }

if (sensorVal == HIGH){
    Serial.println("Manual Override");
  } 

  //Serial.println(String(phValue) +  "," + String(temp) + "," + String(turb) + "," + String(distance));
  //delay(1000);
} 

void Flow() {
  //Every time this function is called, increment "count" by 1
  count++;
}

CAYENNE_OUT(1)
{
    Cayenne.virtualWrite(1,phValue);
    Cayenne.virtualWrite(2,turb); 
    Cayenne.virtualWrite(3,temp); 
    Cayenne.virtualWrite(4,distance); 
    Cayenne.virtualWrite(5,flowRate); 
    
}

CAYENNE_IN(VIRTUAL_CHANNEL){
  CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());
  int value = getValue.asInt();
  Serial.println(value);
 digitalWrite(ACTUATOR_PIN, value);
  
  if( value == 1){
  digitalWrite(pump, HIGH); 
}
if(value != 1){
  digitalWrite(pump, LOW); 
  }
  
  }