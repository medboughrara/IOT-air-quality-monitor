#include <Arduino.h>
#include <WiFi.h>
//#include <Wire.h>//essential to read data from MH_Z19C (sensor)
#include <PMS5003.h>
#include <LiquidCrystal.h>

#define PMS_RX_PIN 12
#define PMS_TX_PIN 13
#define MQ_2_PIN A0   
/*  //more specific sensors
#define MH_Z19C_ADDRESS 0x68   //Read CO2 concentration
#define MQ_135_ADDRESS A0      //Read NO2 concentration
#define MQ_7_ADDRESS A1        // Read CO concentration
#define MQ_137_ADDRESS A2      // Read SO2 concentration
*/
#define BUZZER_PIN 6
#define LCD_RS_PIN 5
#define LCD_EN_PIN 4
#define LCD_D4_PIN 3
#define LCD_D5_PIN 2
#define LCD_D6_PIN A7
#define LCD_D7_PIN A6
#define LED_green 1
#define LED_red 1
/*
// Conversion factors
#define MH_Z19C_CONVERSION_FACTOR 250.0 / 4096.0
#define MQ_135_CONVERSION_FACTOR 0.189
#define MQ_7_CONVERSION_FACTOR 0.046
#define MQ_137_CONVERSION_FACTOR 0.74
*/
// Define WiFi credentials
const char* ssid = "THE_WIFI_SSID";
const char* password = "THE_WIFI_PASSWORD";

// Define server address
const char* serverAddress = "LOCAL HOST";
const int serverPort = 80;
// Define PM in normal condition (µg/m³)
const float PM1_0_normal = 150.0;
const float PM2_5_normal = 35.0;
const float PM10_normal = 150.0;
// Define gas concentration in normal condition
const int GAS_CONCENTRATION_normal = 500;
/*
const int CO2_Toxic_Concentration = 5000;
const int NO2_Toxic_Concentration = 5;
const int CO_Toxic_Concentration = 35;
const int SO2_Toxic_Concentratio = 5;
*/
// Instantiate PMS5003 sensor object
PMS5003 sensor(PMS_RX_PIN, PMS_TX_PIN);

// Instantiate LCD object (LiquidCrystal library)
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);


void setup() {
  // sets the serial port to 9600
  Serial.begin(9600);
 /* //code 
  Wire.begin(); 
  pinMode(MQ_135_ADDRESS, INPUT);
  pinMode(MQ_7_ADDRESS, INPUT);
  pinMode(MQ_137_ADDRESS, INPUT);
  */
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_green, OUTPUT);
  pinMode(LED_red, OUTPUT);
  lcd.begin(16, 2);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
 /*   // Read CO2 concentration
  uint16_t co2Concentration = readSensorData(MH_Z19C_ADDRESS, 0x24);
  float co2Ppm = co2Concentration * MH_Z19C_CONVERSION_FACTOR;
*/
 /* // Read NO2 concentration
  int mq135Voltage = analogRead(MQ_135_ADDRESS);
  float no2Ppm = mq135Voltage * MQ_135_CONVERSION_FACTOR;
*/
/*
  // Read CO concentration
  int mq7Voltage = analogRead(MQ_7_ADDRESS);
  float coPpm = mq7Voltage * MQ_7_CONVERSION_FACTOR;
*/
/*
  // Read SO2 concentration
  int mq137Voltage = analogRead(MQ_137_ADDRESS);
  float so2Ppm = mq137Voltage * MQ_137_CONVERSION_FACTOR;
*/

  // Read PMS data
  sensor.readData();

  // Get PM1.0, PM2.5, and PM10 values
  float pm1_0 = sensor.getPM1_0Concentration();
  float pm2_5 = sensor.getPM2_5Concentration();
  float pm10 = sensor.getPM10Concentration();

  // Read MQ-2 gas sensor
  int gasConcentration = analogRead(MQ_2_PIN);

  bool PmToxic = pm1_0 > PM1_0_normal || pm2_5 > PM2_5_normal || pm10 > PM10_normal;
  bool GasToxic = gasConcentration > GAS_CONCENTRATION_normal;
  /*
  bool CO2Toxic = co2Ppm > CO2_Toxic_Concentration;
  bool COToxic = coPpm > CO_Toxic_Concentration;
  bool NO2Toxic = no2Ppm > NO2_Toxic_Concentration;
  bool SO2Toxic = so2Ppm > SO2_Toxic_Concentration;
  */
  
  // Trigger buzzer if any normal is exceeded
  if (PmToxic || GasToxic) //change "GasToxic" to any specific gaz Toxic_consentration
  {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_red, HIGH);
    Serial.println("Toxic levels detected! Buzzer activated.");
  } else {
    digitalWrite(LED_green, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
  }
  // Prepare data string
  String dataString = "pm1_0=" + String(pm1_0, 6) + "&pm2_5=" + String(pm2_5, 6) + "&pm10=" + String(pm10, 6) + "&gas_concentration=" + String(gasConcentration);

  // Send data to server
  WiFiClient client;
  if (client.connect(serverAddress, serverPort)) {
    client.print("GET /?data=" + dataString + " HTTP/1.1\r\n");
    client.print("Host: " + serverAddress + "\r\n");
    client.print("\r\n");

    // Read response from server
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
    }

    client.stop();
  } else {
    Serial.println("Failed to connect to server");
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PM1.0:");
  lcd.print(pm1_0);
  lcd.print("µg/m³");
  lcd.setCursor(0, 1);
  lcd.print("PM2.5:");
  lcd.print(pm2_5);
  lcd.print("µg/m³");
  lcd.print("gas_concentration:");
  lcd.print(gas_concentration);
  lcd.print("ppm");
  /*  
  lcd.print("CO2 Concentration: ");
  lcd.print(co2Ppm);
  lcd.println(" ppm");
*/
/*
  lcd.print("NO2 Concentration: ");
  lcd.print(no2Ppm);
  lcd.println(" ppm");
*/
/*
  lcd.print("CO Concentration: ");
  lcd.print(coPpm);
  lcd.println(" ppm");
*/
/*
  lcd.print("SO2 Concentration: ");
  lcd.print(so2Ppm);
  lcd.println(" ppm");
  */
  // Delay between readings
  delay(1000);
}
/*//Reads data from a sensor connected via I2C communication (MH-Z19C) at address 0x68.

uint16_t readSensorData(byte address, byte command) {
  // Send command to read sensor data
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();

  // Wait for response
  delay(50);

  // Read data
  Wire.requestFrom(address, 2);
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  // Combine bytes and return sensor data
  return (highByte << 8) | lowByte;
  }*/
