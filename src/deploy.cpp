/*
   Code skeleton for weight measurements acquired from:
      https://github.com/olkal/HX711_ADC/blob/master/examples/Read_1x_load_cell_interrupt_driven/Read_1x_load_cell_interrupt_driven.ino
   WiFi and MQTT code skeleton acquired from:
      https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/

*/

#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <string>
#include <WiFi.h>
#include <PubSubClient.h>


//Define which dout (data out) and serial data clock
const int HX711_dout = 18; // microcontroller unit > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 19; // microcontroller unit > HX711 sck pin

const int HX711_dout_2 = 4; // microcontroller unit > HX711 dout pin, must be external interrupt capable!
const int HX711_sck_2 = 16; // microcontroller unit > HX711 sck pin

// HX711 constructor load cells:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
HX711_ADC LoadCell2(HX711_dout_2, HX711_sck_2);

float calibrationValue = 743.72; // Calibration value for load cell 1
float calibrationValue2 = 1733.58; // Calibration value for load cell 2


const int calVal_eepromAdress = 0; // address for the first load cell
const int calVal_eepromAdress2 = 10; // address for the second load cell

bool useEEPROM = true;
bool serviceEnabled = true;

unsigned long t = 0;
const int serialPrintInterval = 100;

volatile boolean newDataReady;
volatile boolean newDataReady2;


U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
bool enableBootDisplay = true;

// Replace the next variables with your SSID/Password combination
const char *ssid = "Cocktail_Mixer";
const char *password = "process_hubby";
//const char *ssid = "FRITZ!Box 7530 BS";
//const char *password = "09324416513504437202";

IPAddress server(131,159,6,111); // Local address broker
const int port = 1883; 
const std::string sensorID = "1111";

// Weight displayed on the display
int displayedWeight = 0;
int displayedWeight2 = 0;

// Weight Measured in the previous iteration
int lastWeight = 0;
int lastWeight2 = 0;

bool firstMeasurement = true;
bool firstMeasurement2 = true;


WiFiClient espClient;
PubSubClient client(espClient);

void displaySensorIDBottom()
{
  u8g2.setCursor(30, 60);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.print("Sensor ID: ");
  u8g2.print(sensorID.c_str());
}

void displayReconnectMessage() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("Disconnected");
  u8g2.setCursor(0, 40);
  u8g2.print("Reconnecting...");
  displaySensorIDBottom();
  u8g2.sendBuffer();
}

// interrupt routine cell 1
void dataReadyISR()
{
  if (LoadCell.update())
  {
    newDataReady = 1;
  }
}

// interrupt routine cell 2
void dataReadyISR2() {
  if (LoadCell2.update()) {
    newDataReady2 = 1;
  }
}

void setupLoadcell()
{
  if (useEEPROM)
  {
    EEPROM.begin(512);
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    //EEPROM.get(calVal_eepromAdress2, calibrationValue2);
  }
  Serial.print("Using Calibration for LoadCell 1:");
  Serial.println(calibrationValue); 

  Serial.print("Using Calibration value for LoadCell 2:");
  Serial.println(calibrationValue2);

  Serial.print("Using PIN for LoadCell 2 HX711_dout_2:");
  Serial.println(HX711_dout_2); 

  Serial.print("Using PIN for LoadCell 2 HX711_sck_2:");
  Serial.println(HX711_sck_2); 

  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check wiring to HX711 and pin designations");
    while (true);
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Loadcell 1 Startup is complete");
  }
  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);


  // LoadCell2 setup
  LoadCell2.begin();
  LoadCell2.start(stabilizingtime, _tare);
  if (LoadCell2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check wiring to HX711 and pin designations for LoadCell 2");
    while (true);
  } else {
    LoadCell2.setCalFactor(calibrationValue2);
    Serial.println("LoadCell 2 Startup is complete");
  }
  attachInterrupt(digitalPinToInterrupt(HX711_dout_2), dataReadyISR2, FALLING);
}


void setupDisplay()
{
  u8g2.begin();
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *message, unsigned int length)
{
  // For now we do not care about any topics, so we do not implement it
}

void setupMQTT()
{
  setup_wifi();
  client.setServer(server, port);
  client.setCallback(callback);
}

void displayBoot()
{
  u8g2.setCursor(20, 45);
  u8g2.setFont(u8g2_font_ncenB24_tr);
  u8g2.print("TUM");
  u8g2.sendBuffer();
  sleep(1);
}

void setup()
{
  Serial.begin(9600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  setupLoadcell();
  setupDisplay();
  if (enableBootDisplay)
  {
    displayBoot();
  }
  if (serviceEnabled) {
    setupMQTT();
  }
}

void displayWeight()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("Current Weight:");
  u8g2.setCursor(0, 40);
  u8g2.print(displayedWeight);
  u8g2.print(" grams");
  displaySensorIDBottom();
  u8g2.sendBuffer();
}

int gearState = 0;
void displayMeasuring()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("Measuring");
  u8g2.setCursor(0, 40);
  u8g2.print("Weight  ");

  switch (gearState)
  {
  case 0:
    u8g2.print("/");
    break;
  case 1:
    u8g2.print("-");
    break;
  case 2:
    u8g2.print("\\");
    break;
  }
  gearState = ++gearState % 3;
  displaySensorIDBottom();
  u8g2.sendBuffer();
}

void publishWeightToMQTT()
{
  std::string topic = "cocktail/weight/sensor_";
  std::string topic2 = "cocktail/weight/sensor_2";
  topic = topic + sensorID;
  topic2 = topic2 + sensorID;
  client.publish(topic.c_str(), reinterpret_cast<uint8_t *>(&displayedWeight), sizeof(float));
  client.publish(topic2.c_str(), reinterpret_cast<uint8_t *>(&displayedWeight2), sizeof(float));
}

void handleNewWeightData()
{
  newDataReady = 0;
  // Cutoff milligrams as the weighing setup is not that precise
  int newWeight = static_cast<int>(LoadCell.getData());
  // The delta in weight is not large, do not update the display (could be due to noise)
  bool smallDelta = abs(newWeight - displayedWeight) <= 2;
  // The delta in weight is pretty large and thus the weight measurement has not yet stabilized, do not update the display
  bool largeDelta = abs(newWeight - lastWeight) >= 3;
  if (!firstMeasurement)
  {
    if (largeDelta)
    {
      displayMeasuring();
    }
    lastWeight = newWeight;
    if (smallDelta || largeDelta)
    {
      // Do not update the screen
      return;
    }
  }
  else
  {
    firstMeasurement = false;
    lastWeight = newWeight;
  }
  if (newWeight <= 2)
  {
    // for very small values or negative values just display 0 as the precision of the scale is not that great
    newWeight = 0;
  }
  displayedWeight = newWeight;
  if (serviceEnabled)
  {
    publishWeightToMQTT();
  }
  displayWeight();
  Serial.print("Load_cell 1 output val: ");
  Serial.println(newWeight);
}

void handleNewWeightData2() {
  newDataReady2 = 0;
  int newWeight2 = static_cast<int>(LoadCell2.getData());
  bool smallDelta2 = abs(newWeight2 - displayedWeight2) <= 2;
  bool largeDelta2 = abs(newWeight2 - lastWeight2) >= 3;
  
  if (!firstMeasurement2) {
    if (largeDelta2) {
      displayWeight();
    }
    lastWeight2 = newWeight2;
    if (smallDelta2 || largeDelta2) {
      return;
    }
  } else {
    firstMeasurement2 = false;
    lastWeight2 = newWeight2;
  }
  
  if (newWeight2 <= 2) {
    newWeight2 = 0;
  }
  displayedWeight2 = newWeight2;
  if (serviceEnabled) {
    publishWeightToMQTT();
  }
  displayWeight();
  Serial.print("Load_cell 2 output val: ");
  Serial.println(newWeight2);
}


void reconnect()
{
  displayReconnectMessage();
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
      // display weight after reconnection
      displayWeight();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
  if (serviceEnabled)
  {
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
  }
  // get smoothed value from the dataset:
  if (newDataReady2)
  {
    handleNewWeightData2(); 
  }

    if (newDataReady)
    {
    //if (millis() > t + serialPrintInterval)
    //{
      handleNewWeightData(); 
    //  t = millis();
    //}
  }


  if (Serial.available() > 0)
  {
    char inByte = Serial.read();
    if (inByte == 't')
      LoadCell.tareNoDelay();
      LoadCell2.tareNoDelay();
  }

  if (LoadCell.getTareStatus())
  {
    Serial.println("Tare complete");
  }

  if (LoadCell2.getTareStatus()) 
  {
  Serial.println("LoadCell 2 Tare complete");
  }
}
