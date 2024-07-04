#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

// Replace these with your network credentials
#define WIFI_SSID "Orange-Mohamed Salah"
#define WIFI_PASSWORD "A1B2C3D4142002"

// Replace these with your Firebase project credentials
#define FIREBASE_HOST "mosalah185-eafa0-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "mwx159a0mfEngQNzyIfI5tk4brOurokCTFGCoxXl"
FirebaseJson json;
FirebaseData Ms;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
int notificationCount = 0; // To keep track of the notification number
String notificationPath;
// Water Flow Sensor
#define Water_FlowPin D0
volatile int pulseCount = 0;
unsigned long lastTime = 0;
unsigned long motorStartTime = 0;
bool motorJustStarted = true;
unsigned long currentTime = 0;
float calibrationFactor = 4.5; // Initial guess for calibration factor
float flowRate = 0;
float totalFlow = 0;
int eepromAddress = 0;

struct Sensor {
  int pin;
  String firebasePath;
  String header;
  String description;
  bool state; // To keep track of the sensor state from Firebase
  int Case;
  bool notified; // To keep track of whether a notification has been sent
};

Sensor sensors[] = {
  {D1, "/Not/LED", "Light System", "Light level is not optimal", false, HIGH, false},
  {D2, "/Not/PCB", "PCB Protection System", "Fire Detected! Check PCB Status.", false, LOW, false},
  {D3, "/Not/Fan", "Cooling System", "Fan Fault Detected! Check The Fan.", false, HIGH, false},
  {D4, "/Not/Hood", "Purifying System", "Extractor Fan Fault Detected! Check The Extractor Fan.", false, HIGH, false}
};

bool initialSetupDone = false;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void checkSensor(Sensor &sensor) {
  int sensorValue = digitalRead(sensor.pin);
  // Check the corresponding Firebase path
  if (Firebase.getBool(Ms, sensor.firebasePath)) {
    bool firebaseValue = Ms.boolData();
    if (firebaseValue && sensorValue == sensor.Case && !sensor.notified) {
      sendNotification(sensor);
      sensor.notified = true; // Mark notification as sent
    }
  } else {
    Serial.print("Error reading Firebase path: ");
    Serial.println(Ms.errorReason());
  }
}

void checkFirebaseChanges() {
  for (int i = 0; i < sizeof(sensors) / sizeof(Sensor); i++) {
    if (Firebase.getBool(Ms, sensors[i].firebasePath)) {
      bool newState = Ms.boolData();
      if (newState != sensors[i].state) {
        sensors[i].state = newState;
        sensors[i].notified = false; // Reset notification status when state changes
      }
    } else {
      Serial.println("Failed to get data from Firebase");
    }
  }
}

void sendNotification(Sensor sensor) {
  json.set("/heading", sensor.header);
  json.set("/time", timeClient.getFormattedTime());
  json.set("/description", sensor.description);

  String notificationPath = "/Notifications";

  if (Firebase.pushJSON(Ms, notificationPath, json)) {
    Serial.println("Notification sent successfully");

  } else {
    Serial.print("Error sending notification: ");
    Serial.println(Ms.errorReason());
  }
}

void initializeSensors() {
  for (int i = 0; i < sizeof(sensors) / sizeof(Sensor); i++) {
    int sensorValue = digitalRead(sensors[i].pin);
    if (Firebase.getBool(Ms, sensors[i].firebasePath)) {
      bool firebaseValue = Ms.boolData();
      sensors[i].state = firebaseValue;
      sensors[i].notified = (firebaseValue && sensorValue == sensors[i].Case); // Mark notified if the condition is already true
    } else {
      Serial.print("Error initializing sensor from Firebase path: ");
      Serial.println(Ms.errorReason());
    }
  }
}

int gettimeoffset();
bool Motor();
void startMotor();
void Waterflow();
void stopMotor();
void resetTotalFlow();
bool EEpromSetting();
void WaterFlowNotfication();
//void RESETcommand();
void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  timeClient.begin();

  int OFFSET = gettimeoffset();
  timeClient.setTimeOffset(OFFSET);

  // Retrieve the last notification count from Firebase
  if (Firebase.getInt(Ms, "/NotificationCount")) {
    notificationCount = Ms.intData();
  } else {
    notificationCount = 0;
  }

  // Initialize sensor pins
  for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
    pinMode(sensors[i].pin, INPUT);
  }
  // Initial states setup
  for (int i = 0; i < sizeof(sensors) / sizeof(Sensor); i++) {
    if (Firebase.getBool(Ms, sensors[i].firebasePath)) {
      sensors[i].state = Ms.boolData();
    }
  }
  // Initialize water flow sensor
  pinMode(Water_FlowPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Water_FlowPin), pulseCounter, FALLING);

  // Initialize EEPROM
  EEPROM.begin(512);
  // Read the stored total flow from EEPROM
  EEPROM.get(eepromAddress, totalFlow);
  Serial.print("Recovered total flow: ");
  Serial.println(totalFlow);

  lastTime = millis();
  initializeSensors();
  initialSetupDone = true;
}

void loop() {
  timeClient.update();
  //RESETcommand();
   if (initialSetupDone) {
    checkFirebaseChanges();
    for (int i = 0; i < sizeof(sensors) / sizeof(Sensor); i++) {
      checkSensor(sensors[i]);
    }
  
  if(Motor()){
  startMotor();
  Waterflow();
  }
   else{
    stopMotor();
    Firebase.setFloat(Ms, "WaterFlow/FlowRate", 0);
   }

  delay(1000);  // 10 seconds delay
}
}

int gettimeoffset() {
  if (Firebase.getInt(Ms, "/TimeOffSet")) {
    return Ms.intData();
  } else {
    Serial.print("Error getting time offset: ");
    Serial.println(Ms.errorReason());
    return 10800;  // Default to 0 if there's an error
  }
}

void Waterflow() {
  currentTime = millis();
  unsigned long timeInterval = currentTime - lastTime;

  // Add a delay of 2 seconds at the beginning if the motor has just started
  if (motorJustStarted) {
    delay(3000);
    motorStartTime = millis();
    motorJustStarted = false;
  }

  if (timeInterval > 1000) {
    detachInterrupt(digitalPinToInterrupt(Water_FlowPin));

    flowRate = ((1000.0 / timeInterval) * pulseCount) / calibrationFactor;
    totalFlow += (flowRate / 60); // in liters

    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.print(" L/min");
    Serial.print("  Total flow: ");
    Serial.print(totalFlow);
    Serial.println(" L");

    // Upload data to Firebase
    if (Firebase.setFloat(Ms, "WaterFlow/FlowRate", flowRate)) {
      Serial.println("Flow rate uploaded successfully");
    } else {
      Serial.println("Failed to upload flow rate");
    }

    if (Firebase.setFloat(Ms, "WaterFlow/TotalFlow", totalFlow)) {
      Serial.println("Total flow uploaded successfully");
    } else {
      Serial.println("Failed to upload total flow");
    }

    // Save the total flow to EEPROM
    EEPROM.put(eepromAddress, totalFlow);
    EEPROM.commit();

    pulseCount = 0;
    lastTime = millis();
    attachInterrupt(digitalPinToInterrupt(Water_FlowPin), pulseCounter, FALLING);

    // Check if flow rate is zero and stop the motor if it is
    if (flowRate == 0) {
      stopMotor();
      Firebase.setFloat(Ms, "WaterFlow/FlowRate", 0);
      WaterFlowNotfication();
      motorJustStarted = true; // Set the flag to true for the next cycle
    }
  }
}
// Function to reset the total flow in EEPROM
void resetTotalFlow() {
  totalFlow = 0;
  EEPROM.put(eepromAddress, totalFlow);
  EEPROM.commit();
  Serial.println("Total flow has been reset.");
}

bool EEpromSetting(){
  Firebase.getBool(Ms,"/WaterFlow/EEPROM_RESETTING");
  return Ms.boolData();
}

bool Motor(){
  Firebase.getBool(Ms,"/Not/MotorStatus");
  return Ms.boolData();
}
void startMotor() {
  Firebase.setBool(Ms,"/Not/MotorStatus",true);
}

void stopMotor() {
  Firebase.setBool(Ms,"/Not/MotorStatus",false);
}
void WaterFlowNotfication(){

  json.set("/heading", "Irrigation System");
  json.set("/time", timeClient.getFormattedTime());
  json.set("/description", "No Water Flow ! Check The Water Motor.");

  notificationPath = "/Notifications";

  if (Firebase.pushJSON(Ms, notificationPath, json)) {
    Serial.println("Notification sent successfully");
  } 
  else {
    Serial.print("Error sending notification: ");
    Serial.println(Ms.errorReason());
  }
}
/*void RESETcommand(){
  // Check for reset command
  if (Firebase.getString(Ms, "/ResetCommand")) {
    String resetCommand = Ms.stringData();
    if (resetCommand == "RESET") {
      // Delete all notifications
      if (Firebase.deleteNode(Ms, "/Notifications")) {
        Serial.println("All notifications deleted successfully");
      } else {
        Serial.print("Error deleting notifications: ");
        Serial.println(Ms.errorReason());
      }

      // Reset notification count
      notificationCount = 0;
      if (Firebase.setInt(Ms, "/NotificationCount", notificationCount)) {
        Serial.println("Notification count reset successfully");
      } else {
        Serial.print("Error resetting notification count: ");
        Serial.println(Ms.errorReason());
      }

      // Clear the reset command
      if (Firebase.setString(Ms, "/ResetCommand", "SET")) {
        Serial.println("Reset command cleared successfully");
      } else {
        Serial.print("Error clearing reset command: ");
        Serial.println(Ms.errorReason());
      }
    }
  }

}*/