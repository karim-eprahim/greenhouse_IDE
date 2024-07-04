/* ------------------------------------------------------------------------
 * Created by: Tauseef Ahmad
 * Created on: 10 March, 2023
 *  
 * Tutorial: https://youtu.be/aNjkNmHRx4o
 * ------------------------------------------------------------------------
 * Download Resources
 * ------------------------------------------------------------------------
 * Preferences--> Aditional boards Manager URLs : 
 * For ESP8266 and NodeMCU - Board Version 2.6.3
 * http://arduino.esp8266.com/stable/package_esp8266com_index.json
 * ------------------------------------------------------------------------
 * HTTPS Redirect Library:
 * https://github.com/jbuszkie/HTTPSRedirect
 * Example Arduino/ESP8266 code to upload data to Google Sheets
 * Follow setup instructions found here:
 * https://github.com/StorageB/Google-Sheets-Logging
 * ------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <SPI.h>
#include <MFRC522.h>
#include <HTTPSRedirect.h>
//---------------------------------------------------------------------------------------------------------
// Google Script Deployment ID:
const char* GScriptId = "AKfycbwapJIQalUN04LpKcV9Pz5MvBV1TAZ7N9_uZVa3CvVmoJ_2erro73eCEEOjT0YTmnc";
String gate_number = "NO.1";
//---------------------------------------------------------------------------------------------------------
// Enter network credentials:
const char* ssid = "Orange-Mohamed Salah";
const char* password = "A1B2C3D4142002";
//---------------------------------------------------------------------------------------------------------
// Firebase credentials
#define FIREBASE_HOST "mosalah185-eafa0-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "mwx159a0mfEngQNzyIfI5tk4brOurokCTFGCoxXl"

//---------------------------------------------------------------------------------------------------------
// Google Sheets setup (do not edit)
const char* host = "script.google.com";
const int httpsPort = 443;
const char* fingerprint = "";
String url = String("/macros/s/") + GScriptId + "/exec";
HTTPSRedirect* client = nullptr;
//---------------------------------------------------------------------------------------------------------
// Firebase data object
FirebaseData Ms;
//---------------------------------------------------------------------------------------------------------
// Declare variables that will be published to Google Sheets
String student_id;
//---------------------------------------------------------------------------------------------------------
int blocks[] = { 4, 5, 6, 8, 9 };
#define total_blocks (sizeof(blocks) / sizeof(blocks[0]))
//---------------------------------------------------------------------------------------------------------
#define RST_PIN D3     //D3
#define SS_PIN D4      //D4
#define BUZZER_PIN 3  // Buzzer control pin (D8 on NodeMCU)
#define DoorLock 1
#define trigPin D1 // RX 
#define echoPin D2  // Tx
#define  MotorPin1 D0
#define  MotorPin2 D8
bool reopenDoor = false;
//---------------------------------------------------------------------------------------------------------
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
MFRC522::StatusCode status;
//---------------------------------------------------------------------------------------------------------
/* Be aware of Sector Trailer Blocks */
int blockNum = 2;
/* Length of buffer should be 2 Bytes more than the size of Block (16 Bytes) */
byte bufferLen = 18;
byte readBlockData[18];
//---------------------------------------------------------------------------------------------------------

/************
 * setup Function
************/
void setup() {
  //----------------------------------------------------------
  Serial.begin(9600);
  delay(10);
  Serial.println('\n');
  //----------------------------------------------------------
  SPI.begin();
  //----------------------------------------------------------
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(DoorLock, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //----------------------------------------------------------
  connectToWiFi();
  setupFirebase();
  //----------------------------------------------------------
  // Use HTTPSRedirect class to create a new TLS connection
  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  //----------------------------------------------------------
  Serial.print("Connecting to ");
  Serial.println(host);
  //----------------------------------------------------------
  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i = 0; i < 5; i++) {
    int retval = client->connect(host, httpsPort);
    if (retval == 1) {
      flag = true;
      String msg = "Connected. OK";
      Serial.println(msg);
      delay(2000);
      break;
    } else {
      Serial.println("Connection failed. Retrying...");
    }
  }
  //----------------------------------------------------------
  if (!flag) {
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    delay(5000);
    return;
  }
  //----------------------------------------------------------
  delete client;     // delete HTTPSRedirect object
  client = nullptr;  // delete HTTPSRedirect object
  //----------------------------------------------------------
  
  }


/************
 * loop Function
************/
void loop() {
  //----------------------------------------------------------------
  static bool flag = false;
  if (!flag) {
    client = new HTTPSRedirect(httpsPort);
    client->setInsecure();
    flag = true;
    client->setPrintResponseBody(true);
    client->setContentTypeHeader("application/json");
  }
  if (client != nullptr) {
    if (!client->connected()) {
      int retval = client->connect(host, httpsPort);
      if (retval != 1) {
        Serial.println("Disconnected. Retrying...");
        return;  //Reset the loop
      }
    }
  } else {
    Serial.println("Error creating client object!");
  }
  //----------------------------------------------------------------
  Serial.println("Scan your Tag");
  /* Initialize MFRC522 Module */
  mfrc522.PCD_Init();
  /* Look for new cards */
  /* Reset the loop if no new card is present on RC522 Reader */
  if (!mfrc522.PICC_IsNewCardPresent()) { return; }
  /* Select one of the cards */
  if (!mfrc522.PICC_ReadCardSerial()) { return; }
  /* Read data from the same block */
  Serial.println();
  Serial.println(F("Reading last data from RFID..."));
  String uidString = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    uidString += String(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.print("UID tag : ");
  Serial.println(uidString);
  //----------------------------------------------------------------
  String values = "", data;
  //creating payload - method 2 - More efficient
  for (byte i = 0; i < total_blocks; i++) {
    ReadDataFromBlock(blocks[i], readBlockData);
    if (i == 0) {
      data = String((char*)readBlockData);
      data.trim();
      student_id = data;
      values = "\"" + data + ",";
    } else {
      data = String((char*)readBlockData);
      data.trim();
      values += data + ",";
    }
  }
  values += gate_number + "\"}";
  Firebase.setString(Ms, "/Rfid/Last_Uid", uidString);
  //----------------------------------------------------------------
  // Handle access control
  handleAccessControl(uidString, values);
  //----------------------------------------------------------------
   
}

/************
 * handleAccessControl() function
 ************/
void handleAccessControl(String uidString, String values) {
  int Door_delay = getDoorDelay();
  int Motor_delay = getMotorDelay();
  if (Firebase.getString(Ms, "/Access_Control/" + uidString)) {
    if (Ms.stringData() == "allowed") {
      grantAccess(Door_delay, Motor_delay, values);
      // Handle door reopen delay
    if (reopenDoor) {
        delay(Door_delay); // Wait for the specified door delay
        closeDoor(); // Close the door
        reopenDoor = false; // Reset the flag
    }
    } else {
      denyAccess();
    }
  } else {
    cardNotFound();
  }
}

/************
 * ReadDataFromBlock() function
 ************/
void ReadDataFromBlock(int blockNum, byte readBlockData[]) {
  //----------------------------------------------------------------------------
  /* Prepare the key for authentication */
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  //----------------------------------------------------------------------------
  /* Authenticating the desired data block for Read access using Key A */
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockNum, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed for Read: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  } else {
    Serial.println("Authentication success");
  }
  //----------------------------------------------------------------------------
  /* Reading data from the Block */
  status = mfrc522.MIFARE_Read(blockNum, readBlockData, &bufferLen);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Reading failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  } else {
    readBlockData[16] = ' ';
    readBlockData[17] = ' ';
    Serial.println("Block was read successfully");
  }
}

/************
 * getDoorDelay() function
 ************/
int getDoorDelay() {
  Firebase.getInt(Ms, "/Rfid/Door_delay");
  return Ms.intData();
}
/************
 * getDoorDelay() function
 ************/
int getMotorDelay() {
  Firebase.getInt(Ms, "/Rfid/Motor_delay");
  return Ms.intData();
}
/************
 * grantAccess() function
 ************/
void grantAccess(int Door_delay, int Motor_delay, String values) {
  Serial.println("Access Granted");
  activateBuzzer(3, 100);
  digitalWrite(DoorLock, HIGH);
  Serial.println(Motor_delay);
    Motor("Open");
    delay(Motor_delay);  // Adjust this delay based on your motor speed and desired door opening time
    Motor("Stop");

  // Update door status in Firebase
  Firebase.setString(Ms, "/Rfid/DoorStatus", "Open");
  digitalWrite(DoorLock, LOW);

  // Upload data to Google Sheets
  uploadDataToGoogleSheets(values);

  // Gradually decrease Door_delay to zero
  while (Door_delay > 0) {
    delay(1000);         // 1 second delay between decrements
    Door_delay -= 1000;  // Decrement Door_delay by 1 second (1000 ms)
    Firebase.setInt(Ms, "/Rfid/Time_lift", Door_delay);
  }
  Motor("Close");
  delay(Motor_delay);  // Adjust this delay based on your motor speed and desired door opening time
  Motor("Stop");

  // Update door status in Firebase
  Firebase.setString(Ms, "/Rfid/DoorStatus", "Closed");
}

/************
 * denyAccess() function
 ************/
void denyAccess() {
  Serial.println("Access Denied");
  activateBuzzer(1, 2000);
}

/************
 * cardNotFound() function
 ************/
void cardNotFound() {
  Serial.println("UID not found in database");
  activateBuzzer(3, 500);
}

/************
 * activateBuzzer() function
 ************/
void activateBuzzer(int times, int delayTime) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(delayTime);
    digitalWrite(BUZZER_PIN, LOW);
    delay(delayTime);
  }
}

/************
 * connectToWiFi() function
 ************/
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

/************
 * setupFirebase() function
 ************/
void setupFirebase() {
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
}

/************
 * uploadDataToGoogleSheets() function
 ************/
void uploadDataToGoogleSheets(String values) {
  String payload_base = "{\"command\": \"insert_row\", \"sheet_name\": \"SmartGreenHouse\", \"values\": ";
  String payload = payload_base + values;

  Serial.println("Publishing data...");
  Serial.println(payload);
  if (client->POST(url, host, payload)) {
    Serial.println("[OK] Data published.");
    Serial.println("Student ID: " + student_id);
    Serial.println("Thanks");
  } else {
    Serial.println("Error while connecting");
    Serial.println("Failed. Try Again");
  }
}
long getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration / 2) / 29.1; // Convert duration to distance in cm
    return distance;
}

void Motor(String Status) {
    long distance = getDistance(); // Get the distance from the ultrasonic sensor

    if (Status == "Open") {
        if (distance > 40) {
            Serial.println("Distance is more than 40 cm. Motor will not open.");
        } else {
            openDoor();
        }
    } else if (Status == "Close") {
        closeDoor();
        if (distance < 20) {
            openDoor();
            reopenDoor = true; // Set the flag to close the door after a delay
        }
    } else if (Status == "Stop") {
        stopMotor();
    } else {
        Serial.println("Invalid Status");
    }
}

void openDoor() {
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
    Serial.println("Motor is Open");
}

void closeDoor() {
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);
    Serial.println("Motor is Close");
}

void stopMotor() {
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, LOW);
    Serial.println("Motor is Stop");
}