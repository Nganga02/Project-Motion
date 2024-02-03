#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
//#include <BLE2a05.h>
#include <Wire.h>
/*************************************************/
//Pins out
#define ENA_LEFT 14
#define ENA_RIGHT 12
#define IN_LEFT_FW 27
#define IN_LEFT_RV 26
#define IN_RIGHT_FW 25
#define IN_RIGHT_RV 33
#define ENCODER_LEFT_1 5
#define ENCODER_LEFT_2 18
#define ENCODER_RIGHT_1 36
#define ENCODER_RIGHT_2 39
/*************************************************/
//Encoder credentials to use
#define EN_Rev 360
volatile long encoderLeftValue, encoderRightValue = 0;
/*************************************************/
//milliseconds counter during the interval
long previousMillis = 0;
long currentMillis = 0;
/*************************************************/
//Setting up bluetooth credentials
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLEName "S.L.A.M"
bool deviceConnected = false;
bool oldDeviceConnected = false;
/*************************************************/
//Speed control values
double error, previous_error, output, integral = 0;
double setspeed, currentspeed, kp, ki, kd;
/*************************************************/

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Client Connected!");
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Client disconnecting... Waiting for new connection");
    pServer->startAdvertising();  // restart advertising
  }
};
/*************************************************/
//BLE RX callback
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristicTX) {
    Serial.print("Receiving.........");
    uint8_t *data = pCharacteristicTX->getData();
    //std::string rxValue = pCharacteristicTX->getValue();
    int size = pCharacteristicTX->getLength();

    if (size > 0) {
      Serial.println("*********");
      Serial.print("CMD:");
      Serial.print(data[0]);
      Serial.print("     Value:");
      Serial.println(data[1]);


      // for (int i = 0; i < size; i++)
      //   Serial.print(data[i]);

      // Serial.println();
      Serial.println("*********");
    }
  }
};


/*************************************************/
void setup() {
  //Initializing the pins
  Serial.begin(115200);
  Serial.println("Starting BLE work");


  pinMode(ENA_LEFT, OUTPUT);
  pinMode(ENA_RIGHT, OUTPUT);
  pinMode(IN_LEFT_FW, OUTPUT);
  pinMode(IN_LEFT_RV, OUTPUT);
  pinMode(IN_RIGHT_FW, OUTPUT);
  pinMode(IN_RIGHT_RV, OUTPUT);
  pinMode(ENCODER_LEFT_1, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_2, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_1, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_2, INPUT_PULLUP);

  //Attaching interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_1),updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_1), updateEncoderRight, RISING);

  //Initialising timer
  previousMillis = millis();
  
//  // create BLE DEVICE
//  BLEDevice::init(BLEName);
//  Serial.printf("BLE Server Mac Address: %s\n", BLEDevice::getAddress().toString().c_str());
//
//  //CREATE BLE Server
//  BLEServer *pServer = BLEDevice::createServer();
//  pServer->setCallbacks(new MyServerCallbacks());
//
//  //CREATE A SERVICE
//  BLEService *pService = pServer->createService(SERVICE_UUID);
//  //Serial.print(pService);
//
//  //CREATE a BLE CHARACTERISTIC FOR SENDING
//  BLECharacteristic *pCharacteristicTX = pService->createCharacteristic(
//    CHARACTERISTIC_UUID_TX,
//    BLECharacteristic::PROPERTY_NOTIFY);
//  pCharacteristicTX->addDescriptor(new BLE2902());
//  pCharacteristicTX->setValue("Hello World");
//
//  //Create a BLE CHARACTERISTIC FOR RECEIVING
//  BLECharacteristic *pCharacteristicRX = pService->createCharacteristic(
//    CHARACTERISTIC_UUID_RX,
//    BLECharacteristic::PROPERTY_WRITE);
//  pCharacteristicRX->setCallbacks(new MyCallbacks());
//  pCharacteristicRX->addDescriptor(new BLE2902());
//
//  //START THE SERVICE
//  pService->start();
//  // Start advertising
//  // BLEAdvertising *pAdvertising = pServer->getAdvertising();
//  // pAdvertising->start();
//
//  // pServer->getAdvertising()->start();
//  // Serial.println("Waiting a client connection to notify...");
//
//  //START ADVERTISING
//  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//  pAdvertising->addServiceUUID(SERVICE_UUID);
//  pAdvertising->setScanResponse(false);
//  pAdvertising->setMinPreferred(0x00);  // set value to 0x00 to not advertise this parameter
//  BLEDevice::startAdvertising();
//  Serial.println("Waiting a client connection to notify...");
}
/*************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(IN_LEFT_FW, LOW);
  digitalWrite(IN_LEFT_RV, HIGH);
  digitalWrite(IN_RIGHT_FW, LOW);
  digitalWrite(IN_RIGHT_RV, HIGH);
  analogWrite(ENA_LEFT, 255);
  analogWrite(ENA_RIGHT, 255);

}
/*************************************************/
void directionality(int cmd, double desired_speed){
  //Translating data from mobile
  if(cmd == 1){
  //Moving the vehicle forward
  if(desired_speed >= 140){
    digitalWrite(IN_LEFT_FW, HIGH);
    digitalWrite(IN_LEFT_RV, LOW);
    digitalWrite(IN_RIGHT_FW, HIGH);
    digitalWrite(IN_RIGHT_RV, LOW);
    Serial.print("Moving forward");
  }else if(desired_speed <= 110){
  //Backward
    digitalWrite(IN_LEFT_FW, LOW);
    digitalWrite(IN_LEFT_RV, HIGH);
    digitalWrite(IN_RIGHT_FW, LOW);
    digitalWrite(IN_RIGHT_RV, HIGH);
    Serial.print("Moving backward");
  }
  }
  //Moving the vehicle right


  //Left

  
}
/*************************************************/
void control(){
  //Making sure the speed is the same
  
}

/*************************************************/



/*************************************************/
//Updating encoder reading 
void updateEncoderLeft() {
  //Reading the value of the next encoder
  if (digitalRead(ENCODER_LEFT_2) == 0x1)
  {
    Serial.println("Left Moving forward");
  }else{
    Serial.println("Left Moving backward");
  }
  //incrementing encoder value
  encoderLeftValue++;
}

void updateEncoderRight() {
  //Reading the value of the next encoder
  if (digitalRead(ENCODER_RIGHT_2) == 0x1)
  {
    Serial.println("Right Moving backward");
  }else{
    Serial.println("Right Moving forward");
  }
  //incrementing encoder value
  encoderRightValue++;
}
