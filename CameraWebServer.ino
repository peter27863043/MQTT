#include "esp_camera.h"
#include <WiFi.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

/* ------------------------------------------------
 *  WebCamera
   ------------------------------------------------*/
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT         // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE            // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM      // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM   // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE       // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM   // No  PSRAM
#define CAMERA_MODEL_AI_THINKER           // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL     // No  PSRAM
#include "camera_pins.h"

/* ------------------------------------------------
 *  SSD 1306 SPI OLED
   ------------------------------------------------*/
#include <Adafruit_SSD1306.h>
# define  SSD1306_SPI_CLK   12
# define  SSD1306_SPI_SDI   13
# define  SSD1306_SPI_DC    15
# define  SSD1306_SPI_CS    14

#define SCREEN_ADDRESS      0x3C

Adafruit_SSD1306 displaySPI(128, 64 , SSD1306_SPI_SDI
                                    , SSD1306_SPI_CLK
                                    , SSD1306_SPI_DC
                                    , -1
                                    , SSD1306_SPI_CS);

char mqtt_message1[256];
char mqtt_message2[256]; 

//#define WHITE    0xFFFF

void SSD1306_Test(){
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!displaySPI.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println("SSD1306 allocation failed");
  }
  else{ 
    Serial.println("SSD1306 allocation ok");

    // 清除顯示緩衝區
    displaySPI.clearDisplay();

    // 顯示文字
    displaySPI.setTextSize(1);
    displaySPI.setTextColor(WHITE);
    displaySPI.setCursor(0,5);
    displaySPI.println("MQTT Subscribe Client");

    displaySPI.drawFastHLine(0, 1,  128, WHITE);
    displaySPI.drawFastHLine(0, 15, 128, WHITE);
    displaySPI.drawFastHLine(0, 63, 128, WHITE);
    
    displaySPI.display();
    displaySPI.setTextSize(2);
    displaySPI.setCursor(0,18);
    displaySPI.println("ESP32 2022 ");
    displaySPI.display();
  }
}

void SSD1306_MQTT_2Message(char *pstr1, char *pstr2){
    // 清除顯示緩衝區
    displaySPI.clearDisplay();

    // 顯示文字
    displaySPI.setTextSize(1);
    displaySPI.setTextColor(WHITE);
    displaySPI.setCursor(0,5);
    displaySPI.println("MQTT Subscribe Client");

    displaySPI.drawFastHLine(0, 1,  128, WHITE);
    displaySPI.drawFastHLine(0, 15, 128, WHITE);
    displaySPI.drawFastHLine(0, 63, 128, WHITE);
        
    displaySPI.setCursor(0,18);
    displaySPI.println(pstr1);
    displaySPI.setCursor(0,28);
    displaySPI.println(pstr2);
    displaySPI.display();
}



void SSD1306_MQTT_4Message(char *pstr1, char *pstr2,char *pstr3, char *pstr4){
    // 清除顯示緩衝區
    displaySPI.clearDisplay();

    // 顯示文字
    displaySPI.setTextSize(1);
    displaySPI.setTextColor(WHITE);
    displaySPI.setCursor(0,5);
    displaySPI.println("MQTT Subscribe Client");

    displaySPI.drawFastHLine(0, 1,  128, WHITE);
    displaySPI.drawFastHLine(0, 15, 128, WHITE);
    displaySPI.drawFastHLine(0, 63, 128, WHITE);
    
    displaySPI.setCursor(0,18);
    displaySPI.println(pstr1);
    
    displaySPI.setCursor(0,28);
    displaySPI.println(pstr2);

    displaySPI.setCursor(0,38);
    displaySPI.println(pstr3);

    displaySPI.setCursor(0,48);
    displaySPI.println(pstr4);
    
    displaySPI.display();
}


/* ------------------------------------------------
 *  Blue tooth BLE
   ------------------------------------------------*/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_DEVICE_NAME     "XXXXXXXXX"

void BLE_setup() {
  Serial.println("Starting BLE work!");

  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);                          // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}


/* ------------------------------------------------
 * MQTT Client
   ------------------------------------------------*/
// ------ 以下修改成你MQTT設定 ------
#include <PubSubClient.h>

const char* ssid        = "XXXXXXXXXX";
const char* password    = "XXXXXXXXXX";

//const char* mqtt_server     = "mqtt.eclipseprojects.io";          //免註冊 MQTT 伺服器
//const char* mqtt_server     = "test.mosquitto.org";               //免註冊 MQTT 伺服器
const char* mqtt_server       = "172.16.2.92";                      //Local MQTT 伺服器
const unsigned int mqtt_port  = 1883;                               //unencrypt port
int test_counter              =0;
int room1_light;
#define MQTT_USER                       "XXXX"                      //User
#define MQTT_PASSWORD                   "XXXXXXX"                   //Passward
#define MQTT_PUBLISH_Monitor            "house/room1"               //放置MQTT Messagee的 Picture Topic
#define MQTT_PUBLISH_Topic              "house/room1"               //放置MQTT Messagee的 Counter Topic 
#define MQTT_PUBLISH_Topic_r1_light     "house/room1/light"         //放置MQTT Messagee的 LED Status  Topic
#define MQTT_SUBSCRIBE_Topic1_r1_light  "house/room1/ctrl_light"    //放置MQTT Subsribe的 LED Control Topic 

char clientId[50];
void mqtt_callback(char* topic, byte* payload, unsigned int msgLength);

WiFiClient wifiClient;
PubSubClient mqttClient(mqtt_server, mqtt_port, mqtt_callback, wifiClient);


void MQTT_setup(){
  room1_light=0; 
  pinMode(33, OUTPUT);        // Set the Red LED pin as output
  digitalWrite(33, HIGH);     // Turn off

  pinMode(4, OUTPUT);         // Set Flash Light LED pin pin as output
  digitalWrite(4, LOW);       // Turn off
  
  sprintf(clientId, "ESP32CAM_%04X", 1234);
  mqttClient.setCallback(mqtt_callback);
  mqtt_nonblock_reconnect(); 
  
  //啟動MQTT Subsribe
  if (mqttClient.connected()){

      sprintf(mqtt_message1, "Connected to %s :%d", mqtt_server,mqtt_port);
      SSD1306_MQTT_4Message(mqtt_message1,MQTT_PUBLISH_Topic,MQTT_USER,MQTT_PASSWORD);
    
      // Subscribe
      if(mqttClient.subscribe(MQTT_SUBSCRIBE_Topic1_r1_light)){
        Serial.println(MQTT_SUBSCRIBE_Topic1_r1_light);
        Serial.println(" subsribe ok!\n");
      }
      else{
        Serial.println(MQTT_SUBSCRIBE_Topic1_r1_light);
        Serial.println(" subsribe fail!\n");
      }
      delay(3000);
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int msgLength) {

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < msgLength; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic esp32/output, you check if the message is either "1" or "0". 
  // Changes the output state according to the message
  if (String(topic) == MQTT_SUBSCRIBE_Topic1_r1_light) {
    Serial.print("Changing output to ");
    if(messageTemp == "1"){
      Serial.println("on");
      room1_light=1;
      //digitalWrite(33, LOW);       // Turn on Red LED
      digitalWrite(4, HIGH);       // Turn on Flash Light LED
      //digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "0"){
      Serial.println("off");
      room1_light=0;
      //digitalWrite(33, HIGH);     // Turn off Red LED
      digitalWrite(4, LOW);       // Turn off Flash Light LED
      //digitalWrite(ledPin, LOW);
    }
  }
}

boolean mqtt_nonblock_reconnect() {
  boolean doConn = false;
  if (! mqttClient.connected()) {
    /* --------------------------------------------
    // select your connect funtion for USR_PASSWARD
       --------------------------------------------*/
    //boolean isConn = mqttClient.connect(clientId);
    boolean isConn = mqttClient.connect(clientId, MQTT_USER, MQTT_PASSWORD);
    
    char logConnected[200];
    sprintf(logConnected, "MQTT Client [%s] Connect to %s ,port:=%d %s!\n", clientId, mqtt_server,mqtt_port,(isConn ? "Successful" : "Failed"));
    Serial.print(logConnected);
    
    delay(1000);
     //啟動MQTT Subsribe
    if (mqttClient.connected()){

      sprintf(mqtt_message1, "Connected to %s :%d", mqtt_server,mqtt_port);
      SSD1306_MQTT_4Message(mqtt_message1,MQTT_PUBLISH_Topic,MQTT_USER,MQTT_PASSWORD);
    
      // Subscribe
      if(mqttClient.subscribe(MQTT_SUBSCRIBE_Topic1_r1_light)){
        Serial.println(MQTT_SUBSCRIBE_Topic1_r1_light);
        Serial.println(" subsribe ok!\n");
      }
      else{
        Serial.println(MQTT_SUBSCRIBE_Topic1_r1_light);
        Serial.println(" subsribe fail!\n");
      }
      delay(1000);
  }
    
  }
  //else
  //   Serial.println("MQTT Connected! \n");
  return doConn;
}

void MQTT_string(char *pTopic,char * pMessage) {

  int i;
  int len;
  char* logIsPublished;
  char tmep_string[128];
  boolean isPublished;
  
  if (! mqttClient.connected()) {
      // client loses its connection
      Serial.printf("MQTT Client [%s] Connection LOST(String)!\n", clientId);
      return;
  }

  //Serial.printf(pMessage);
   
  len=strlen(pMessage);
  mqttClient.beginPublish(pTopic, len, false);
  mqttClient.write((uint8_t *)pMessage, len);
  
  isPublished= mqttClient.endPublish();
  if (isPublished)
      logIsPublished = "  Publishing Message to MQTT Successfully !";
  else
      logIsPublished = "  Publishing Message to MQTT Failed !";

  //Serial.printf(logIsPublished);
}

void MQTT_picture() {
  //camera_fb_t * fb;    // camera frame buffer.
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    delay(100);
    Serial.println("Camera capture failed, Reset");
    ESP.restart();
  }

  char* logIsPublished;

  if (! mqttClient.connected()) {
    // client loses its connection
    Serial.printf("MQTT Client [%s] Connection LOST !\n", clientId);
    mqtt_nonblock_reconnect();
  }

  if (! mqttClient.connected())
        logIsPublished = "  No MQTT Connection, Photo NOT Published !";
  else {
        int imgSize = fb->len;
        int ps = MQTT_MAX_PACKET_SIZE;
        // start to publish the picture
        mqttClient.beginPublish(MQTT_PUBLISH_Monitor, imgSize, false);
        for (int i = 0; i < imgSize; i += ps) {
            int s = (imgSize - i < s) ? (imgSize - i) : ps;
            mqttClient.write((uint8_t *)(fb->buf) + i, s);
        }

        boolean isPublished = mqttClient.endPublish();
        if (isPublished)
            logIsPublished = "  Publishing Photo to MQTT Successfully !";
        else
            logIsPublished = "  Publishing Photo to MQTT Failed !";
  }
  Serial.println(logIsPublished);
  esp_camera_fb_return(fb);//清除緩衝區
}

int mqtt_loop_count=0;
int mqtt_red_led=0;
void MQTT_loop(){
  
  //sprintf(clientId, "ESP32CAM_%04X", random(0xffff));  // Create a random client ID
  sprintf(clientId, "ESP32CAM_%04X", 1234);
  
  mqtt_nonblock_reconnect(); 
  if (mqttClient.connected()){
      mqttClient.loop();
      //delay(1000);
    
      test_counter++;
      mqtt_red_led++;
      if(mqtt_red_led==1){
        digitalWrite(33, HIGH);     // Turn off
      }else{
        digitalWrite(33, LOW);      // Turn on  
        mqtt_red_led=0;
      }
      

      sprintf(mqtt_message1, "%d", room1_light);
      MQTT_string(MQTT_PUBLISH_Topic_r1_light,mqtt_message1);

      sprintf(mqtt_message1, "room1 counter=%04d\n", test_counter);
      MQTT_string(MQTT_PUBLISH_Topic,mqtt_message1);

      //LCD Message Display
      mqtt_loop_count++;
      if(mqtt_loop_count<4){
          if(room1_light==1)
            sprintf(mqtt_message2, "light = On");
          else
            sprintf(mqtt_message2, "light = Off");
          SSD1306_MQTT_4Message(MQTT_PUBLISH_Topic,mqtt_message1,MQTT_PUBLISH_Topic_r1_light,mqtt_message2);
      }else{
          sprintf(mqtt_message1, "%s :%d", mqtt_server,mqtt_port);
          SSD1306_MQTT_4Message("Connected to",mqtt_message1,MQTT_USER,MQTT_PASSWORD);
          if(mqtt_loop_count==5)
            mqtt_loop_count=0;
      }
      //Serial.println("\n");
      //MQTT_picture();//用MQTT傳照片
  }    
}

/* ------------------------------------------------
 *  EEPROM Access 
   ------------------------------------------------*/
  
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 256

void EEPROM_String_Write(int addr, char *pdata)
{
  
}

void EEPROM_String_Read(int addr, char *pdata)
{
  
}

void EEPROM_Dump_Diplay()
{
  int eeprom_data;
  int line_char=0;
  Serial.printf("\nEEPROM Dump");
  for(int i=0; i<256;i++){
    if(line_char==0){
      Serial.printf("\n%02x:",i);
    }
    eeprom_data=EEPROM.read(i);
    
    Serial.printf("%02x ",eeprom_data);
    line_char++;
    if(line_char==16)
      line_char=0;
  }
  Serial.printf("\n\n");
}

void EEPROM_ID_Write()
{
  int IDB1=0xa5;
  int IDB2=0x90;
  EEPROM.write(0, IDB1);
  EEPROM.write(1, IDB2);  

  IDB1=EEPROM.read(0);
  IDB2=EEPROM.read(1);  

  Serial.printf("EEPROM ID Read %x %x \n",IDB1,IDB2);
  
}


bool EEPROM_Setup(int opCode){
  bool bResult=false;
  int IDB1,IDB2;
  switch(opCode){
    case 1:   if(EEPROM.begin(EEPROM_SIZE))
                Serial.print("EEPROM Begin Ok\n");
              else  
                Serial.print("EEPROM Begin Fail\n");
              bResult=true;
              delay(1000);
              break;
    case 2:   if(EEPROM.commit())
                Serial.print("EEPROM Commit Ok\n");
              else
                Serial.print("EEPROM Commit Fail\n");
              bResult=true;
              break;
    case 3:   IDB1=EEPROM.read(0);
              IDB2=EEPROM.read(1);
              if((IDB1==0xa5)&&(IDB2==0x90)){
                bResult=true;
                sprintf(mqtt_message1,"EEPROM Tag OK %x %x \n", IDB1, IDB2);
                Serial.printf(mqtt_message1);
                EEPROM_Dump_Diplay();
              }
              else{
                sprintf(mqtt_message1,"EEPROM Tag Fail %x %x \n", IDB1, IDB2);
                Serial.printf(mqtt_message1);
                SSD1306_MQTT_2Message("EEPROM Data Restore...",mqtt_message1); 
                for(int k=2; k<256;k++)
                    EEPROM.write(k, k);
                Serial.print("EEPROM Data Write\n");
              }
    default:
              break;
  }
  return bResult;
}


/* ------------------------------------------------
 *  CameraServer Service Funtion 
   ------------------------------------------------*/
void startCameraServer();

/* ------------------------------------------------
 *  Wifi & Board Setup Funtion 
   ------------------------------------------------*/
void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println();
  delay(1000);
  Serial.print("Serial Port is initialed\n");

  camera_config_t config;
  config.ledc_channel   = LEDC_CHANNEL_0;
  config.ledc_timer     = LEDC_TIMER_0;
  config.pin_d0         = Y2_GPIO_NUM;
  config.pin_d1         = Y3_GPIO_NUM;
  config.pin_d2         = Y4_GPIO_NUM;
  config.pin_d3         = Y5_GPIO_NUM;
  config.pin_d4         = Y6_GPIO_NUM;
  config.pin_d5         = Y7_GPIO_NUM;
  config.pin_d6         = Y8_GPIO_NUM;
  config.pin_d7         = Y9_GPIO_NUM;
  config.pin_xclk       = XCLK_GPIO_NUM;
  config.pin_pclk       = PCLK_GPIO_NUM;
  config.pin_vsync      = VSYNC_GPIO_NUM;
  config.pin_href       = HREF_GPIO_NUM;
  config.pin_sscb_sda   = SIOD_GPIO_NUM;
  config.pin_sscb_scl   = SIOC_GPIO_NUM;
  config.pin_pwdn       = PWDN_GPIO_NUM;
  config.pin_reset      = RESET_GPIO_NUM;
  config.xclk_freq_hz   = 20000000;
  config.pixel_format   = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  // Initial SSD1306 OLED Module
  SSD1306_Test();
  Serial.print("\n");
  SSD1306_Test();
  Serial.print("\n");
  delay(3000);
  
  EEPROM_Setup(1);
  //Check EEPROM ID Byte1 & Byte2
  if(EEPROM_Setup(3)==false){
    EEPROM_ID_Write(); 
    EEPROM_Setup(2);
  }
  delay(1000);
  EEPROM.end();
  
  WiFi.begin(ssid, password);

  int count =0;
  char message[256];
  while (WiFi.status() != WL_CONNECTED) {
    count++;
    sprintf(message,"%d Wifi is not \nconnected to ",count);
    Serial.print(message);
    Serial.print(ssid);
    Serial.print("\n");
    SSD1306_MQTT_4Message(message,"",(char *)ssid,"");
    delay(1000);
  }
  Serial.println("\n");
  Serial.println("WiFi connected to ");
  Serial.print(ssid);
  Serial.print("\n");
  SSD1306_MQTT_2Message("WiFi connected to ",(char *) ssid);
  delay(2000);
  
  /* ------------------------------------------ 
      disable the Camera Server
     ------------------------------------------*/  
  //startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect\n");

  //啟動BLE連線
  //BLE_setup();

  //啟動MQTT連線
  MQTT_setup();

 
}


/* ------------------------------------------------
 *  main loop Funtion 
   ------------------------------------------------*/

void loop() {

  MQTT_loop();
  delay(1000);
}
