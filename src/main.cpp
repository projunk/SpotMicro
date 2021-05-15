#include <functions.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>


// configure failsafe for PPM on IA6B receiver: https://www.youtube.com/watch?v=T0DcXxpBS78
// procedure to program receiver:
// 1) set ch3 min endpoint to 120% (Setup/Endpoints)
// 2) set failsafe on ch 3 to -118%  (Function/RX Setup/Failsafe)
// 3) set ch3 min endpoint back to 100% (Setup/Endpoints)
// throttle signal smaller then certain value => failsafe triggered
// apperently this method makes a change on the receiver, since after disconnecting transmitter, 
// the transmitter itself cannot be sending these shorter pulses anymore

TaskHandle_t core2;

IPAddress ip(192, 168, 1, 170);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
String ssid = WIFI_SSID;
String password = WIFI_PASSWORD;

IPAddress AP_ip(192,168,8,1);
IPAddress AP_gateway(192,168,8,1);
IPAddress AP_subnet(255,255,255,0);
const char* AP_password = "123gps456";
const uint8_t AP_channel = 13;

AsyncWebServer server(80);

unsigned long displayTimer;


void setup() {
  Serial.begin(115200);

  robotName = identifyRobot(); 
  Serial.println();
  UniqueIDdump(Serial);
  Serial.println();
  Serial.print("Robot: ");
  Serial.println(robotName);

  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());

  initDogSpecs();
  if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    loadProps();
  } else {
    Serial.println("SPIFFS Mount Failed");
  }

  // pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VOLTAGE_SENSOR_PIN, ANALOG);

  // start second core
  xTaskCreatePinnedToCore(
    runOnCore2,
    "Core2",
    STACK_SIZE_CORE2,
    NULL,
    1,
    &core2,
    0);

  int32_t strongestChannel;
  uint8_t* strongestBssid;

  strongestBssid = getChannelWithStrongestSignal(ssid, &strongestChannel);
  if (strongestBssid == NULL) {
    // standalone accesspoint
    WiFi.onEvent(WiFiAPStarted, WiFiEvent_t::SYSTEM_EVENT_AP_START);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(getSSID().c_str(), AP_password, AP_channel); 
    for (;;) {
      if (isAPStarted()) {
        break;
      }
      delay(1);
    }  
    WiFi.softAPConfig(AP_ip, AP_gateway, AP_subnet);
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(getSSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    ipAddress = new IPAddress(WiFi.softAPIP());
  } else {
    // connect to local wifi network
    Serial.print("Connecting WiFi"); 
    WiFi.config(ip, gateway, subnet);

    // disable power safe for performance (low latency)
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.begin(ssid.c_str(), password.c_str(), strongestChannel, strongestBssid, true);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(ip);
    ipAddress = new IPAddress(ip);
  }

  voltage = readVoltage();
  Serial.print("Voltage [volt]: ");
  Serial.println(getVoltageStr());
  Serial.println();

  Serial.println("WebServer startup");

  distanceLeft = 0;
  distanceRight = 0;

  if (displayFound) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      Serial.println("Error starting diplay");
    }
  }

  server.on("/", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {   
    request->send(200, "text/html", getWebPage());
  }); 

  server.on("/Save", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Save");
    
    voltageCorrectionFactor = request->getParam(getIdFromName(NAME_VOLTAGE_CORRECTION))->value().toDouble();

    dog.getFrontLeftLeg()->setOffsetNeutralMicros(checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_LEFT_LEG, 1))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_LEFT_LEG, 2))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_LEFT_LEG, 3))->value().toInt()));
    dog.getBackLeftLeg()->setOffsetNeutralMicros(checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_LEFT_LEG, 1))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_LEFT_LEG, 2))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_LEFT_LEG, 3))->value().toInt()));
    dog.getFrontRightLeg()->setOffsetNeutralMicros(checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_RIGHT_LEG, 1))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_RIGHT_LEG, 2))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_RIGHT_LEG, 3))->value().toInt()));
    dog.getBackRightLeg()->setOffsetNeutralMicros(checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_RIGHT_LEG, 1))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_RIGHT_LEG, 2))->value().toInt()), checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_RIGHT_LEG, 3))->value().toInt()));            

    printProps();
    saveProps();

    request->redirect("/");
  });  

  server.on("/Cancel", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Cancel");
    request->redirect("/");
  });  

  server.on("/WifiOff", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/WifiOff");
    WiFi.mode(WIFI_OFF);
  });  

  server.on("/Defaults", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Defaults");
    
    initDogNeutralMicros();    
    voltageCorrectionFactor = defaultVoltageCorrectionFactor;
    printProps();
    saveProps();

    request->redirect("/");
  });    

  server.on("/RequestLatestData", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    //Serial.println("/RequestLatestData");
    request->send(200, "application/json", getLatestData());
  });  

  server.begin();
  displayTimer = millis();
}


void loop() {
  unsigned long millisTimer = millis();
  // the buckconverter cause too much noice to get an accurate reading of the voltage, so average
  voltage = averageFilter(readVoltage());
  distanceLeft = ultrasonicLeft.read(CM);
  distanceRight = ultrasonicRight.read(CM);  
/*
  Serial.print(voltage);
  Serial.print("\t");
  Serial.print(distanceLeft);
  Serial.print("\t");
  Serial.print(distanceRight);
  Serial.println();
*/
  if ((millisTimer - displayTimer) > 250) {  
    displayTimer = millisTimer;
    updateDisplay();
  }
  vTaskDelay(1);
}

