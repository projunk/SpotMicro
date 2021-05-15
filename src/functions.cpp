#include <functions.h>


volatile bool APStarted = false;
volatile unsigned long previousTimerValue;
volatile int channel[NR_OF_RECEIVER_CHANNELS];
volatile float voltage;
String robotName;
volatile bool signal_detected = false;
int signal_detected_count = 0;
int signal_lost_count = SIGNALS_DETECTED_LOST_THRESHOLD;
volatile bool buzzerDisabled = false;
volatile double voltageCorrectionFactor = defaultVoltageCorrectionFactor;

IPAddress *ipAddress;
volatile long usedUpLoopTime;
short gyro_x, gyro_y, gyro_z;
short acc_x, acc_y, acc_z;
volatile short temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

bool mpu_6050_found = false;
bool displayFound;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
volatile unsigned int distanceLeft, distanceRight;

HardwareSerial hs(2);
IBusBM IBus; 

DogMode dogMode;

TwoPosSwitch switchA(4);
TwoPosSwitch switchB(6);
TwoPosSwitch switchD(7);
ThreePosSwitch switchC(5);
Dog dog;

Ultrasonic ultrasonicLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);	
Ultrasonic ultrasonicRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);	
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);



double lowPassFilter(const double prmAlpha, const double prmCurrentValue, const double prmPreviousValue) {
  // https://sites.google.com/site/myimuestimationexperience/filters/complementary-filter
  return prmAlpha * prmPreviousValue + (1.0 - prmAlpha) * prmCurrentValue;
}


double averageFilter(const double prmCurrentValue) {
  const int nrOfMeasurements = 100;
  static double measurements[nrOfMeasurements];
  static int index = -1;
  static bool doCalcAverage = false;

  index++;
  if (index == nrOfMeasurements) {
    index = 0;
    doCalcAverage = true;
  }
  measurements[index] = prmCurrentValue;

  if (doCalcAverage) {
    double sum = 0.0;
    for (int i = 0; i < nrOfMeasurements; i++) {
      sum += measurements[i];
    }
    return sum / nrOfMeasurements;
  } else {
    return 0.0;
  }
}


int checkCenterOffset(const int prmCenterOffset) {
  if (prmCenterOffset > (MAX_PULSE - MID_CHANNEL)) {
    return (MAX_PULSE - MID_CHANNEL);
  }
  if (prmCenterOffset < (MIN_PULSE - MID_CHANNEL)) {
    return (MIN_PULSE - MID_CHANNEL);
  }  
  return prmCenterOffset;
}


void printProps() {
  dog.printOffsetNeutralMicros();
  Serial.print(voltageCorrectionFactor);  
  Serial.println();
}


void loadProps() {
  Serial.println("loadProps");
  buzzerDisabled = true;
  if (SPIFFS.exists("/props")) {
    File f = SPIFFS.open("/props", "r");  
    if (f) {
      dog.loadProps(f);
      voltageCorrectionFactor = f.readStringUntil('\n').toDouble();
      f.close();
    }
  }
  buzzerDisabled = false;
}


void saveProps() {
  Serial.println("saveProps");
  buzzerDisabled = true;
  File f = SPIFFS.open("/props", "w");
  if (f) {
    dog.saveProps(f);
    f.println(String(voltageCorrectionFactor, 2));
    f.close();
  }
  buzzerDisabled = false;
}


bool isEqualID(const uint8_t prmID[UniqueIDsize]) {
  for (size_t i = 0; i < UniqueIDsize; i++) {
			if (UniqueID[i] != prmID[i]) {
        return false;
      }
  }
  return true;
}


String identifyRobot() {
  String name;
  if (isEqualID(BREADBOARD_SPOT)) {
    name = "Breadboard Spot";
  } else if (isEqualID(YELLOW_SPOT)) {
    name = "Yellow Spot";
  } else {
    name = "Unknown";
  }
  return name;
}


bool isValidSignal(int prmPulse) {
  return (prmPulse > INVALID_SIGNAL_PULSE);
}


int getNrOfCells(float prmVBatTotal) {
  const float ABS_MAX_CELLVOLTAGE = FULLY_CHARGED_VOLTAGE+0.2;
  
  if (prmVBatTotal < 0.1) {
    return 0;
  } else if (prmVBatTotal < ABS_MAX_CELLVOLTAGE) {
    return 1;
  } else if (prmVBatTotal < 2*ABS_MAX_CELLVOLTAGE) {
    return 2;
  } else if (prmVBatTotal < 3*ABS_MAX_CELLVOLTAGE) {
    return 3;
  } else if (prmVBatTotal < 4*ABS_MAX_CELLVOLTAGE) {
    return 4;
  } else {
    // not supported 
    return 0; 
  }   
}


float readVoltage() {
  const float R1 = 4700.0;
  const float R2 = 1000.0;

  float vBatTotal = analogRead(VOLTAGE_SENSOR_PIN) * (3.3 / 4095.0) * (R1+R2)/R2;
  int nrOfCells = getNrOfCells(vBatTotal);
  float cellVoltage = 0.0;
  if (nrOfCells > 0) {
    cellVoltage = voltageCorrectionFactor * vBatTotal / nrOfCells;
  }
  /*
  Serial.print(vBatTotal); 
  Serial.print("\t");   
  Serial.print(nrOfCells); 
  Serial.print("\t");   
  Serial.println(cellVoltage); 
  */
  return cellVoltage;
}


String getVoltageStr() {
  return String(voltage, 2);
}


int fixChannelDirection(int prmChannel, boolean prmReversed) {
  if (prmReversed) {
    return map(prmChannel, MIN_PULSE, MAX_PULSE, MAX_PULSE, MIN_PULSE);
  } else {
    return prmChannel;
  }
}


bool isDisplayFound() {
  Wire1.beginTransmission(OLED_ADDR);
  return Wire1.endTransmission() == 0;
}


bool is_mpu_6050_found() {
  Wire.beginTransmission(0x68);
  return Wire.endTransmission() == 0;
} 


void setup_mpu_6050_registers() {
  if (!mpu_6050_found) return;
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


void read_mpu_6050_data() {   
  if (mpu_6050_found) {
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x3B);                                                    //Send the requested starting register
    Wire.endTransmission();                                              //End the transmission
    Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
    while(Wire.available() < 14);                                        //Wait until all the bytes are received
    acc_x = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the acc_x variable
    acc_y = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the acc_y variable
    acc_z = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the acc_z variable
    temperature = Wire.read()<<8 | Wire.read();                          //Add the low and high byte to the temperature variable
    gyro_x = Wire.read()<<8 | Wire.read();                               //Add the low and high byte to the gyro_x variable
    gyro_y = Wire.read()<<8 | Wire.read();                               //Add the low and high byte to the gyro_y variable
    gyro_z = Wire.read()<<8 | Wire.read();                               //Add the low and high byte to the gyro_z variable
  } else {
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
    temperature = 0;
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0;
  }
}


void calibrate_mpu_6050() {
  if (mpu_6050_found) {
    for (int cal_int = 0; cal_int < GYRO_CALIBRATION_COUNT ; cal_int ++) {                  //Run this code GYRO_CALIBRATION_COUNT times
      read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
      gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
      gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
      gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
      delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    }
    gyro_x_cal /= GYRO_CALIBRATION_COUNT;                                //Divide the gyro_x_cal variable by GYRO_CALIBRATION_COUNT to get the avarage offset
    gyro_y_cal /= GYRO_CALIBRATION_COUNT;                                //Divide the gyro_y_cal variable by GYRO_CALIBRATION_COUNT to get the avarage offset
    gyro_z_cal /= GYRO_CALIBRATION_COUNT;                                //Divide the gyro_z_cal variable by GYRO_CALIBRATION_COUNT to get the avarage offset  
  } else {
    gyro_x_cal = 0;
    gyro_y_cal = 0;
    gyro_z_cal = 0;
  }

  Serial.print(" gyro_x_cal:");
  Serial.print(gyro_x_cal);
  Serial.print(" gyro_y_cal:");
  Serial.print(gyro_y_cal);
  Serial.print(" gyro_z_cal:");
  Serial.println(gyro_z_cal);
}


float getTempCelsius() {
  return 36.53 + temperature/340.0;
}


void print_gyro_values() {
  /*
  Serial.print("acc_x:\t");
  Serial.print(acc_x);
  Serial.print("\tacc_y:\t");
  Serial.print(acc_y);
  Serial.print("\tacc_z:\t");
  Serial.print(acc_z);
*/
  Serial.print("\ttemp:\t");
  Serial.print(getTempCelsius());

  Serial.print("\tgyro_x:\t");
  Serial.print(gyro_x);
  Serial.print("\tgyro_y:\t");
  Serial.print(gyro_y);
  Serial.print("\tgyro_z:\t");
  Serial.println(gyro_z);
}


bool isArmed() {
  return (signal_detected && (switchA.readPos() == 2));
}


DogMode getDogMode() {
  switch (switchC.readPos()) {
    case 1:
      return dmManual;
    case 2:
      return dmSit;
    default:
      return dmCrawl;
  } 
}


void playTune(String prmTune) {
  if (!buzzerDisabled) {
    static char buf[64];
    strcpy(buf, prmTune.c_str());
    rtttl::begin(BUZZER_PIN, buf);
    rtttl::play();
  }
}


void playShortBeep() {
  playTune("ShortBeep:d=32,o=5,b=140:c5");
}


void playLowVoltageAlarm() {
  playTune("LowVoltageAlarm:d=16,o=5,b=140:c6,P,c5,P,c6,P,c5,P");
}


void playArmed() {
  Serial.println("Armed");        
  playTune("Armed:d=16,o=5,b=140:c5,P,c6,P,a7");
}


void playDisarmed() {
  Serial.println("Disarmed");        
  playTune("DisArmed:d=16,o=5,b=140:a7,P,c6,P,c5");  
}


void playSignalDetected() {
  Serial.println("Signal Detected");   
  playTune("SignalDetected:d=16,o=5,b=140:c5,P,c5,P,c5,P,c5,P,c5,P,c5,P,a7,P");
}


void playSignalLost() {
  Serial.println("Signal Lost");   
  playTune("SignalLost:d=16,o=5,b=140:a7,P,c5,P,c5,P,c5,P,c5,P,c5,P,c5,P");  
}


void servoTest() {
  int servonum = 15;
  Serial.println(servonum);
  for (;;) {
    pwm.writeMicroseconds(servonum, 1500);
    delay(2000);

    pwm.writeMicroseconds(servonum, 800);
    delay(2000);

    pwm.writeMicroseconds(servonum, 1500);
    delay(2000);

    pwm.writeMicroseconds(servonum, 2200);
    delay(2000);
  }
}


void readChannels() {
  for (int i = 0; i < NR_OF_RECEIVER_CHANNELS; i++) {
    channel[i] = IBus.readChannel(i);
  }
}


bool isSignalLostDetected(int prmPulse) {
  // https://github.com/aanon4/FlySkyIBus/issues/1
  // last 4bits of prmPulse
  unsigned int failBits = (prmPulse >> 12) & 0xF;
  if (failBits == 0x2) {
    // the first 12 bits of channel the value contains the actual value from receiver
    for (int i = 0; i < NR_OF_RECEIVER_CHANNELS; i++) {
      channel[i] = channel[i] & 0xFFF;
    }
    return true;
  } else {
    return false;
  }
}


void printChannels() {
  for (int i = 0; i < NR_OF_RECEIVER_CHANNELS; i++) {
    Serial.print(channel[i]);
    Serial.print("\t");
  }
  Serial.println();
}
  

void updateDisplay() {
  int lineDistance = 12;
  if (displayFound) {  
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("STATUS SPOTMICRO");

    int yPos = 16;
    display.setCursor(0, yPos);
    display.print("IP:");
    display.print(ipAddress->toString());
    yPos += lineDistance;

    display.setCursor(0, yPos);
    display.print("Distance Left:");
    display.print(distanceLeft);
    display.print(" cm");
    yPos += lineDistance;

    display.setCursor(0, yPos);
    display.print("Distance Right:");
    display.print(distanceRight);
    display.print(" cm");
    yPos += lineDistance;

    display.setCursor(0, yPos);
    display.print("Voltage:");
    display.print(voltage);
    display.print(" V");

    display.display();
  }  
}


void printString(String prmValue, String prmTitle, String prmUnit)
{
  char buf[64];
  sprintf(buf, "%s : %4s %s", prmTitle.c_str(), prmValue.c_str(), prmUnit.c_str()); 
  Serial.println(buf); 
}


void printInt(int32_t prmValue, String prmTitle, String prmUnit)
{
  char buf[64];
  sprintf(buf, "%s : %6d %s", prmTitle.c_str(), prmValue, prmUnit.c_str()); 
  Serial.println(buf); 
}


String getSSID() {
  String ssid = SSID_BASE + robotName;
  ssid.toUpperCase();
  return ssid;
}


String getBSSID(uint8_t* prmStrongestBssid) {
  if (prmStrongestBssid == NULL) {
    return "";
  }
  static char BSSID_char[18];
  for (int i = 0; i < 6; ++i){
    sprintf(BSSID_char,"%s%02x:",BSSID_char,prmStrongestBssid[i]);
  } 
  BSSID_char[17] = '\0';
  return String(BSSID_char);
}


uint8_t* getChannelWithStrongestSignal(String prmSSID, int32_t *prmStrongestChannel) {
  Serial.println("getChannelWithStrongestSignal ...............");
  byte available_networks = WiFi.scanNetworks();

  uint8_t* strongestBssid = NULL;
  int32_t rssiMax = -2147483648;
  *prmStrongestChannel = -1;
  for (int network = 0; network < available_networks; network++) {
    if (WiFi.SSID(network).equalsIgnoreCase(prmSSID)) {
      if (WiFi.RSSI(network) > rssiMax) {
        rssiMax = WiFi.RSSI(network);
        strongestBssid = WiFi.BSSID(network);
        *prmStrongestChannel = WiFi.channel(network);
      }
    }    
  }

  printInt(rssiMax, "rssiMax", "dB");          
  printInt(*prmStrongestChannel, "StrongestChannel", "");
  printString(getBSSID(strongestBssid), "StrongestBssid", "");  
  Serial.println();

  return strongestBssid;
}


String getIdFromName(String prmName) {
  String id = prmName;
  id.replace(" ", "_");
  id.replace("[", "");
  id.replace("]", "");
  id.replace("(", "");
  id.replace(")", "");
  return id;
}


String getIdFromName(String prmName, int prmIndex) {
  String id = prmName;
  id.replace(" ", "_");
  id.replace("[", "");
  id.replace("]", "");
  id.replace("(", "");
  id.replace(")", "");
  return id + String(prmIndex);
}


bool isAPStarted() {
  return APStarted;
}


void WiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info) {
  APStarted = true;
  Serial.println("AP Started");
}


String addDQuotes(String prmValue) {
  return "\"" + prmValue + "\"";
}


String addRow(String prmName, bool prmIsEditableCell, bool prmIsHeader, String prmValue = "") {
  String editableCell = prmIsEditableCell ? "contenteditable='true'" : "";
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:20%\" ALIGN=CENTER>" + prmName + suffix;
  String col2 = prefix + editableCell + " id=" + addDQuotes(getIdFromName(prmName)) + " style=\"width:20%\" ALIGN=CENTER>" + prmValue + suffix;
  return "<tr>" + col1 + col2 + "</tr>";
}


String addRow(String name, String value1, String value2, String value3, bool prmIsEditableCell, bool prmIsHeader = false) {
  String editableCell = prmIsEditableCell ? "contenteditable='true'" : "";
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:25%\" ALIGN=CENTER>" + name + suffix;
  String col2 = prefix + editableCell + " id=" + addDQuotes(getIdFromName(name, 1)) + " style=\"width:25%\" ALIGN=CENTER>" + value1 + suffix;
  String col3 = prefix + editableCell + " id=" + addDQuotes(getIdFromName(name, 2)) + " style=\"width:25%\" ALIGN=CENTER>" + value2 + suffix;
  String col4 = prefix + editableCell + " id=" + addDQuotes(getIdFromName(name, 3)) + " style=\"width:25%\" ALIGN=CENTER>" + value3 + suffix;
  return "<tr>" + col1 + col2 + col3 + col4 + "</tr>";
}


String toString(bool prmValue) {
  if (prmValue) return "True";
  return "False";
}


String toString(int prmValue) {
  return String(prmValue);
}


String toString(double prmValue) {
  return String(prmValue, 2);
}


String getBatteryProgressStr() {
  return "<div class=\"progress\"><div id=\"" ID_PROGRESS_BATTERY "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">0%</div></div>";
}


String getChannelProgressStr(String prmChannelDivID, String prmChannelSpanID, String prmColor) {
  return "<div class=\"progress\"><div id=\"" + prmChannelDivID + "\" class=\"progress-bar " + prmColor + "\"" + " role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"800\" aria-valuemax=\"2200\" style=\"width:0%\"><span id=\"" + prmChannelSpanID + "\" ></span></div></div>";
}


String getHtmlHeader() {
  String s = "";
  s += "<head>";
  s += "  <meta><title>WebService: " + robotName + "</title>";
  s += "  <style>";
  s += "    table, th, td {border: 1px solid black; border-collapse: collapse;}";
  s += "    tr:nth-child(even) { background-color: #eee }";
  s += "    tr:nth-child(odd) { background-color: #fff;}";
  s += "    td:first-child { background-color: lightgrey; color: black;}";
  s += "    th { background-color: lightgrey; color: black;}";
  s += "  </style>";  
  s += "  <style>";
  s += "    html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
  s += "    .button { border-radius: 12px; background-color: grey; border: none; color: white; padding: 16px 40px;";
  s += "    text-decoration: none; font-size: 30px; margin: 10px; cursor: pointer;}";
  s += "    .progress-bar{float:left;width:0%;height:100%;font-size:16px;line-height:20px;color:#fff;text-align:center;background-color:#337ab7;-webkit-box-shadow:inset 0 -1px 0 rgba(0,0,0,.15);box-shadow:inset 0 -1px 0 rgba(0,0,0,.15);-webkit-transition:width .6s ease;-o-transition:width .6s ease;transition:width .6s ease}";
  s += "    .progress-bar-success{background-color:#5cb85c}";
  s += "    .progress-bar-warning{background-color:#f0ad4e}";
  s += "    .progress-bar-danger{background-color:#d9534f}";
  s += "    .progress-bar-ch1{background-color:#cc0000}";
  s += "    .progress-bar-ch2{background-color:purple}";
  s += "    .progress-bar-ch3{background-color:#0066cc}";
  s += "    .progress-bar-ch4{background-color:#2eb8b8}";
  s += "    .progress-bar-ch5{background-color:#26734d}";
  s += "    .progress-bar-ch6{background-color:#2eb82e}";
  s += "    .progress-bar-ch7{background-color:#ccff99}";  
  s += "    .progress-bar-ch8{background-color:#ffcc00}";
  s += "    .progress-bar-ch9{background-color:#ff6600}";
  s += "    .progress-bar-ch10{background-color:#663300}";
  s += "  </style>";  
  s += "  <style>";
  s += "    .progress {";
  s += "        position: relative;";
  s += "        height:20px;";
  s += "    }";
  s += "    .progress span {";
  s += "        position: absolute;";
  s += "        display: block;";
  s += "        width: 100%;";
  s += "        color: black;";
  s += "     }";
  s += "  </style>";  
  s += "</head>";
  return s;
}


String getScript() {
  String s = "";
  s += "<script>";

  s += "requestData();";
  s += "var timerId = setInterval(requestData, " WEBPAGE_REFRESH_INTERVAL ");";

  s += "function getBatColorMsg(prmVoltage) {";  
  s += "  if (prmVoltage < " + String(LOW_VOLTAGE_ALARM, 2) + ") {";
  s += "    return \"progress-bar progress-bar-danger\";";
  s += "  } else if (prmVoltage <" + String(WARNING_VOLTAGE, 2) + ") {";
  s += "    return \"progress-bar progress-bar-warning\";";
  s += "  } else {";
  s += "    return \"progress-bar progress-bar-success\";";
  s += "  }";  
  s += "}";

  s += "function getBatPercentage(prmVoltage) {";
  s += "  var percentage = 100.0*((prmVoltage - " + String(LOW_VOLTAGE_ALARM, 2) + ")/" + String(FULLY_CHARGED_VOLTAGE - LOW_VOLTAGE_ALARM, 2) + ");";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function getChannelPercentage(prmPulse) {";
  s += "  var percentage = 100.0*((prmPulse - 800)/(2200-800));";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function updateChannel(prmDivProgressID, prmSpanProgressID, prmValue) {";
  s += "  var divProgressElem = document.getElementById(prmDivProgressID);";
  s += "  var spanProgressElem = document.getElementById(prmSpanProgressID);";
  s += "  var value = parseInt(prmValue);";
  s += "  divProgressElem.setAttribute(\"aria-valuenow\", value);";
  s += "  divProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getChannelPercentage(value)).toFixed(0) + \"%\");";
  s += "  spanProgressElem.innerText = value;";
  s += "}";

  s += "function requestData() {";
  s += "  var xhr = new XMLHttpRequest();";  
  s += "  xhr.open(\"GET\", \"/RequestLatestData\", true);";
  s += "  xhr.timeout = (" WEBPAGE_TIMEOUT ");";  
  s += "  xhr.onload = function() {";
  s += "    if (xhr.status == 200) {";
  s += "      if (xhr.responseText) {";
  s += "        var data = JSON.parse(xhr.responseText);";
  s += "        var parser = new DOMParser();";
  s += "        document.getElementById(\"" + getIdFromName(NAME_SIGNAL_DETECTED) + "\").innerText = data." + getIdFromName(NAME_SIGNAL_DETECTED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ARMED) + "\").innerText = data." + getIdFromName(NAME_ARMED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_MODE) + "\").innerText = data." + getIdFromName(NAME_MODE) + ";";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_1) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_1) + "\"," + "data." + getIdFromName(NAME_CHANNEL_1) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_2) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_2) + "\"," + "data." + getIdFromName(NAME_CHANNEL_2) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_3) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_3) + "\"," + "data." + getIdFromName(NAME_CHANNEL_3) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_4) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_4) + "\"," + "data." + getIdFromName(NAME_CHANNEL_4) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_5) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_5) + "\"," + "data." + getIdFromName(NAME_CHANNEL_5) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_6) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_6) + "\"," + "data." + getIdFromName(NAME_CHANNEL_6) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_7) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_7) + "\"," + "data." + getIdFromName(NAME_CHANNEL_7) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_8) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_8) + "\"," + "data." + getIdFromName(NAME_CHANNEL_8) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_9) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_9) + "\"," + "data." + getIdFromName(NAME_CHANNEL_9) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_10) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_10) + "\"," + "data." + getIdFromName(NAME_CHANNEL_10) + ");";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_BATTERY) + "\").innerText = data." + getIdFromName(NAME_BATTERY) + ";";  
  s += "        var progressElem = document.getElementById(\"" ID_PROGRESS_BATTERY "\");";
  s += "        var voltage = parseFloat(data." + getIdFromName(NAME_BATTERY) + ");";
  s += "        progressElem.setAttribute(\"class\", getBatColorMsg(voltage));";
  s += "        progressElem.setAttribute(\"aria-valuenow\", parseFloat(getBatPercentage(voltage)).toFixed(0));";
  s += "        progressElem.setAttribute(\"style\", \"width:\" + parseFloat(getBatPercentage(voltage)).toFixed(0) + \"%\");";
  s += "        progressElem.innerText = parseFloat(getBatPercentage(voltage)).toFixed(0) + \"%\";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_DISTANCE_LEFT) + "\").innerText = data." + getIdFromName(NAME_DISTANCE_LEFT) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_DISTANCE_RIGHT) + "\").innerText = data." + getIdFromName(NAME_DISTANCE_RIGHT) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_USED_UP_LOOPTIME) + "\").innerText = data." + getIdFromName(NAME_USED_UP_LOOPTIME) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_TEMPERATURE) + "\").innerText = data." + getIdFromName(NAME_TEMPERATURE) + ";";
  s += "      }";  
  s += "    }";
  s += "  };";
  s += "  xhr.send();";  
  s += "}";
  
  s += "function getNameValue(prmName) {";
  s += "  var value = document.getElementById(prmName).innerHTML;";
  s += "  return prmName + \"=\" + value;";
  s += "}";

  s += "function savePropValues() {";
  s += "  clearInterval(timerId);";
  s += "  var xhr = new XMLHttpRequest();";
  s += "  var frontLeftLegValues = getNameValue(\"" + getIdFromName(NAME_FRONT_LEFT_LEG, 1) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_FRONT_LEFT_LEG, 2) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_FRONT_LEFT_LEG, 3) + "\");";
  s += "  var backLeftLegValues = getNameValue(\"" + getIdFromName(NAME_BACK_LEFT_LEG, 1) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_BACK_LEFT_LEG, 2) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_BACK_LEFT_LEG, 3) + "\");";
  s += "  var frontRightLegValues = getNameValue(\"" + getIdFromName(NAME_FRONT_RIGHT_LEG, 1) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_FRONT_RIGHT_LEG, 2) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_FRONT_RIGHT_LEG, 3) + "\");";
  s += "  var backRightLegValues = getNameValue(\"" + getIdFromName(NAME_BACK_RIGHT_LEG, 1) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_BACK_RIGHT_LEG, 2) + "\") + \"&\" + getNameValue(\"" + getIdFromName(NAME_BACK_RIGHT_LEG, 3) + "\");";
  s += "  var voltageCorrectionFactor =  getNameValue(\"" + getIdFromName(NAME_VOLTAGE_CORRECTION) + "\");";
  s += "  xhr.open(\"GET\", \"/Save?\" + frontLeftLegValues + \"&\" + backLeftLegValues + \"&\" + frontRightLegValues + \"&\" + backRightLegValues + \"&\" + voltageCorrectionFactor, false);";
  s += "  xhr.send();";
  s += "  location.reload();";
  s += "}";

  s += "</script>";
  
  return s;
}


String getDogModeSt() {
  switch (dogMode) {
    case dmManual:
      return "Manual";
    case dmSit:
      return "Sit";
    default:
      return "Crawl";
  }
}


String getWebPage() {
  String s = "<!DOCTYPE html><html>";
  s += getHtmlHeader();
  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow(NAME_MODEL, false, true, robotName);
  s += addRow(NAME_VERSION, false, false, SPOTMICRO_VERSION);
  s += addRow(NAME_SIGNAL_DETECTED, false, false);
  s += addRow(NAME_ARMED, false, false);
  s += addRow(NAME_MODE, false, false);
  s += addRow(NAME_CHANNEL_1, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_1, ID_SPAN_PROGRESS_CHANNEL_1, "progress-bar-ch1"));
  s += addRow(NAME_CHANNEL_2, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_2, ID_SPAN_PROGRESS_CHANNEL_2, "progress-bar-ch2"));
  s += addRow(NAME_CHANNEL_3, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_3, ID_SPAN_PROGRESS_CHANNEL_3, "progress-bar-ch3"));
  s += addRow(NAME_CHANNEL_4, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_4, ID_SPAN_PROGRESS_CHANNEL_4, "progress-bar-ch4"));
  s += addRow(NAME_CHANNEL_5, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_5, ID_SPAN_PROGRESS_CHANNEL_5, "progress-bar-ch5"));
  s += addRow(NAME_CHANNEL_6, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_6, ID_SPAN_PROGRESS_CHANNEL_6, "progress-bar-ch6"));  
  s += addRow(NAME_CHANNEL_7, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_7, ID_SPAN_PROGRESS_CHANNEL_7, "progress-bar-ch7"));  
  s += addRow(NAME_CHANNEL_8, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_8, ID_SPAN_PROGRESS_CHANNEL_8, "progress-bar-ch8"));  
  s += addRow(NAME_CHANNEL_9, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_9, ID_SPAN_PROGRESS_CHANNEL_9, "progress-bar-ch9"));  
  s += addRow(NAME_CHANNEL_10, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_10, ID_SPAN_PROGRESS_CHANNEL_10, "progress-bar-ch10"));    
  s += addRow(NAME_BATTERY, false, false);
  s += addRow(NAME_BATTERY_PROGRESS, false, false, getBatteryProgressStr());
  s += addRow(NAME_DISTANCE_LEFT, false, false);
  s += addRow(NAME_DISTANCE_RIGHT, false, false);
  s += addRow(NAME_USED_UP_LOOPTIME, false, false);
  s += addRow(NAME_TEMPERATURE, false, false);
  s += addRow(NAME_VOLTAGE_CORRECTION, true, false, String(voltageCorrectionFactor, 2));
  s += "</table>";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow("Servo Center Offset", "Shoulder", "Upper Leg", "Knee", false, true);
  s += addRow(NAME_FRONT_LEFT_LEG, toString(dog.getFrontLeftLeg()->getShoulderJoint()->getOffsetNeutralMicros()), toString(dog.getFrontLeftLeg()->getUpperLegJoint()->getOffsetNeutralMicros()), toString(dog.getFrontLeftLeg()->getKneeJoint()->getOffsetNeutralMicros()), true);
  s += addRow(NAME_BACK_LEFT_LEG, toString(dog.getBackLeftLeg()->getShoulderJoint()->getOffsetNeutralMicros()), toString(dog.getBackLeftLeg()->getUpperLegJoint()->getOffsetNeutralMicros()), toString(dog.getBackLeftLeg()->getKneeJoint()->getOffsetNeutralMicros()), true);
  s += addRow(NAME_FRONT_RIGHT_LEG, toString(dog.getFrontRightLeg()->getShoulderJoint()->getOffsetNeutralMicros()), toString(dog.getFrontRightLeg()->getUpperLegJoint()->getOffsetNeutralMicros()), toString(dog.getFrontRightLeg()->getKneeJoint()->getOffsetNeutralMicros()), true);
  s += addRow(NAME_BACK_RIGHT_LEG, toString(dog.getBackRightLeg()->getShoulderJoint()->getOffsetNeutralMicros()), toString(dog.getBackRightLeg()->getUpperLegJoint()->getOffsetNeutralMicros()), toString(dog.getBackRightLeg()->getKneeJoint()->getOffsetNeutralMicros()), true);
  s += "</table>";

  s += "<br>";

  s += "<div class=\"btn-group\" style=\"width:100%\">";
  s += "<a><button onclick=\"savePropValues()\" type=\"button\" style=\"width:170px\" class=\"button\">Save</button></a>";  
  s += "<a href=\"/Cancel\"><button type=\"button\" style=\"width:170px\" class=\"button\">Cancel</button></a>";
  s += "<a href=\"/WifiOff\"><button type=\"button\" style=\"width:170px;white-space:nowrap\" class=\"button\">Wifi Off</button></a>";
  s += "<a href=\"/Defaults\"><button type=\"button\" style=\"width:170px\" class=\"button\">Defaults</button></a>";
  s += "</div>";

  s += getScript();

  s += "</body></html>";  
  return s;
}


String getLatestData() {
  String data = "{";
  data += "\"" + getIdFromName(NAME_SIGNAL_DETECTED) + "\":" + addDQuotes(toString(signal_detected)) + ",";
  data += "\"" + getIdFromName(NAME_ARMED) + "\":" + addDQuotes(toString(isArmed())) + ",";
  data += "\"" + getIdFromName(NAME_MODE) + "\":" + addDQuotes(getDogModeSt()) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_1) + "\":" + addDQuotes(toString(channel[0])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_2) + "\":" + addDQuotes(toString(channel[1])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_3) + "\":" + addDQuotes(toString(channel[2])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_4) + "\":" + addDQuotes(toString(channel[3])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_5) + "\":" + addDQuotes(toString(channel[4])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_6) + "\":" + addDQuotes(toString(channel[5])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_7) + "\":" + addDQuotes(toString(channel[6])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_8) + "\":" + addDQuotes(toString(channel[7])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_9) + "\":" + addDQuotes(toString(channel[8])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_10) + "\":" + addDQuotes(toString(channel[9])) + ",";
  data += "\"" + getIdFromName(NAME_BATTERY) + "\":" + addDQuotes(getVoltageStr()) + ",";  
  data += "\"" + getIdFromName(NAME_DISTANCE_LEFT) + "\":" + addDQuotes(String(distanceLeft)) + ",";
  data += "\"" + getIdFromName(NAME_DISTANCE_RIGHT) + "\":" + addDQuotes(String(distanceRight)) + ",";
  data += "\"" + getIdFromName(NAME_USED_UP_LOOPTIME) + "\":" + String(usedUpLoopTime) + ",";
  data += "\"" + getIdFromName(NAME_TEMPERATURE) + "\":" + addDQuotes(String(getTempCelsius(), 1));
  data += "}";
  //Serial.println(data);
  return data;
}


void initDogNeutralMicros() {
  dog.getFrontLeftLeg()->getShoulderJoint()->setOffsetNeutralMicros(-50);
  dog.getFrontLeftLeg()->getUpperLegJoint()->setOffsetNeutralMicros(-150);
  dog.getFrontLeftLeg()->getKneeJoint()->setOffsetNeutralMicros(-180);

  dog.getBackLeftLeg()->getShoulderJoint()->setOffsetNeutralMicros(20);
  dog.getBackLeftLeg()->getUpperLegJoint()->setOffsetNeutralMicros(-100);
  dog.getBackLeftLeg()->getKneeJoint()->setOffsetNeutralMicros(-130);
  
  dog.getFrontRightLeg()->getShoulderJoint()->setOffsetNeutralMicros(5);
  dog.getFrontRightLeg()->getUpperLegJoint()->setOffsetNeutralMicros(-50);
  dog.getFrontRightLeg()->getKneeJoint()->setOffsetNeutralMicros(10); 

  dog.getBackRightLeg()->getShoulderJoint()->setOffsetNeutralMicros(-50);
  dog.getBackRightLeg()->getUpperLegJoint()->setOffsetNeutralMicros(-100);
  dog.getBackRightLeg()->getKneeJoint()->setOffsetNeutralMicros(-30);
}


void initDogSpecs() {
  Leg frontLeftLeg(12, 13, 14, LENGTH_SHOULDER, LENGTH_UPPER_ARM, LENGTH_LOWER_ARM);
  frontLeftLeg.getShoulderJoint()->setReversed(true);
  frontLeftLeg.getKneeJoint()->setReversed(true);

  Leg backLeftLeg(8, 9, 10, LENGTH_SHOULDER, LENGTH_UPPER_ARM, LENGTH_LOWER_ARM);
  backLeftLeg.getKneeJoint()->setReversed(true);

  Leg frontRightLeg(0, 1, 2, LENGTH_SHOULDER, LENGTH_UPPER_ARM, LENGTH_LOWER_ARM);
  frontRightLeg.getUpperLegJoint()->setReversed(true);

  Leg backRightLeg(4, 5, 6, LENGTH_SHOULDER, LENGTH_UPPER_ARM, LENGTH_LOWER_ARM);
  backRightLeg.getShoulderJoint()->setReversed(true);
  backRightLeg.getUpperLegJoint()->setReversed(true);

  dog.setBackLeftLeg(backLeftLeg);
  dog.setFrontLeftLeg(frontLeftLeg);
  dog.setFrontRightLeg(frontRightLeg);
  dog.setBackRightLeg(backRightLeg);

  initDogNeutralMicros();

  //dog.printInfo();
}


void doWalkMethod1(int prmWalkAngleUpperLeg, int prmWalkAngleKnee, LegAngles *prmLegAngles) {
  prmLegAngles->frontLeftShoulderAngle = 0;
  prmLegAngles->frontRightShoulderAngle = 0;
  prmLegAngles->backLeftShoulderAngle = 0;
  prmLegAngles->backRightShoulderAngle = 0;

  prmLegAngles->frontLeftUpperLegAngle = NEUTRAL_WALK_ANGLE_UPPERLEG + prmWalkAngleUpperLeg;
  prmLegAngles->frontRightUpperLegAngle = NEUTRAL_WALK_ANGLE_UPPERLEG - prmWalkAngleUpperLeg;
  prmLegAngles->backLeftUpperLegAngle = NEUTRAL_WALK_ANGLE_UPPERLEG - prmWalkAngleUpperLeg;
  prmLegAngles->backRightUpperLegAngle = NEUTRAL_WALK_ANGLE_UPPERLEG + prmWalkAngleUpperLeg;

  prmLegAngles->frontLeftKneeAngle = NEUTRAL_WALK_ANGLE_KNEE + prmWalkAngleKnee;
  prmLegAngles->frontRightKneeAngle = NEUTRAL_WALK_ANGLE_KNEE - prmWalkAngleKnee;
  prmLegAngles->backLeftKneeAngle = NEUTRAL_WALK_ANGLE_KNEE - prmWalkAngleKnee;
  prmLegAngles->backRightKneeAngle = NEUTRAL_WALK_ANGLE_KNEE + prmWalkAngleKnee;
}


