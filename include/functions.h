#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <NonBlockingRtttl.h>
#include <ArduinoUniqueID.h>
#include <Adafruit_PWMServoDriver.h>
#include <FS.h>
#include <SPIFFS.h>
#include <..\..\MyCommon\Credentials.h>
#include <Ultrasonic.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <IBusBM.h>
#include <HardwareSerial.h>



#define STACK_SIZE_CORE2        30000

#define SPOTMICRO_VERSION       "1.1"

#define FORMAT_SPIFFS_IF_FAILED true

#define LOOP_TIME               4000   // microseconds; 250Hz
#define LOOP_TIME_HZ            (1000000.0/LOOP_TIME)

#define PPM_PULSE_TRAIN_PERIOD  5000   // microsec
#define NR_OF_RECEIVER_CHANNELS 10     //FS-IA6B (i-bus mode)

#define SDA_2_PIN               27
#define SCL_2_PIN               14 
#define BUZZER_PIN              23
#define VOLTAGE_SENSOR_PIN      35
#define TRIGGER_PIN_RIGHT       32
#define ECHO_PIN_RIGHT          33
#define TRIGGER_PIN_LEFT        25
#define ECHO_PIN_LEFT           26

#define SCREEN_WIDTH            128
#define SCREEN_HEIGHT           64
#define OLED_RESET              -1
#define OLED_ADDR               0x3C

#define GYRO_CALIBRATION_COUNT  250

#define MID_CHANNEL             1500

#define SERVO_ANGLE_RANGE       120 
#define MAX_HALF_SERVO_ANGLE    (SERVO_ANGLE_RANGE/2)

#define SERVO_PULSE_RANGE       1000
#define SERVO_HALF_PULSE_RANGE  (SERVO_PULSE_RANGE/2)

#define MIN_PULSE                1000
#define MAX_PULSE                2000
#define SIGNAL_LOST_PULSE         950
#define INVALID_SIGNAL_PULSE      800
#define SIGNALS_DETECTED_LOST_THRESHOLD   50 


#define NEUTRAL_WALK_ANGLE_UPPERLEG     0.1*MAX_HALF_SERVO_ANGLE
#define NEUTRAL_WALK_ANGLE_KNEE         0.1*MAX_HALF_SERVO_ANGLE

#define CRAWL_ANGLE_UPPERLEG            1.2*MAX_HALF_SERVO_ANGLE
#define CRAWL_ANGLE_KNEE                1.2*MAX_HALF_SERVO_ANGLE

#define SIT_ANGLE_FRONT_UPPERLEG        -0.2*MAX_HALF_SERVO_ANGLE
#define SIT_ANGLE_FRONT_KNEE            -0.5*MAX_HALF_SERVO_ANGLE
#define SIT_ANGLE_BACK_UPPERLEG         1.2*MAX_HALF_SERVO_ANGLE
#define SIT_ANGLE_BACK_KNEE             1.5*MAX_HALF_SERVO_ANGLE


#define SERVO_FREQ              50 // Analog servos run at ~50 Hz updates


#define LENGTH_SHOULDER         55
#define LENGTH_UPPER_ARM        115
#define LENGTH_LOWER_ARM        140


#define BATTERY_NOICE_FILTER        0.92

#define SSID_BASE                   "SPOTMICRO_ESP32_"


#define NAME_MODEL                  "Model"
#define NAME_VERSION                "Version"
#define NAME_CHANNEL_1              "Channel 1 (Roll)"
#define NAME_CHANNEL_2              "Channel 2 (Pitch)"
#define NAME_CHANNEL_3              "Channel 3 (Throttle)"
#define NAME_CHANNEL_4              "Channel 4 (Yaw)"
#define NAME_CHANNEL_5              "Channel 5 (Aux 1)"
#define NAME_CHANNEL_6              "Channel 6 (Aux 2)"
#define NAME_CHANNEL_7              "Channel 7 (Aux 3)"
#define NAME_CHANNEL_8              "Channel 8 (Aux 4)"
#define NAME_CHANNEL_9              "Channel 9 (Aux 5)"
#define NAME_CHANNEL_10             "Channel 10 (Aux 6)"
#define NAME_SIGNAL_DETECTED        "Signal Detected"
#define NAME_ARMED                  "Armed"
#define NAME_MODE                   "Mode"
#define NAME_BATTERY                "Battery [volt]"
#define NAME_BATTERY_PROGRESS       "Battery"
#define NAME_DISTANCE_LEFT          "Distance Left [cm]"
#define NAME_DISTANCE_RIGHT         "Distance Right [cm]"

#define NAME_USED_UP_LOOPTIME       "Used up Looptime [us]"
#define NAME_TEMPERATURE            "Temperature [C]"

#define NAME_VOLTAGE_CORRECTION     "Voltage Correction Factor"

#define NAME_FRONT_LEFT_LEG         "Front Left"
#define NAME_BACK_LEFT_LEG          "Back Left"
#define NAME_FRONT_RIGHT_LEG        "Front Right"
#define NAME_BACK_RIGHT_LEG         "Back Right"


#define ID_PROGRESS_BATTERY         "progressID_bat"
#define ID_PROGRESS_CHANNEL_1       "progressID_ch1"
#define ID_PROGRESS_CHANNEL_2       "progressID_ch2"
#define ID_PROGRESS_CHANNEL_3       "progressID_ch3"
#define ID_PROGRESS_CHANNEL_4       "progressID_ch4"
#define ID_PROGRESS_CHANNEL_5       "progressID_ch5"
#define ID_PROGRESS_CHANNEL_6       "progressID_ch6"
#define ID_PROGRESS_CHANNEL_7       "progressID_ch7"
#define ID_PROGRESS_CHANNEL_8       "progressID_ch8"
#define ID_PROGRESS_CHANNEL_9       "progressID_ch9"
#define ID_PROGRESS_CHANNEL_10       "progressID_ch10"
#define ID_SPAN_PROGRESS_CHANNEL_1  "progressSpanID_ch1"
#define ID_SPAN_PROGRESS_CHANNEL_2  "progressSpanID_ch2"
#define ID_SPAN_PROGRESS_CHANNEL_3  "progressSpanID_ch3"
#define ID_SPAN_PROGRESS_CHANNEL_4  "progressSpanID_ch4"
#define ID_SPAN_PROGRESS_CHANNEL_5  "progressSpanID_ch5"
#define ID_SPAN_PROGRESS_CHANNEL_6  "progressSpanID_ch6"
#define ID_SPAN_PROGRESS_CHANNEL_7  "progressSpanID_ch7"
#define ID_SPAN_PROGRESS_CHANNEL_8  "progressSpanID_ch8"
#define ID_SPAN_PROGRESS_CHANNEL_9  "progressSpanID_ch9"
#define ID_SPAN_PROGRESS_CHANNEL_10 "progressSpanID_ch10"


#define WEBPAGE_REFRESH_INTERVAL    "250"
#define WEBPAGE_TIMEOUT             "200"


const uint8_t BREADBOARD_SPOT[UniqueIDsize] = {0xF0, 0x08, 0xD1, 0xD2, 0x5C, 0xF4};
const uint8_t YELLOW_SPOT[UniqueIDsize] = {0xF0, 0x08, 0xD1, 0xD3, 0x1E, 0x4C};

const float LOW_VOLTAGE_ALARM = 3.5;
const float FULLY_CHARGED_VOLTAGE = 4.2;
const float WARNING_VOLTAGE = 3.8;

const double defaultVoltageCorrectionFactor = 1.0;

const boolean rollChannelReversed = false;
const boolean pitchChannelReversed = false;
const boolean throttleChannelReversed = false;
const boolean yawChannelReversed = false;


extern IPAddress *ipAddress;
extern volatile long usedUpLoopTime;
extern short gyro_x, gyro_y, gyro_z;
extern short acc_x, acc_y, acc_z;
extern volatile short temperature;
extern long gyro_x_cal, gyro_y_cal, gyro_z_cal;
extern bool mpu_6050_found;
extern bool displayFound;
extern String robotName;
extern volatile bool signal_detected;
extern int signal_detected_count;
extern int signal_lost_count;
extern Adafruit_PWMServoDriver pwm;
extern volatile int channel[];
extern volatile float voltage;
extern volatile bool buzzerDisabled;
extern volatile double voltageCorrectionFactor;
extern volatile unsigned int distanceLeft, distanceRight;

extern Ultrasonic ultrasonicLeft;	
extern Ultrasonic ultrasonicRight;	
extern Adafruit_SSD1306 display;

extern HardwareSerial hs;
extern IBusBM IBus; 


class LegAngles {
    public:
        int frontLeftShoulderAngle;
        int frontRightShoulderAngle;
        int backLeftShoulderAngle;
        int backRightShoulderAngle;
        int frontLeftUpperLegAngle;
        int frontRightUpperLegAngle;
        int backLeftUpperLegAngle;
        int backRightUpperLegAngle;
        int frontLeftKneeAngle;
        int frontRightKneeAngle;
        int backLeftKneeAngle;
        int backRightKneeAngle;

        LegAngles() {}
};

class TwoPosSwitch {
    private:
        int channelNr;
    public:
        TwoPosSwitch() {}
        TwoPosSwitch(int prmChannenNr) : channelNr(prmChannenNr)  {}
        int readPos() { return (channel[channelNr] < MID_CHANNEL) ? 1 : 2; }
};

class ThreePosSwitch {
    private:
        int channelNr;
    public:
        ThreePosSwitch() {}
        ThreePosSwitch(int prmChannenNr) : channelNr(prmChannenNr)  {}
        int readPos() { return (channel[channelNr] < 1250) ? 1 : (channel[channelNr] > 1750) ? 3 : 2; }
};

class Joint {
    private:
        int servoNr;        
        int offsetNeutralMicroseconds;
        bool reversed;

        int getMinPulse() { return MID_CHANNEL+offsetNeutralMicroseconds-SERVO_HALF_PULSE_RANGE; }
        int getMaxPulse() { return MID_CHANNEL+offsetNeutralMicroseconds+SERVO_HALF_PULSE_RANGE; }
        int convertAngleToMicroseconds(int prmAngle) { return reversed ? map(prmAngle, MAX_HALF_SERVO_ANGLE, -MAX_HALF_SERVO_ANGLE, getMinPulse(), getMaxPulse()) : map(prmAngle, -MAX_HALF_SERVO_ANGLE, MAX_HALF_SERVO_ANGLE, getMinPulse(), getMaxPulse()); }
    public:
        Joint() {}
        Joint(int prmServoNr) : servoNr(prmServoNr) { offsetNeutralMicroseconds = 0; reversed = false; }
        
        void loadProps(File prmF) { offsetNeutralMicroseconds = prmF.readStringUntil('\n').toInt(); }
        void saveProps(File prmF) { prmF.println(String(offsetNeutralMicroseconds)); }

        void setOffsetNeutralMicros(int prmOffsetNeutralMicroseconds) { offsetNeutralMicroseconds = prmOffsetNeutralMicroseconds; }
        int getOffsetNeutralMicros() { return offsetNeutralMicroseconds; }
        void setReversed(bool prmReversed) { reversed = prmReversed; }

        void printOffsetNeutralMicros() { Serial.print(offsetNeutralMicroseconds); }
        void printInfo() { Serial.print("\t\t\tservoNr="); Serial.println(servoNr); Serial.print("\t\t\toffsetNeutralMicroseconds="); Serial.println(offsetNeutralMicroseconds); Serial.print("\t\t\treversed="); Serial.println(reversed); }
        void powerOff() { writeMicroseconds(0); }
        void setNeutral() { pwm.writeMicroseconds(servoNr, MID_CHANNEL + offsetNeutralMicroseconds); }
        void writeMicroseconds(int prmMicroseconds) { pwm.writeMicroseconds(servoNr, prmMicroseconds); } 
        void setAngle(int prmAngle) { writeMicroseconds(convertAngleToMicroseconds(prmAngle)); }
};

class Leg {
    private:
        Joint shoulder;
        Joint upperLeg;        
        Joint knee;
        int lengthShoulder;
        int lengthUpperLeg;
        int lengthLowerLeg;

    public:
        Leg() {}
        Leg(int prmShoulderServoNr, int prmUpperLegServoNr, int prmKneeServoNr, int prmLengthShoulder, int prmLengthUpperLeg, int prmLengthLowerLeg) : 
            shoulder(prmShoulderServoNr), upperLeg(prmUpperLegServoNr), knee(prmKneeServoNr), lengthShoulder(prmLengthShoulder), lengthUpperLeg(prmLengthUpperLeg), lengthLowerLeg(prmLengthLowerLeg) {}

        Joint *getShoulderJoint() { return &shoulder; }
        Joint *getUpperLegJoint() { return &upperLeg; }
        Joint *getKneeJoint() { return &knee; }      

        void loadProps(File prmF) { shoulder.loadProps(prmF); upperLeg.loadProps(prmF); knee.loadProps(prmF); }
        void saveProps(File prmF) { shoulder.saveProps(prmF); upperLeg.saveProps(prmF); knee.saveProps(prmF); }
        void setOffsetNeutralMicros(int prmShoulderOffsetNeutralMicroseconds, int prmUpperLegOffsetNeutralMicroseconds, int prmKneeOffsetNeutralMicroseconds) { shoulder.setOffsetNeutralMicros(prmShoulderOffsetNeutralMicroseconds); upperLeg.setOffsetNeutralMicros(prmUpperLegOffsetNeutralMicroseconds); knee.setOffsetNeutralMicros(prmKneeOffsetNeutralMicroseconds); }
        void printOffsetNeutralMicros() { shoulder.printOffsetNeutralMicros(); Serial.print("\t"); upperLeg.printOffsetNeutralMicros(); Serial.print("\t"); knee.printOffsetNeutralMicros(); Serial.println(); }       
        void printInfo() { Serial.println("\t\tShoulder:"); shoulder.printInfo(); Serial.println("\t\tUpperLeg:"); upperLeg.printInfo(); Serial.println("\t\tKnee:"); knee.printInfo(); }       
        void powerOff() { shoulder.powerOff(); upperLeg.powerOff(); knee.powerOff(); }
        void setAllJointsNeutral() { knee.setNeutral(); upperLeg.setNeutral(); shoulder.setNeutral(); }
        void setShoulderNeutral() { shoulder.setNeutral(); }
        void setUpperLegNeutral() { upperLeg.setNeutral(); }
        void setKneeNeutral() { knee.setNeutral(); }
        void setNeutralWalk() { shoulder.setNeutral(); upperLeg.setAngle(NEUTRAL_WALK_ANGLE_UPPERLEG); knee.setAngle(NEUTRAL_WALK_ANGLE_KNEE); }        
        void sit(int prmUpperLegAngle, int prmKneeAngle) { shoulder.setNeutral(); upperLeg.setAngle(prmUpperLegAngle); knee.setAngle(prmKneeAngle); }
        void crawl() { shoulder.setNeutral(); upperLeg.setAngle(CRAWL_ANGLE_UPPERLEG); knee.setAngle(CRAWL_ANGLE_KNEE); }

        void setShoulderAngle(int prmAngle) { shoulder.setAngle(prmAngle); }
        void setUpperLegAngle(int prmAngle) { upperLeg.setAngle(prmAngle); }
        void setKneeAngle(int prmAngle) { knee.setAngle(prmAngle); }
        void setAngles(int prmShoulderAngle, int prmUpperLegAngle, int prmKneeAngle) { shoulder.setAngle(prmShoulderAngle); upperLeg.setAngle(prmUpperLegAngle); knee.setAngle(prmKneeAngle); }
};

class Dog {
    private:
        Leg frontLeftLeg;
        Leg frontRightLeg;
        Leg backLeftLeg;
        Leg backRightLeg;

    public:
        Dog() {}

        void setFrontLeftLeg(Leg prmFrontLeftLeg) { frontLeftLeg = prmFrontLeftLeg; }
        void setFrontRightLeg(Leg prmFrontRightLeg) { frontRightLeg = prmFrontRightLeg; }
        void setBackLeftLeg(Leg prmBackLeftLeg) { backLeftLeg = prmBackLeftLeg; }
        void setBackRightLeg(Leg prmBackRightLeg) { backRightLeg = prmBackRightLeg; }
        Leg *getFrontLeftLeg() { return &frontLeftLeg; }
        Leg *getFrontRightLeg() { return &frontRightLeg; }
        Leg *getBackLeftLeg() { return &backLeftLeg; }
        Leg *getBackRightLeg() { return &backRightLeg; }

        void loadProps(File prmF) { frontLeftLeg.loadProps(prmF); backLeftLeg.loadProps(prmF); frontRightLeg.loadProps(prmF); backRightLeg.loadProps(prmF);}
        void saveProps(File prmF) { frontLeftLeg.saveProps(prmF); backLeftLeg.saveProps(prmF); frontRightLeg.saveProps(prmF); backRightLeg.saveProps(prmF);}
        void printOffsetNeutralMicros() { frontLeftLeg.printOffsetNeutralMicros(); backLeftLeg.printOffsetNeutralMicros(); frontRightLeg.printOffsetNeutralMicros(); backRightLeg.printOffsetNeutralMicros();}
        void printInfo() { Serial.println("Dog:"); Serial.println("\tFrontLeftLeg:"); frontLeftLeg.printInfo(); Serial.println("\tFrontRightLeg:"); frontRightLeg.printInfo(); Serial.println("\tBackLeftLeg:"); backLeftLeg.printInfo(); Serial.println("\tBackRightLeg:"); backRightLeg.printInfo();}
        void powerOff() { frontLeftLeg.powerOff(); frontRightLeg.powerOff(); backLeftLeg.powerOff(); backRightLeg.powerOff(); }
        void setShouldersNeutral() { frontLeftLeg.setShoulderNeutral(); frontRightLeg.setShoulderNeutral(); backLeftLeg.setShoulderNeutral(); backRightLeg.setShoulderNeutral(); }
        void setUpperLegsNeutral() { frontLeftLeg.setUpperLegNeutral(); frontRightLeg.setUpperLegNeutral(); backLeftLeg.setUpperLegNeutral(); backRightLeg.setUpperLegNeutral(); }
        void setKneesNeutral() { frontLeftLeg.setKneeNeutral(); frontRightLeg.setKneeNeutral(); backLeftLeg.setKneeNeutral(); backRightLeg.setKneeNeutral(); }
        void setNeutral(int prmDelay = 0) { setShouldersNeutral(); delay(prmDelay); setUpperLegsNeutral(); delay(prmDelay); setKneesNeutral(); }
        void setNeutralWalk() { frontLeftLeg.setNeutralWalk(); frontRightLeg.setNeutralWalk(); backLeftLeg.setNeutralWalk(); backRightLeg.setNeutralWalk(); }        
        void sit() { frontLeftLeg.sit(SIT_ANGLE_FRONT_UPPERLEG, SIT_ANGLE_FRONT_KNEE); frontRightLeg.sit(SIT_ANGLE_FRONT_UPPERLEG, SIT_ANGLE_FRONT_KNEE); backLeftLeg.sit(SIT_ANGLE_BACK_UPPERLEG, SIT_ANGLE_BACK_KNEE); backRightLeg.sit(SIT_ANGLE_BACK_UPPERLEG, SIT_ANGLE_BACK_KNEE); }
        void crawl() { frontLeftLeg.crawl(); frontRightLeg.crawl(); backLeftLeg.crawl(); backRightLeg.crawl(); }        
        void setShoulderAngles(int prmFrontLeftAngle, int prmFrontRightAngle, int prmBackLeftAngle, int prmBackRightAngle) { frontLeftLeg.setShoulderAngle(prmFrontLeftAngle); frontRightLeg.setShoulderAngle(prmFrontRightAngle); backLeftLeg.setShoulderAngle(prmBackLeftAngle); backRightLeg.setShoulderAngle(prmBackRightAngle); }
        void setUpperLegAngles(int prmFrontLeftAngle, int prmFrontRightAngle, int prmBackLeftAngle, int prmBackRightAngle) { frontLeftLeg.setUpperLegAngle(prmFrontLeftAngle); frontRightLeg.setUpperLegAngle(prmFrontRightAngle); backLeftLeg.setUpperLegAngle(prmBackLeftAngle); backRightLeg.setUpperLegAngle(prmBackRightAngle); }
        void setKneeAngles(int prmFrontLeftAngle, int prmFrontRightAngle, int prmBackLeftAngle, int prmBackRightAngle) { frontLeftLeg.setKneeAngle(prmFrontLeftAngle); frontRightLeg.setKneeAngle(prmFrontRightAngle); backLeftLeg.setKneeAngle(prmBackLeftAngle); backRightLeg.setKneeAngle(prmBackRightAngle); }
};


enum DogMode { dmManual, dmSit, dmCrawl };

extern DogMode dogMode;


extern TwoPosSwitch switchA;
extern TwoPosSwitch switchB;
extern TwoPosSwitch switchD;
extern ThreePosSwitch switchC;
extern Dog dog;


extern double lowPassFilter(const double prmAlpha, const double prmCurrentValue, const double prmPreviousValue);
extern double averageFilter(const double prmCurrentValue);
extern int checkCenterOffset(const int prmCenterOffset);
extern void printProps();
extern void loadProps();
extern void saveProps();
extern String identifyRobot();
extern bool isValidSignal(int prmPulse);
extern float readVoltage();
extern String getVoltageStr();
extern int fixChannelDirection(int prmChannel, boolean prmReversed);
extern bool isDisplayFound();
extern bool is_mpu_6050_found();
extern void setup_mpu_6050_registers();
extern void read_mpu_6050_data();
extern void calibrate_mpu_6050();
extern float getTempCelsius();
extern void print_gyro_values();
extern bool isArmed();
extern DogMode getDogMode();
extern void playShortBeep();
extern void playLowVoltageAlarm();
extern void playArmed();
extern void playDisarmed();
extern void playSignalDetected();
extern void playSignalLost();
extern void servoTest();
extern void readChannels();
extern bool isSignalLostDetected(int prmPulse);
extern void printChannels();
extern void runOnCore2(void *parameter);
extern void updateDisplay();
extern String getSSID();
extern uint8_t* getChannelWithStrongestSignal(String prmSSID, int32_t *prmStrongestChannel);
extern String getIdFromName(String prmName);
extern String getIdFromName(String prmName, int prmIndex);
extern bool isAPStarted();
extern void WiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info);
extern String getWebPage();
extern String getLatestData();
extern void initDogNeutralMicros();
extern void initDogSpecs();
extern void doWalkMethod1(int prmWalkAngleUpperLeg, int prmWalkAngleKnee, LegAngles *prmLegAngles);




#endif