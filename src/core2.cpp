#include <functions.h>


unsigned long loopTimer, batteryVoltageTimer;
int lowVoltageAlarmCount;
bool previousIsArmed = false;




void setup2() {
  Wire.begin();
  Wire.setClock(1000000);
  Wire1.begin(SDA_2_PIN, SCL_2_PIN);
  Wire1.setClock(1000000);  

  displayFound = isDisplayFound();
  Serial.print("displayFound=");
  Serial.println(displayFound);

  mpu_6050_found = is_mpu_6050_found();
  Serial.print("mpu_6050_found=");
  Serial.println(mpu_6050_found);

  if (mpu_6050_found) {
    Serial.println("calibrating gyro");
    setup_mpu_6050_registers();
    calibrate_mpu_6050();
  }

  // servos
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  //dog.setNeutral(); 
  //dog.setNeutralWalk(); 
  dog.powerOff();

  lowVoltageAlarmCount = 0;

  IBus.begin(hs, IBUSBM_NOTIMER); 

  usedUpLoopTime = 0;
  batteryVoltageTimer = millis();
  loopTimer = micros();    
}


void loop2() {
  unsigned long millisTimer = millis();

  IBus.loop();
  readChannels();
  //printChannels();

  int throttleSignal = channel[2]; 
  bool is_signal_detected = !isSignalLostDetected(throttleSignal);
  if (is_signal_detected) {
      signal_detected_count++;      
      if (signal_detected_count == SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected = true;
        playSignalDetected();
      } else if (signal_detected_count > SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected_count = SIGNALS_DETECTED_LOST_THRESHOLD;
        signal_lost_count = 0;        
      }
  } else {
      signal_lost_count++;      
      if (signal_lost_count == SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected = false;
        playSignalLost();
      } else if (signal_lost_count > SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_lost_count = SIGNALS_DETECTED_LOST_THRESHOLD;
        signal_detected_count = 0;
      }
  }

  dogMode = getDogMode();

  int rollChannel = fixChannelDirection(channel[0], rollChannelReversed);
  int pitchChannel = fixChannelDirection(channel[1], pitchChannelReversed);
  int throttleChannel = fixChannelDirection(channel[2], throttleChannelReversed);
  int yawChannel = fixChannelDirection(channel[3], yawChannelReversed);

  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  //print_gyro_values();

  bool actualIsArmed = isArmed();
  if (actualIsArmed && !previousIsArmed) {
    playArmed();
    dog.setNeutralWalk();
  } else if (!actualIsArmed && previousIsArmed) {
    playDisarmed();
    dog.setNeutralWalk();
    dog.powerOff();
  }
  previousIsArmed = actualIsArmed;

  if (actualIsArmed) {
    int angle0 = map(rollChannel, MID_CHANNEL-SERVO_HALF_PULSE_RANGE, MID_CHANNEL+SERVO_HALF_PULSE_RANGE, -MAX_HALF_SERVO_ANGLE, MAX_HALF_SERVO_ANGLE);
    int angle1 = map(pitchChannel, MID_CHANNEL-SERVO_HALF_PULSE_RANGE, MID_CHANNEL+SERVO_HALF_PULSE_RANGE, -MAX_HALF_SERVO_ANGLE, MAX_HALF_SERVO_ANGLE);
    int angle2 = map(throttleChannel, MID_CHANNEL-SERVO_HALF_PULSE_RANGE, MID_CHANNEL+SERVO_HALF_PULSE_RANGE, -MAX_HALF_SERVO_ANGLE, MAX_HALF_SERVO_ANGLE); 
    int angle3 = map(yawChannel, MID_CHANNEL-SERVO_HALF_PULSE_RANGE, MID_CHANNEL+SERVO_HALF_PULSE_RANGE, -MAX_HALF_SERVO_ANGLE, MAX_HALF_SERVO_ANGLE);
    
    if (dogMode == dmManual) {
      LegAngles legAngles;
      legAngles.frontLeftShoulderAngle = angle0 + angle3;
      legAngles.backLeftShoulderAngle = angle0 - angle3;    
      legAngles.frontRightShoulderAngle = -angle0 - angle3;
      legAngles.backRightShoulderAngle = -angle0 + angle3;

      legAngles.frontLeftUpperLegAngle = angle1 + NEUTRAL_WALK_ANGLE_UPPERLEG;
      legAngles.backLeftUpperLegAngle = angle1 + NEUTRAL_WALK_ANGLE_UPPERLEG;
      legAngles.frontRightUpperLegAngle = angle1 + NEUTRAL_WALK_ANGLE_UPPERLEG;
      legAngles.backRightUpperLegAngle = angle1 + NEUTRAL_WALK_ANGLE_UPPERLEG;

      legAngles.frontLeftKneeAngle = angle1 + NEUTRAL_WALK_ANGLE_KNEE;
      legAngles.backLeftKneeAngle = angle1 + NEUTRAL_WALK_ANGLE_KNEE;
      legAngles.frontRightKneeAngle = angle1 + NEUTRAL_WALK_ANGLE_KNEE;
      legAngles.backRightKneeAngle = angle1 + NEUTRAL_WALK_ANGLE_KNEE;

      if (angle2 > -MAX_HALF_SERVO_ANGLE + 5) {
        static int walkCount = 0;
        static int direction = 1;
        int maxWalkAngleUpperLeg = 10;
        int maxWalkAngleKnee = 10;        
        int speed = map(angle2, -MAX_HALF_SERVO_ANGLE, MAX_HALF_SERVO_ANGLE, 100, 15);

        if (walkCount > speed) {
          walkCount = speed;
          direction = -direction;
        }
        if (walkCount <= -speed) {
          walkCount = -speed;
          direction = -direction;
        }        

        int walkAngleUpperLeg = map(walkCount, 0, speed, 0, maxWalkAngleUpperLeg);
        int walkAngleKnee = map(walkCount, 0, speed, 0, maxWalkAngleKnee);

        doWalkMethod1(walkAngleUpperLeg, walkAngleKnee, &legAngles);

        if (direction > 0) {   
          walkCount++;       
        } else {
          walkCount--;
        }
      }

      dog.setShoulderAngles(legAngles.frontLeftShoulderAngle, legAngles.frontRightShoulderAngle, legAngles.backLeftShoulderAngle, legAngles.backRightShoulderAngle);       
      dog.setUpperLegAngles(legAngles.frontLeftUpperLegAngle, legAngles.frontRightUpperLegAngle, legAngles.backLeftUpperLegAngle, legAngles.backRightUpperLegAngle);       
      dog.setKneeAngles(legAngles.frontLeftKneeAngle, legAngles.frontRightKneeAngle, legAngles.backLeftKneeAngle, legAngles.backRightKneeAngle);
    } else if (dogMode == dmSit) {
      dog.sit(); 
    } else {
      dog.crawl(); 
    }
  }

  // check battery voltage once per second
  if ((millisTimer - batteryVoltageTimer) > 1000) {
    batteryVoltageTimer = millisTimer;
    //Serial.println(voltage);     
    if (voltage < LOW_VOLTAGE_ALARM) {
//      Serial.println("lowVoltageAlarmCount++"); 
      lowVoltageAlarmCount++;
      if (lowVoltageAlarmCount >= 10) {
//        Serial.println("lowVoltageAlarm!!"); 
//        playLowVoltageAlarm(); 
      }
    } else {
      lowVoltageAlarmCount = 0;
    }
  }    

//  Serial.print("distance: ");
//  Serial.print(distanceLeft);
//  Serial.print("\t");
//  Serial.print(distanceRight);
//  Serial.println("cm");

  rtttl::play();

  usedUpLoopTime = micros() - loopTimer;
  /*
  Serial.print(usedUpLoopTime);
  Serial.println();
  */
  while(micros() - loopTimer < LOOP_TIME) {
    vTaskDelay(1);
  };
  loopTimer = micros();
}


void runOnCore2(void *parameter) {
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());
  setup2();
  for (;;) {
    loop2();
  }
}
