#define KINEMATRIX_ENABLE_FIREBASE
#include "Kinematrix.h"
#include "index/all.h"
#include "Preferences.h"
#include "WiFiManager.h"
#include "ESP32Servo.h"
#include "PCF8574.h"

////////// Utility //////////
WiFiManager wm;
TaskHandle task;
FirebaseModule firebase;
FirebaseAuthentication auth;
TimerTask firebaseTimer(20000);
Preferences preferences;
TimerDuration waterPumpTimer;

////////// Sensor //////////
SensorModule sensor;
AnalogCalibration phCalibration("phCalibration", &preferences);
AnalogCalibration wlCalibration("wlCalibration", &preferences);
MovingAverageFilter phFilter(50);
MovingAverageFilter heightFilter(30);

////////// Communication //////////
HardSerial usbSerial;

////////// Input Module //////////
PCF8574 buttonI2C(0x20);
DigitalInI2C buttonDown(P1);
DigitalInI2C buttonOk(P2);

////////// Output Module //////////
LcdMenu menu(0x27, 16, 2);
DigitalOut buzzer(4);
DigitalOut waterPump(18, true);
DigitalOut waterSolenoid(19, true);

Servo servoPhUp;
Servo servoPhDown;
Servo servoSalt;

////////// Global Variable //////////
struct CalibrationData {
  float voltage;
  float calibrationValue;
};

CalibrationData *calibrationDataArray = nullptr;
int calibrationDataCount = 0;

struct SystemData {
  String statusTurbidity;
  String statusWaterLevel1;
  String statusWaterLevel2;
  String statusWaterLevel3;

  float turbidity;
  float height;
  float ph;
  float temperature;
  float waterLevel1;
  float waterLevel2;
  float waterLevel3;
  int sensorDebug;
  String buttonStatus = "";

  int systemEnable = 0;

  int waterPumpState = 0;
  int waterPumpTrigger = 0;

  int phUpState = 0;
  int phUpTrigger = 0;
  int phUpServoOn = 60;
  int phUpServoOff = 140;

  int phDownState = 0;
  int phDownTrigger = 0;
  int phDownServoOn = 0;
  int phDownServoOff = 90;

  int saltState = 0;
  int saltTrigger = 0;
  int saltServoOn = 0;
  int saltServoOff = 90;
};

SystemData var;

#include "Header.h"

void setup() {
  usbSerial.begin(&Serial, 115200);

  menu.initialize();
  menu.setLen(16, 2);
  auto initMenu = menu.createMenu(2, "  System  ", "Initialize");
  menu.showMenu(initMenu, true);
  menu.freeMenu(initMenu);

  task.initialize(wifiConTask);

  initServo();

  phCalibration.setAnalogConfig(32, 3.3, 4095);
  wlCalibration.setAnalogConfig(34, 3.3, 4095);

  phCalibration.loadCalibration();
  wlCalibration.loadCalibration();

  sensor.addModule("temp", new DS18B20Sens(26));
  sensor.addModule("sonar", new UltrasonicSens(33, 25, 200));
  sensor.addModule("ph", []() -> BaseSens* {
    auto ph = new AnalogSens(32, 3.3, 4095.0, [](JsonVariant value, int analogValue, float voltage) {
      // value["val"] = phCalibration.voltageToValue(voltage, LINEAR_INTERPOLATION);
      phFilter.addMeasurement(voltage);
      float filteredValue = phFilter.getFilteredValue();
      value["avg"] = filteredValue;
      value["val"] = regressPh(filteredValue);
    });
    return ph;
  });
  sensor.addModule("turbid", []() -> BaseSens* {
    auto turbid = new AnalogSens(35, 3.3, 4095.0, [](JsonVariant value, int analogValue, float voltage) {
      value["val"] = voltage < 1.2 ? "Keruh" : "Jernih";  // 1.50, 0.81
    });
    return turbid;
  });
  sensor.addModule("wl1", new AnalogSens(34, 3.3, 4095.0));
  sensor.addModule("wl2", new AnalogSens(39, 3.3, 4095.0));
  sensor.addModule("wl3", new AnalogSens(36, 3.3, 4095.0));
  sensor.init();

  waterPumpTimer.setDuration(10000);
  waterPumpTimer.reset();
  waterPumpTimer.start();

  buttonI2C.pinMode(P1, INPUT_PULLUP);
  buttonI2C.pinMode(P2, INPUT_PULLUP);
  buttonI2C.begin();
  buttonDown.init(&buttonI2C);
  buttonOk.init(&buttonI2C);
  buttonDown.setDebounceTime(75);
  buttonOk.setDebounceTime(75);

  buzzer.toggleInit(100, 5);
}

void loop() {
  sensor.update([]() {
    debug();
    static uint32_t heightAverageTimer;
    if (millis() - heightAverageTimer >= 100) {
      var.height = sensor["sonar"];
      var.height = 26 - var.height;
      heightFilter.addMeasurement(var.height);
      var.height = round(heightFilter.getFilteredValue());
      var.height = var.height < 0 ? 0 : var.height;
      heightAverageTimer = millis();
    }
    var.turbidity = sensor["turbid"]["volt"];
    var.ph = sensor["ph"]["val"];
    var.temperature = sensor["temp"];
    var.waterLevel1 = sensor["wl1"]["volt"];
    var.waterLevel2 = sensor["wl2"]["volt"];
    var.waterLevel3 = sensor["wl3"]["volt"];

    var.statusTurbidity = var.turbidity < 1.2 ? "Keruh" : "Jernih";
    var.statusWaterLevel1 = var.waterLevel1 <= 0.5 ? "Habis" : "Penuh";
    var.statusWaterLevel2 = var.waterLevel2 <= 0.5 ? "Habis" : "Penuh";
    var.statusWaterLevel3 = var.waterLevel3 <= 0.5 ? "Habis" : "Penuh";

    var.waterLevel1 = wlCalibration.voltageToValue(var.waterLevel1, LINEAR_INTERPOLATION);
    var.waterLevel2 = wlCalibration.voltageToValue(var.waterLevel2, LINEAR_INTERPOLATION);
    var.waterLevel3 = wlCalibration.voltageToValue(var.waterLevel3, LINEAR_INTERPOLATION);
  });

  usbSerial.receive(usbCommunicationTask);

  if (buttonDown.isPressed() || buttonOk.isPressed()) {
    buzzer.on();
    buzzer.offDelay(75);
  }

  MenuCursor cursor{
    .up = false,
    .down = buttonDown.isPressed(),
    .select = buttonOk.isPressed(),
    .back = false,
    .show = true
  };
  menu.onListen(&cursor, menuTaskCallback);

  DigitalOut::updateAll(&buzzer, &waterPump, &waterSolenoid, DigitalOut::stop());
  DigitalIn::updateAll(&buttonDown, &buttonOk, DigitalIn::stop());

  if (!var.systemEnable) {
    waterPump.off();
    waterSolenoid.off();
    return;
  }

  // var.waterPumpTrigger = ;

  if (var.waterPumpState == 0) {
    if (waterPumpTimer.isExpired()) {
      if (var.waterPumpTrigger) {
        var.waterPumpState = 1;
        buzzer.off();
        buzzer.toggleInit(100, 2);
        return;
      }
    }
    /////////////////// HEIGHT ///////////////////
    // if (var.height < 14) {
    //   if (waterPumpTimer.isExpired()) {
    //     waterPump.on();
    //     waterSolenoid.off();
    //   }
    // } else if (var.height > 15) {
    //   if (waterPumpTimer.isExpired()) {
    //     waterPump.off();
    //     waterSolenoid.on();
    //   }
    // } else {
    //   waterPump.off();
    //   waterSolenoid.off();
    // }
  } else if (var.waterPumpState == 1) {
    if (var.height <= 5) {  // 8
      var.waterPumpState = 2;
      waterPump.off();
      waterSolenoid.off();
      buzzer.off();
      buzzer.toggleInit(100, 2);
      delay(3000);
      return;
    }
    waterPump.on();
    waterSolenoid.off();
  } else if (var.waterPumpState == 2) {
    if (var.height >= 16) {
      var.waterPumpState = 0;
      waterPump.off();
      waterSolenoid.off();
      buzzer.off();
      buzzer.toggleInit(100, 2);
      delay(3000);
      waterPumpTimer.setDuration(30000);
      waterPumpTimer.reset();
      waterPumpTimer.start();
      return;
    }
    waterPump.off();
    waterSolenoid.on();
  }
}

void debug() {
  if (!sensor.isReady()) return;
  if (var.sensorDebug == 1) sensor.debug(500, false);
  else if (var.sensorDebug == 2) sensor.debug("wl1");
  else if (var.sensorDebug == 3) {
    static uint32_t debugTimer;
    if (millis() - debugTimer >= 500) {
      Serial.print("| wl1: ");
      Serial.print(sensor["wl1"]["volt"].as<float>());
      Serial.print("| var.waterLevel1: ");
      Serial.print(var.waterLevel1);
      Serial.println();
      debugTimer = millis();
    }
  } else if (var.sensorDebug == 4) {
    Serial.print("| turb: ");
    Serial.print(var.turbidity);
    Serial.print("| height: ");
    Serial.print(var.height);
    Serial.print("| ph: ");
    Serial.print(var.ph);
    Serial.print("| temp: ");
    Serial.print(var.temperature);
    Serial.print("| wl1: ");
    Serial.print(var.waterLevel1);
    Serial.print("| wl2: ");
    Serial.print(var.waterLevel2);
    Serial.print("| wl3: ");
    Serial.print(var.waterLevel3);

    Serial.print("| wState: ");
    Serial.print(var.waterPumpState);
    Serial.print("| wTrigger: ");
    Serial.print(var.waterPumpTrigger);

    Serial.print("| T: ");
    Serial.print(waterPumpTimer.getSeconds());
    Serial.println();
  }
}

void initServo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoPhUp.setPeriodHertz(50);
  servoPhUp.attach(12, 500, 2500);
  servoPhUp.write(var.phUpServoOff);
  servoPhDown.setPeriodHertz(50);
  servoPhDown.attach(14, 500, 2500);
  servoPhDown.write(var.phDownServoOff);
  servoSalt.setPeriodHertz(50);
  servoSalt.attach(27, 500, 2500);
  servoSalt.write(var.saltServoOff);
}

void wifiConTask() {
  task.setInitCoreID(1);
  disableCore1WDT();
  disableCore0WDT();
  task.createTask(10000, [](void* pvParameter) {
    auth.apiKey = "AIzaSyCC3q3_ENzR33ajT-zVhPEaUuSjbxmeDBc";
    auth.databaseURL = "https://devaapps-d5f1a-default-rtdb.firebaseio.com/";
    auth.projectID = "devaapps-d5f1a";

    auth.user.email = "admin@gmail.com";
    auth.user.password = "admin123";

    if (!wm.autoConnect("DevaAP", ""))  // 192.168.4.1
      Serial.println("| Failed to connect");
    firebase.waitConnection(3000);
    firebase.init(&auth);

    buzzer.toggleInit(100, 2);
    disableCore1WDT();

    for (;;) {
      if (firebase.isConnect()) {
        static uint32_t dataSendTimer;
        if (millis() - dataSendTimer >= 2000) {
          JsonDocument setJson;
          setJson["kekeruhan"] = var.statusTurbidity;
          setJson["ketinggian"] = String(var.height, 2);
          setJson["ph"] = String(var.ph, 2);
          setJson["suhu"] = String(var.temperature, 2);
          setJson["waterlevel1"] = var.statusWaterLevel1;
          setJson["waterlevel2"] = var.statusWaterLevel2;
          setJson["waterlevel3"] = var.statusWaterLevel3;
          firebase.setJson("/data", setJson, FirebaseModule::resultStatusCallback);
          dataSendTimer = millis();
        }
      }
      task.delay(20);
    }
  });

  task.createTask(6000, [](void* pvParameter) {
    disableCore0WDT();
    for (;;) {
      task.delay(20);
    }
  });
}

void menuTaskCallback() {
  static auto mainMenu = menu.createMenu(menu.begin(7), "Turbid: ", "Height: ", "pH    : ", "Temp  : ", "Water1: ", "Water2: ", "Water3: ");
  menu.formatMenu(mainMenu, menu.get(0), "Turbid: %5s", var.statusTurbidity.c_str());
  menu.formatMenu(mainMenu, menu.get(1), "Height: %5d", var.height);
  menu.formatMenu(mainMenu, menu.get(2), "pH    : %5.2f", var.ph);
  menu.formatMenu(mainMenu, menu.get(3), "Temp  : %5.2f", var.temperature);
  menu.formatMenu(mainMenu, menu.get(4), "Water1: %5s", var.statusWaterLevel1.c_str());
  menu.formatMenu(mainMenu, menu.get(5), "Water2: %5s", var.statusWaterLevel2.c_str());
  menu.formatMenu(mainMenu, menu.get(6), "Water3: %5s", var.statusWaterLevel3.c_str());
  menu.showMenu(mainMenu);
}

void usbCommunicationTask(const String& dataRecv) {
  String data = dataRecv;
  String dataHeader = usbSerial.getStrData(data, 0, "#");
  String dataValue = usbSerial.getStrData(data, 1, "#");
  if (isDigit(data[0]) || isDigit(data[1])) {
    // nums
  } else {
    dataHeader.toUpperCase();
    if (dataHeader == "R") ESP.restart();
    if (dataHeader == "BTN") var.buttonStatus = dataValue;  // BTN#W

    if (dataHeader == "D") var.sensorDebug = constrain(dataValue.toInt(), 0, 5);  // E#0
    if (dataHeader == "TESTTEMP") var.temperature = dataValue.toFloat();          // TESTTEMP#28.5
    if (dataHeader == "TESTHEIGHT") var.height = dataValue.toFloat();             // TESTHEIGHT#30
    if (dataHeader == "TESTTURBID") var.turbidity = dataValue.toFloat();          // TESTTURBID#0.5

    if (dataHeader == "CALPH") calibratePhMeter();
    if (dataHeader == "LOADPH") loadCalibratePhMeter();
    if (dataHeader == "SETPH") var.ph = dataValue.toFloat();
    if (dataHeader == "CALWL1") wlCalibration.calibrateSensor();
    if (dataHeader == "LOADWL1") wlCalibration.loadCalibration();

    if (dataHeader == "TESTPUMP") waterPump.getState() ? waterPump.off() : waterPump.on();
    if (dataHeader == "TESTSOLENOID") waterSolenoid.getState() ? waterSolenoid.off() : waterSolenoid.on();
    if (dataHeader == "SERVOPHUP") servoPhUp.write(constrain(dataValue.toInt(), 0, 180));      // SERVOPHUP#140 | SERVOPHUP#60
    if (dataHeader == "SERVOPHDOWN") servoPhDown.write(constrain(dataValue.toInt(), 0, 180));  // SERVOPHDOWN#90 | SERVOSALT#0
    if (dataHeader == "SERVOSALT") servoSalt.write(constrain(dataValue.toInt(), 0, 180));      // SERVOSALT#90 | SERVOSALT#0
    if (dataHeader == "SERVOALL") {                                                            // SERVOALL#90
      servoPhUp.write(constrain(dataValue.toInt(), 0, 180));
      servoPhDown.write(constrain(dataValue.toInt(), 0, 180));
      servoSalt.write(constrain(dataValue.toInt(), 0, 180));
    }

    if (dataHeader == "TESTPHSERVO") var.ph = dataValue.toFloat();  // TESTPHSERVO#5.21 5.21, 7.43, 9.18
    if (dataHeader == "SERVOPHUPON") servoPhUp.write(60);
    if (dataHeader == "SERVOPHUPOFF") servoPhUp.write(140);
    if (dataHeader == "SERVOSALTON") servoSalt.write(0);
    if (dataHeader == "SERVOSALTOFF") servoSalt.write(90);

    if (dataHeader == "SYSENA") {  // SYSENA
      var.systemEnable = !var.systemEnable;
      Serial.print("| var.systemEnable: ");
      Serial.print(var.systemEnable);
      Serial.println();
    }

    if (dataHeader == "SYSWPUMPSTATE") {  // SYSWPUMPSTATE#1
      var.waterPumpState = dataValue.toInt();
      Serial.print("| var.waterPumpState: ");
      Serial.print(var.waterPumpState);
      Serial.println();
    }

    if (dataHeader == "SYSWPUMPTRIGGER") {  // SYSWPUMPTRIGGER
      var.waterPumpTrigger = !var.waterPumpTrigger;
      Serial.print("| var.waterPumpTrigger: ");
      Serial.print(var.waterPumpTrigger);
      Serial.println();
    }
  }
}

CalibrationData getCalibrationValueFromUser(int i) {
  Serial.println();
  CalibrationData cal;
  while (true) {
    float voltage = analogRead(32) * (3.3 / 4095.0);
    static uint32_t voltageTimer;
    if (millis() - voltageTimer >= 1000) {
      Serial.print("| voltage: ");
      Serial.print(voltage);
      Serial.print("| Masukan Data ke " + String(i + 1) + ": ");
      Serial.println();
      voltageTimer = millis();
    }
    if (Serial.available()) {
      float calibrationValueUser = Serial.readStringUntil('\n').toFloat();
      cal.voltage = voltage;
      cal.calibrationValue = calibrationValueUser;
      return cal;
    }
  }
  return cal;
}

void calibratePhMeter() {
  Serial.println("| Kalibrasi pH");
  Serial.print("| Masukan Angka Kalibrasi: ");
  int numPoints = 0;
  while (numPoints == 0) {
    if (Serial.available()) {
      numPoints = Serial.readStringUntil('\n').toInt();
    }
  }

  Serial.println(numPoints);
  calibrationDataArray = new CalibrationData[numPoints];
  calibrationDataCount = numPoints;

  for (int i = 0; i < numPoints; i++) {
    Serial.print("| Masukan Data ke " + String(i + 1) + ": ");
    CalibrationData cal = getCalibrationValueFromUser(i);
    calibrationDataArray[i].calibrationValue = cal.calibrationValue;
    calibrationDataArray[i].voltage = cal.voltage;
    Serial.print("| pH: ");
    Serial.print(calibrationDataArray[i].calibrationValue);
    Serial.print("| volt: ");
    Serial.print(calibrationDataArray[i].voltage);
    Serial.println();
  }

  preferences.begin("phCalibration", false);
  preferences.putUInt("numPoints", numPoints);
  for (int i = 0; i < numPoints; i++) {
    String phKey = "ph" + String(i);
    String voltKey = "volt" + String(i);
    preferences.putFloat(phKey.c_str(), calibrationDataArray[i].calibrationValue);
    preferences.putFloat(voltKey.c_str(), calibrationDataArray[i].voltage);
  }
  preferences.end();
  Serial.println("| Kalibrasi Selesai");

  delete[] calibrationDataArray;
  calibrationDataArray = nullptr;
}

void loadCalibratePhMeter() {
  preferences.begin("phCalibration", true);
  int numPoints = preferences.getUInt("numPoints", 0);
  calibrationDataCount = numPoints;
  if (numPoints > 0) {
    if (calibrationDataArray != nullptr) delete[] calibrationDataArray;
    calibrationDataArray = new CalibrationData[numPoints];
    Serial.print("| ph Load: ");
    Serial.println(calibrationDataCount);
    for (int i = 0; i < numPoints; i++) {
      String phKey = "ph" + String(i);
      String voltKey = "volt" + String(i);
      calibrationDataArray[i].calibrationValue = preferences.getFloat(phKey.c_str(), 0.0);
      calibrationDataArray[i].voltage = preferences.getFloat(voltKey.c_str(), 0.0);

      Serial.print("| index: ");
      Serial.print(i + 1);
      Serial.print("| calibrationValue: ");
      Serial.print(calibrationDataArray[i].calibrationValue);
      Serial.print("| voltage: ");
      Serial.print(calibrationDataArray[i].voltage);
      Serial.println();
    }
    Serial.print("| ESP.getFreeHeap(): ");
    Serial.print(ESP.getFreeHeap());
    Serial.println();
  }
  preferences.end();
}

float lagrangeInterpolation(float voltage) {
  float result = 0.0;
  for (int i = 0; i < calibrationDataCount; i++) {
    float term = calibrationDataArray[i].calibrationValue;
    for (int j = 0; j < calibrationDataCount; j++) {
      if (i != j) {
        term *= (voltage - calibrationDataArray[j].voltage) / (calibrationDataArray[i].voltage - calibrationDataArray[j].voltage);
      }
    }
    result += term;
  }
  return result;
}

float voltageToPH(float voltage) {
  if (calibrationDataCount < 2) return 0.0;
  if (voltage < calibrationDataArray[0].voltage) {
    return calibrationDataArray[0].calibrationValue;
  } else if (voltage > calibrationDataArray[calibrationDataCount - 1].voltage) {
    return calibrationDataArray[calibrationDataCount - 1].calibrationValue;
  }
  return lagrangeInterpolation(voltage);
}

double regressPh(double x) {
  double terms[] = {
    -6.9166547464124113e+005,
    1.3125982761604455e+006,
    -7.3845177407151216e+005,
    -7.5630999639436137e+004,
    2.2883163010343912e+005,
    -8.3380483977886848e+004,
    9.8781468173595022e+003
  };

  double t = 1;
  double r = 0;
  for (double c : terms) {
    r += c * t;
    t *= x;
  }
  r = constrain(r, 3, 9);
  return r;
}
