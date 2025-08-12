#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// WiFi and Firebase credentials
#define WIFI_SSID "ebrahim"
#define WIFI_PASSWORD "snake.robot.g1"
#define API_KEY "AIzaSyAS4Mae1M-kDnXT5Iv08-Men8VNr608s4M"
#define DATABASE_URL "https://iot-project-31b61-default-rtdb.firebaseio.com/"

// تعريف الـ Pins لكل نظام
// 1. نظام المنزل (Home)
#define GAS_SENSOR_PIN     34
#define FLAME_SENSOR_PIN   35
#define GAS_SERVO_PIN      32
#define WINDOW_SERVO_PIN   33
#define HOME_BUZZER_PIN    25

// 2. نظام الإضاءة (Light)
#define LDR_PIN            36
#define LIGHT_TRIG_PIN     12
#define LIGHT_ECHO_PIN     14
#define LED_PIN            27
#define PWM_CHANNEL        0

// 3. نظام المواقف (Parking)
#define NUM_SLOTS          3
const int PARKING_TRIG_PINS[NUM_SLOTS] = {15, 16, 17};
const int PARKING_ECHO_PINS[NUM_SLOTS] = {4, 5, 18};
#define GATE_TRIG_PIN      19
#define GATE_ECHO_PIN      21
#define GATE_SERVO_PIN     26

// 4. نظام الأمان (Security)
#define SECURITY_BUZZER_PIN    25  // مشترك مع HOME_BUZZER_PIN
#define RED_LED_PIN        2
#define GREEN_LED_PIN      4
#define DOOR_SERVO_PIN     13
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 22, 23, 24, 25 };
byte colPins[COLS] = { 26, 27, 28, 29 };

// تعريف الكائنات
Servo gasServo;
Servo windowServo;
Servo gateServo;
Servo doorServo;
Keypad securityKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(0x27, 16, 2);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOk = false;

// =============== نظام المنزل (Home) ===============
bool gasControl = true;
bool flameControl = true;

void setupHomeSystem() {
  pinMode(GAS_SENSOR_PIN, INPUT);
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(HOME_BUZZER_PIN, OUTPUT);
  gasServo.attach(GAS_SERVO_PIN);
  windowServo.attach(WINDOW_SERVO_PIN);
}

void loopHomeSystem() {
  int gasValue = analogRead(GAS_SENSOR_PIN);
  int flameValue = digitalRead(FLAME_SENSOR_PIN);

  Firebase.RTDB.getBool(&fbdo, "Home_alert/gas_control");
  gasControl = fbdo.boolData();
  Firebase.RTDB.getBool(&fbdo, "Home_alert/flame_control");
  flameControl = fbdo.boolData();

  if (gasValue > 400 && gasControl) {
    digitalWrite(HOME_BUZZER_PIN, HIGH);
    gasServo.write(90);
    windowServo.write(90);
    Firebase.RTDB.setBool(&fbdo, "Home_alert/gas_alert", true);
  } 
  else if (flameValue == LOW && flameControl) {
    digitalWrite(HOME_BUZZER_PIN, HIGH);
    gasServo.write(90);
    Firebase.RTDB.setBool(&fbdo, "Home_alert/flame_alert", true);
  } 
  else {
    digitalWrite(HOME_BUZZER_PIN, LOW);
    gasServo.write(0);
    windowServo.write(0);
    Firebase.RTDB.setBool(&fbdo, "Home_alert/gas_alert", false);
    Firebase.RTDB.setBool(&fbdo, "Home_alert/flame_alert", false);
  }
}

// =============== نظام الإضاءة (Light) ===============
int ldr_value = 2000;
int distance_value = 150;
unsigned long lightOnStart = 0;
bool isLightOn = false;

void setupLightSystem() {
  pinMode(LDR_PIN, INPUT);
  pinMode(LIGHT_TRIG_PIN, OUTPUT);
  pinMode(LIGHT_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  ledcSetup(PWM_CHANNEL, 5000, 12);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
}

long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void loopLightSystem() {
  int ldrValue = analogRead(LDR_PIN);
  bool isNight = (ldrValue < ldr_value);
  long distance = readUltrasonicDistance(LIGHT_TRIG_PIN, LIGHT_ECHO_PIN);

  if (isNight) {
    if (distance > 0 && distance < distance_value) {
      ledcWrite(PWM_CHANNEL, 4095); // إضاءة قوية
      isLightOn = true;
      lightOnStart = millis();
    } else if (isLightOn && (millis() - lightOnStart >= 20000)) {
      ledcWrite(PWM_CHANNEL, 1024); // إضاءة خافتة بعد 20 ثانية
      isLightOn = false;
    } else if (!isLightOn) {
      ledcWrite(PWM_CHANNEL, 1024); // إضاءة خافتة
    }
  } else {
    ledcWrite(PWM_CHANNEL, 0); // لا إضاءة
    isLightOn = false;
  }
}

// =============== نظام المواقف (Parking) ===============
bool parkingOccupied[NUM_SLOTS] = {false};
unsigned long parkingStartTime[NUM_SLOTS] = {0};
unsigned long gateOpenTime = 0;
bool gateIsOpen = false;
#define DISTANCE_THRESHOLD 20

void setupParkingSystem() {
  for (int i = 0; i < NUM_SLOTS; i++) {
    pinMode(PARKING_TRIG_PINS[i], OUTPUT);
    pinMode(PARKING_ECHO_PINS[i], INPUT);
  }
  pinMode(GATE_TRIG_PIN, OUTPUT);
  pinMode(GATE_ECHO_PIN, INPUT);
  gateServo.attach(GATE_SERVO_PIN);
  gateServo.write(0);
}

void updateFirebaseParking(int slot, bool isOccupied, unsigned long duration = 0) {
  String path = "/slots/slot" + String(slot + 1);
  if (Firebase.ready()) {
    if (isOccupied) {
      Firebase.RTDB.setInt(&fbdo, path + "/startTime", millis());
      Firebase.RTDB.setBool(&fbdo, path + "/occupied", true);
    } else {
      Firebase.RTDB.setInt(&fbdo, path + "/duration", duration);
      Firebase.RTDB.setBool(&fbdo, path + "/occupied", false);
      int currentTotal = 0;
      if (Firebase.RTDB.getInt(&fbdo, path + "/totalOccupiedTime")) {
        currentTotal = fbdo.intData();
      }
      Firebase.RTDB.setInt(&fbdo, path + "/totalOccupiedTime", currentTotal + duration);
    }
  }
}

void openParkingGate() {
  gateServo.write(90);
  gateOpenTime = millis();
  gateIsOpen = true;
}

void loopParkingSystem() {
  static unsigned long lastCheck = 0;
  
  // إغلاق البوابة بعد 2 ثانية
  if (gateIsOpen && (millis() - gateOpenTime >= 2000)) {
    gateServo.write(0);
    gateIsOpen = false;
  }

  if (millis() - lastCheck >= 500) {
    lastCheck = millis();
    
    bool allReserved = true;
    bool carAtGate = false;
    int firstFreeSlot = -1;

    for (int i = 0; i < NUM_SLOTS; i++) {
      String reservePath = "/slots/slot" + String(i + 1) + "/reserved";
      bool reserved = false;
      if (Firebase.RTDB.getBool(&fbdo, reservePath)) {
        reserved = fbdo.boolData();
      }

      float distance = readUltrasonicDistance(PARKING_TRIG_PINS[i], PARKING_ECHO_PINS[i]);

      if (!reserved) {
        allReserved = false;
        if (firstFreeSlot == -1) {
          firstFreeSlot = i;
        }
      }

      if (distance > DISTANCE_THRESHOLD && parkingOccupied[i]) {
        unsigned long duration = (millis() - parkingStartTime[i]) / 1000;
        parkingOccupied[i] = false;
        updateFirebaseParking(i, false, duration);
      }
    }

    float gateDistance = readUltrasonicDistance(GATE_TRIG_PIN, GATE_ECHO_PIN);
    if (gateDistance < DISTANCE_THRESHOLD) {
      carAtGate = true;
    }

    if (!allReserved && carAtGate && firstFreeSlot != -1 && !parkingOccupied[firstFreeSlot]) {
      parkingOccupied[firstFreeSlot] = true;
      parkingStartTime[firstFreeSlot] = millis();
      updateFirebaseParking(firstFreeSlot, true);
      openParkingGate();
    }
  }
}

// =============== نظام الأمان (Security) ===============
String correctPassword = "1245";
String inputPassword = "";
struct PasswordChar {
  char ch;
  unsigned long timestamp;
};
PasswordChar passwordBuffer[16];

unsigned long securityStartTime;
unsigned long accessStartTime = 0;
unsigned long previousSecurityMillis = 0;
unsigned long showPasswordTime = 0;
unsigned long wrongPasswordTime = 0;

const int securityInterval = 1000;
int attempts = 0;
int remainingTime = 60;
int passwordLength = 0;

bool countdownActive = false;
bool accessGranted = false;
bool showPassword = false;
bool alarmActive = false;
bool wrongPasswordDisplay = false;
unsigned long alarmBlinkMillis = 0;
bool redLedState = false;

void setupSecuritySystem() {
  doorServo.attach(DOOR_SERVO_PIN);
  doorServo.write(0);
  pinMode(SECURITY_BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  lcd.init();
  lcd.backlight();
}

void updateLCDStatus() {
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  lcd.print(remainingTime);
  lcd.print("  Tries:");
  lcd.print(3 - attempts);
  lcd.print(" ");
}

void checkPassword() {
  if (inputPassword == correctPassword) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    lcd.clear();
    lcd.print("Access Granted");
    doorServo.write(90);
    accessStartTime = millis();
    accessGranted = true;
    inputPassword = "";
    passwordLength = 0;
  } else {
    attempts++;
    tone(SECURITY_BUZZER_PIN, 1000, 200);
    digitalWrite(RED_LED_PIN, HIGH);
    lcd.clear();
    lcd.print("Wrong Password");
    wrongPasswordTime = millis();
    wrongPasswordDisplay = true;
    inputPassword = "";
    passwordLength = 0;
  }
}

void triggerAlarm() {
  lcd.clear();
  lcd.print("ALARM!!");
  alarmActive = true;
  alarmBlinkMillis = millis();
}

void resetSecuritySystem() {
  inputPassword = "";
  attempts = 0;
  remainingTime = 60;
  previousSecurityMillis = millis();
  countdownActive = true;
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  lcd.clear();
  updateLCDStatus();
  lcd.setCursor(0, 1);
  lcd.print("Enter Password:");
  passwordLength = 0;
}

void checkFirebaseCommands() {
  static String lastCheckedPassword = "";

  if (Firebase.RTDB.getString(&fbdo, "Security_system/password")) {
    String enteredPassword = fbdo.stringData();
    if (enteredPassword != "" && enteredPassword != lastCheckedPassword) {
      lastCheckedPassword = enteredPassword;
      if (enteredPassword == correctPassword) {
        lcd.clear();
        lcd.print("Access Granted");
        digitalWrite(GREEN_LED_PIN, HIGH);
        doorServo.write(90);
        accessStartTime = millis();
        accessGranted = true;
        Firebase.RTDB.setString(&fbdo, "Security_system/password", "");
        lastCheckedPassword = "";
      } else {
        lcd.clear();
        lcd.print("Wrong Password");
        tone(SECURITY_BUZZER_PIN, 1000, 200);
        digitalWrite(RED_LED_PIN, HIGH);
        wrongPasswordTime = millis();
        wrongPasswordDisplay = true;
        Firebase.RTDB.setString(&fbdo, "Security_system/password", "");
      }
    }
  }

  if (Firebase.RTDB.getString(&fbdo, "Security_system/reset_password")) {
    String newPassword = fbdo.stringData();
    if (newPassword != "" && newPassword != correctPassword) {
      correctPassword = newPassword;
      Firebase.RTDB.setString(&fbdo, "Security_system/reset_password", "");
      lcd.clear();
      lcd.print("Password Updated");
      unsigned long messageTime = millis();
      while (millis() - messageTime < 2000) {
        // انتظار بدون delay
      }
      resetSecuritySystem();
    }
  }
}

void loopSecuritySystem() {
  char key = securityKeypad.getKey();
  
  // إدارة حالة الوصول الممنوح
  if (accessGranted) {
    if (millis() - accessStartTime >= 10000) {
      doorServo.write(0);
      digitalWrite(GREEN_LED_PIN, LOW);
      accessGranted = false;
      resetSecuritySystem();
    }
    return;
  }

  // إدارة عرض رسالة كلمة المرور الخاطئة
  if (wrongPasswordDisplay) {
    if (millis() - wrongPasswordTime >= 1000) {
      digitalWrite(RED_LED_PIN, LOW);
      wrongPasswordDisplay = false;
      updateLCDStatus();
      lcd.setCursor(0, 1);
      lcd.print("Enter Password:");
    }
    return;
  }

  // بدء النظام بالأزرار
  if (key == 'A') {
    countdownActive = true;
    lcd.backlight();
    lcd.clear();
    updateLCDStatus();
    lcd.setCursor(0, 1);
    lcd.print("Enter Password:");
    securityStartTime = millis();
    previousSecurityMillis = millis();
  }

  if (countdownActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousSecurityMillis >= securityInterval) {
      previousSecurityMillis = currentMillis;
      remainingTime--;
      updateLCDStatus();
      if (remainingTime <= 0 || attempts >= 3) {
        triggerAlarm();
        return;
      }
    }

    key = securityKeypad.getKey();
    if (key) {
      if (key == '#') {
        checkPassword();
      } else if (key == '*') {
        showPassword = true;
        showPasswordTime = millis();
      } else if (key == 'D') {
        if (passwordLength > 0) {
          passwordLength--;
          inputPassword.remove(passwordLength, 1);
        }
      } else if (passwordLength < 16) {
        passwordBuffer[passwordLength].ch = key;
        passwordBuffer[passwordLength].timestamp = millis();
        passwordLength++;
        inputPassword += key;
      }
    }

    if (showPassword && millis() - showPasswordTime < 2000) {
      lcd.setCursor(0, 1);
      lcd.print(inputPassword);
      lcd.print("                ");
    } else {
      showPassword = false;
      lcd.setCursor(0, 1);
      for (int i = 0; i < passwordLength; i++) {
        if (millis() - passwordBuffer[i].timestamp < 500) {
          lcd.print(passwordBuffer[i].ch);
        } else {
          lcd.print("*");
        }
      }
      lcd.print("                ");
    }
  }

  checkFirebaseCommands();

  if (alarmActive) {
    tone(SECURITY_BUZZER_PIN, 1000);
    if (millis() - alarmBlinkMillis >= 500) {
      alarmBlinkMillis = millis();
      redLedState = !redLedState;
      digitalWrite(RED_LED_PIN, redLedState);
    }

    if (Firebase.RTDB.getBool(&fbdo, "Security_system/alarm_status")) {
      if (fbdo.boolData() == false) {
        noTone(SECURITY_BUZZER_PIN);
        digitalWrite(RED_LED_PIN, LOW);
        lcd.clear();
        lcd.print("Alarm Stopped");
        unsigned long messageTime = millis();
        while (millis() - messageTime < 2000) {
          // انتظار بدون delay
        }
        resetSecuritySystem();
        alarmActive = false;
      }
    }
  }
}

// =============== الإعداد الرئيسي واللووب ===============
void setup() {
  Serial.begin(115200);
  
  // الاتصال بالواي فاي والفايربيز
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("SignupOk");
    signupOk = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096, 1024);
  Firebase.setDoubleDigits(3);

  // إعداد الأنظمة الفرعية
  setupHomeSystem();
  setupLightSystem();
  setupParkingSystem();
  setupSecuritySystem();

  Serial.println("All systems initialized");
}

void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastLoopTime >= 100) {
    lastLoopTime = currentMillis;
    
    loopHomeSystem();
    loopLightSystem();
    loopParkingSystem();
    loopSecuritySystem();
  }
}

