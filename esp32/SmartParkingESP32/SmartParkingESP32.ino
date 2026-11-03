#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>   // ติดตั้ง "LiquidCrystal I2C" (Frank de Brabander) ผ่าน Library Manager

#define WIFI_SSID       " ur wifi named "
#define WIFI_PASSWORD   " ur wifi pass "

// Firebase Console > Project settings > General > Web API Key
#define API_KEY         " api_key "

// ต้องเป็นผู้ใช้ Email/Password ที่สร้างใน Authentication > Users
// (ห้ามใช้อีเมล service account .iam.gserviceaccount.com)
#define USER_EMAIL      " ur mail "
#define USER_PASSWORD   " ur pass "

#define DATABASE_URL    " ur link "
// ======================================

// ---------- LCD I2C ----------
#define LCD_SDA 32
#define LCD_SCL 4
#define LCD_ADDR 0x27   // ถ้าจอไม่ขึ้น ลอง 0x3F
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
String lastLine1 = "";
String lastLine2 = "";

// ---------- Ultrasonic ทางเข้า / ทางออก / VIP ----------
#define ENTRY_TRIG  25
#define ENTRY_ECHO  34

#define EXIT_TRIG   26
#define EXIT_ECHO   35

#define VIP_TRIG    13
#define VIP_ECHO    36

// ---------- IR ช่อง A2, A3, B1, B2, B3 ----------
const int IR_PINS[5] = {14, 16, 17, 18, 19};

const bool IR_ACTIVE_LOW = true;

// ---------- 74HC595 / Buzzer ----------
#define SR_DATA  23
#define SR_CLOCK 22
#define SR_LATCH 21

const float VIP_OCCUPIED_CM = 5.0;
const float VIP_BUMPER_CM   = 2.0;

const char* SLOT_IDS[6] = {"A1", "A2", "A3", "B1", "B2", "B3"};

// ---------- ไม้กั้นทางเข้า / ทางออก ----------
Servo servoEntry;
Servo servoExit;

const int ENTRY_SERVO_PIN = 27;
const int EXIT_SERVO_PIN  = 33;

const float TRIGGER_DISTANCE_CM = 5.0;
const float CLEAR_DISTANCE_CM   =6.5;
const unsigned long GATE_HOLD_MS = 3000;

const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE   = 90;

struct GateState {
  bool isOpen = false;
  bool waitingForClear = false;
  int clearCount = 0;
  unsigned long openedAt = 0;
};

GateState entryGate;
GateState exitGate;

// ---------- Firebase client ----------
WiFiClientSecure sslClient;
using AsyncClient = AsyncClientClass;
AsyncClient client(sslClient);

UserAuth userAuth(API_KEY, USER_EMAIL, USER_PASSWORD);
FirebaseApp app;
RealtimeDatabase Database;

// ---------- สถานะระบบ ----------
int lastSlotState[6] = {-1, -1, -1, -1, -1, -1};

int lastEntryCM = -999;
int lastExitCM = -999;
int lastVIPCM = -999;

bool buzzerOn = false;
uint8_t ledMask = 0;
unsigned long lastBuzzerToggle = 0;
unsigned long lastSensorRead = 0;
unsigned long lastDistanceWrite = 0;

float entryCM = -1;
float exitCM = -1;
float vipCM = -1;

bool irOccupied[5] = {false, false, false, false, false};
bool vipOccupied = false;
bool vipTooClose = false;
float lastValidVipCM = 999;

void processData(AsyncResult &result) {
  if (!result.isResult()) return;

  if (result.isError()) {
    Serial.printf("Firebase error: %s, code: %d\n",
                  result.error().message().c_str(),
                  result.error().code());
  }

  if (result.isEvent()) {
    Serial.printf("Firebase event: %s\n",
                  result.eventLog().message().c_str());
  }
}

void write595(byte value) {
  digitalWrite(SR_LATCH, LOW);
  shiftOut(SR_DATA, SR_CLOCK, MSBFIRST, value);
  digitalWrite(SR_LATCH, HIGH);
}

void refresh595() {
  byte outputValue = ledMask | (buzzerOn ? 0b01000000 : 0b00000000);
  write595(outputValue);
}

void updateSlotLEDs() {
  bool occupied[6] = {
    vipOccupied,
    irOccupied[0],
    irOccupied[1],
    irOccupied[2],
    irOccupied[3],
    irOccupied[4]
  };

  ledMask = 0;
  for (int i = 0; i < 6; i++) {
    if (occupied[i]) {
      ledMask |= (1 << i);
    }
  }
  refresh595();
}

// ---------- LCD ----------
// บรรทัด 1: A1:VIP A2:x A3:x   (A1 ขึ้น VIP เสมอ เพราะจองได้ผ่านเว็บเท่านั้น)
// บรรทัด 2: B1:x B2:x B3:x
// x:  "-" = ว่าง, "X" = มีรถจอด
char slotMark(bool occupied) {
  return occupied ? 'X' : '-';
}

void printLine(int row, String &last, const String &text) {
  if (text == last) return;      // เขียนเฉพาะตอนข้อความเปลี่ยน ลดการกะพริบ
  last = text;
  String padded = text;
  while (padded.length() < LCD_COLS) padded += ' ';
  lcd.setCursor(0, row);
  lcd.print(padded);
}

void updateLCD() {
  String line1 = String("A1:VIP A2:") + slotMark(irOccupied[0]) +
                 " A3:" + slotMark(irOccupied[1]);
  // เว้นวรรคให้ B2 อยู่ใต้ A2 (คอลัมน์ 7) และ B3 อยู่ใต้ A3 (คอลัมน์ 12)
  String line2 = String("B1:") + slotMark(irOccupied[2]) +
                 "   B2:" + slotMark(irOccupied[3]) +
                 " B3:" + slotMark(irOccupied[4]);

  printLine(0, lastLine1, line1);
  printLine(1, lastLine2, line2);
}

float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return -1;

  return duration * 0.0343 / 2.0;
}

bool readIR(int pin) {
  int value = digitalRead(pin);

  if (IR_ACTIVE_LOW) {
    return value == LOW;
  }

  return value == HIGH;
}

void sendSlotStatus(int slotIndex, bool occupied) {
  if (slotIndex < 0 || slotIndex >= 6) return;

  int value = occupied ? 1 : 0;
  if (lastSlotState[slotIndex] == value) return;

  lastSlotState[slotIndex] = value;

  char path[40];
  snprintf(path, sizeof(path), "/sensorStatus/%s/occupied",
           SLOT_IDS[slotIndex]);

  Database.set<bool>(client, path, occupied, processData, "slotStatus");
}

void sendIntIfChanged(const char *path, int value, int &lastValue) {
  if (value == lastValue) return;

  lastValue = value;

  Database.set<int>(client, path, value, processData, "distance");
}

void updateGate(Servo &servo, GateState &gate, float distanceCM,
                const char *gateName) {
  unsigned long now = millis();

  if (gate.isOpen) {
    if (now - gate.openedAt >= GATE_HOLD_MS) {
      servo.write(GATE_CLOSED_ANGLE);
      gate.isOpen = false;
      gate.waitingForClear = true;
      gate.clearCount = 0;
      Serial.printf("%s: CLOSED\n", gateName);
    }
    return;
  }

  if (gate.waitingForClear) {
    bool clear = (distanceCM < 0 || distanceCM > CLEAR_DISTANCE_CM);
    gate.clearCount = clear ? gate.clearCount + 1 : 0;

    if (gate.clearCount >= 2) {
      gate.waitingForClear = false;
      Serial.printf("%s: READY\n", gateName);
    }
    return;
  }

  if (distanceCM > 0 && distanceCM <= TRIGGER_DISTANCE_CM) {
    servo.write(GATE_OPEN_ANGLE);
    gate.isOpen = true;
    gate.openedAt = now;
    Serial.printf("%s: OPEN (%.1f cm)\n", gateName, distanceCM);
  }
}

void setup() {
  Serial.begin(115200);

  // LCD I2C: SDA = GPIO32, SCL = GPIO4
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Parking");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  pinMode(ENTRY_TRIG, OUTPUT);
  pinMode(ENTRY_ECHO, INPUT);

  pinMode(EXIT_TRIG, OUTPUT);
  pinMode(EXIT_ECHO, INPUT);

  pinMode(VIP_TRIG, OUTPUT);
  pinMode(VIP_ECHO, INPUT);

  digitalWrite(ENTRY_TRIG, LOW);
  digitalWrite(EXIT_TRIG, LOW);
  digitalWrite(VIP_TRIG, LOW);

  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  pinMode(SR_DATA, OUTPUT);
  pinMode(SR_CLOCK, OUTPUT);
  pinMode(SR_LATCH, OUTPUT);

  ledMask = 0;
  buzzerOn = false;
  write595(0x00);

  servoEntry.setPeriodHertz(50);
  servoExit.setPeriodHertz(50);
  servoEntry.attach(ENTRY_SERVO_PIN, 600, 2300);
  servoExit.attach(EXIT_SERVO_PIN, 600, 2300);
  servoEntry.write(GATE_CLOSED_ANGLE);
  servoExit.write(GATE_CLOSED_ANGLE);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  sslClient.setInsecure();

  initializeApp(client, app, getAuth(userAuth), processData, "authTask");

  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  lcd.clear();
  updateLCD();
}

void loop() {
  app.loop();

  unsigned long now = millis();

  if (now - lastSensorRead >= 250) {
    lastSensorRead = now;

    for (int i = 0; i < 5; i++) {
      irOccupied[i] = readIR(IR_PINS[i]);
    }

    entryCM = readDistanceCM(ENTRY_TRIG, ENTRY_ECHO);
    delay(30);

    exitCM = readDistanceCM(EXIT_TRIG, EXIT_ECHO);
    delay(30);

    vipCM = readDistanceCM(VIP_TRIG, VIP_ECHO);

    if (vipCM > 0) lastValidVipCM = vipCM;

    bool inBlindZone = (vipCM < 0 && lastValidVipCM <= 3.0);

    vipOccupied = (vipCM > 0 && vipCM <= VIP_OCCUPIED_CM) || inBlindZone;
    vipTooClose = (vipCM > 0 && vipCM <= VIP_BUMPER_CM) || inBlindZone;

    updateSlotLEDs();
    updateLCD();   // อัปเดตจอ LCD ทุกครั้งที่อ่านเซนเซอร์

    Serial.print("ENTRY cm: ");
    Serial.print(entryCM);
    Serial.print(" | EXIT cm: ");
    Serial.print(exitCM);
    Serial.print(" | VIP cm: ");
    Serial.print(vipCM);
    Serial.print(" | occupied: ");
    Serial.print(vipOccupied ? "YES" : "NO");
    Serial.print(" | bumper: ");
    Serial.println(vipTooClose ? "YES" : "NO");

    updateGate(servoEntry, entryGate, entryCM, "ENTRY");
    updateGate(servoExit, exitGate, exitCM, "EXIT");

    if (app.ready()) {
      sendSlotStatus(0, vipOccupied);
      sendSlotStatus(1, irOccupied[0]);
      sendSlotStatus(2, irOccupied[1]);
      sendSlotStatus(3, irOccupied[2]);
      sendSlotStatus(4, irOccupied[3]);
      sendSlotStatus(5, irOccupied[4]);

      if (now - lastDistanceWrite >= 2000) {
        lastDistanceWrite = now;

        int entryInt = entryCM < 0 ? -1 : (int)(entryCM + 0.5);
        int exitInt  = exitCM  < 0 ? -1 : (int)(exitCM + 0.5);
        int vipInt   = vipCM   < 0 ? -1 : (int)(vipCM + 0.5);

        sendIntIfChanged("/smartParking/sensors/entryDistanceCM", entryInt, lastEntryCM);
        sendIntIfChanged("/smartParking/sensors/exitDistanceCM", exitInt, lastExitCM);
        sendIntIfChanged("/smartParking/sensors/vipDistanceCM", vipInt, lastVIPCM);
      }
    }
  }

  if (vipTooClose) {
    if (now - lastBuzzerToggle >= 200) {
      lastBuzzerToggle = now;
      buzzerOn = !buzzerOn;
      refresh595();
    }
  } else if (buzzerOn) {
    buzzerOn = false;
    refresh595();
  }
}