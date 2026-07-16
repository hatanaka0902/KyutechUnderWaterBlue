#include <ETH.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

//==============================
// 設定
//==============================
const char* host = "192.168.2.10";
const int   port = 5007;

IPAddress myIP(192, 168, 2, 201);
IPAddress myGW(192, 168, 2, 1);
IPAddress mySN(255, 255, 255, 0);

#define BNO08X_RESET -1
#define I2C_SDA 13
#define I2C_SCL 16

//==============================
// グローバル
//==============================
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

static bool eth_connected = false;
WiFiClient client;

// モード：STOPが来るまで継続（切断しても保持）
enum Mode { MODE_IDLE, MODE_CALIB, MODE_MEASURE };
volatile Mode currentMode = MODE_IDLE;

struct { float x, y, z; uint8_t accuracy; } data_accel, data_gyro, data_mag;
struct { float x, y, z, w; uint8_t accuracy; } data_rot;

// 再接続バックオフ
unsigned long lastReconnectAttemptMs = 0;
unsigned long reconnectIntervalMs = 500;
const unsigned long reconnectIntervalMaxMs = 8000;

// コマンド受信バッファ（非ブロッキング）
String rxLineBuf;

//==============================
// Ethernet Event
//==============================
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] IPv4: ");
      Serial.println(ETH.localIP());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

//==============================
// ソケット管理
//==============================
void closeClient() {
  if (client) client.flush();
  client.stop();
}

bool safePrintln(const String& s) {
  if (!eth_connected) return false;
  if (!client.connected()) return false;

  size_t n = client.println(s);
  if (n == 0) {
    Serial.println("[NET] TX failed -> close");
    closeClient();
    return false;
  }
  return true;
}

bool tryConnectOnce() {
  closeClient();
  client.setTimeout(20);

  if (!client.connect(host, port)) return false;

  client.println("HELLO_FROM_ESP");
  client.print("MODE=");
  client.println((currentMode == MODE_MEASURE) ? "MEASURE" :
                 (currentMode == MODE_CALIB)   ? "CALIB"   : "IDLE");
  return true;
}

void ensureConnected() {
  if (!eth_connected) return;
  if (client.connected()) return;

  unsigned long now = millis();
  if (now - lastReconnectAttemptMs < reconnectIntervalMs) return;
  lastReconnectAttemptMs = now;

  Serial.printf("[NET] Reconnecting... interval=%lu ms\n", reconnectIntervalMs);
  if (tryConnectOnce()) {
    Serial.println("[NET] Connected to PC");
    reconnectIntervalMs = 500;
  } else {
    Serial.println("[NET] Connect failed");
    reconnectIntervalMs = min(reconnectIntervalMs * 2, reconnectIntervalMaxMs);
  }
}

//==============================
// BNO08x
//==============================
void setReports() {
  long report_interval_us = 20000; // 50Hz
  bno08x.enableReport(SH2_ACCELEROMETER, report_interval_us);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval_us);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, report_interval_us);
  bno08x.enableReport(SH2_ROTATION_VECTOR, report_interval_us);
}

void startCalibration() {
  sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  safePrintln(">> [CALIB MODE] Moving sensor...");
}

void saveCalibration() {
  sh2_saveDcdNow();
  safePrintln(">> Calibration Saved to Flash.");
  currentMode = MODE_IDLE;
}

//==============================
// コマンド処理（非ブロッキング）
//==============================
void handleCommandLine(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "CALIB") {
    currentMode = MODE_CALIB;
    startCalibration();
    return;
  }
  if (cmd == "SAVE") {
    saveCalibration();
    return;
  }
  if (cmd == "MEASURE" || cmd == "REC") {
    currentMode = MODE_MEASURE;  // ★STOP来るまで継続
    safePrintln(">> MEASURE");
    return;
  }
  if (cmd == "STOP") {
    currentMode = MODE_IDLE;
    safePrintln(">> STOP");
    return;
  }
  if (cmd == "GET_ONCE") {
    String report = "ACCURACY_ONCE:Accel=" + String(data_accel.accuracy) +
                    ",Gyro=" + String(data_gyro.accuracy) +
                    ",Mag=" + String(data_mag.accuracy) +
                    ",Rot=" + String(data_rot.accuracy);
    safePrintln(report);
    return;
  }

  safePrintln(">> UNKNOWN_CMD:" + cmd);
}

void pollCommandsNonBlocking() {
  if (!eth_connected) return;
  if (!client.connected()) return;

  while (client.available() > 0) {
    char c = (char)client.read();
    if (c == '\r') continue;

    if (c == '\n') {
      String line = rxLineBuf;
      rxLineBuf = "";
      handleCommandLine(line);
    } else {
      if (rxLineBuf.length() < 200) rxLineBuf += c;
    }
  }
}

//==============================
// Setup / Loop
//==============================
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  WiFi.mode(WIFI_OFF);
  WiFi.onEvent(WiFiEvent);
  ETH.begin();
  ETH.config(myIP, myGW, mySN);

  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    if (!bno08x.begin_I2C(0x4A, &Wire)) {
      while (1) {
        Serial.println("Sensor Missing");
        delay(1000);
      }
    }
  }
  setReports();
}

void loop() {
  // 1) 再接続管理
  ensureConnected();

  // 2) コマンド受信
  pollCommandsNonBlocking();

  // 3) センサーリセット検知
  if (bno08x.wasReset()) {
    setReports();
    if (currentMode == MODE_CALIB) startCalibration();
  }

  // 4) センサー更新
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        data_accel.accuracy = sensorValue.status;
        data_accel.x = sensorValue.un.accelerometer.x;
        data_accel.y = sensorValue.un.accelerometer.y;
        data_accel.z = sensorValue.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        data_gyro.accuracy = sensorValue.status;
        data_gyro.x = sensorValue.un.gyroscope.x;
        data_gyro.y = sensorValue.un.gyroscope.y;
        data_gyro.z = sensorValue.un.gyroscope.z;
        break;

      case SH2_MAGNETIC_FIELD_CALIBRATED:
        data_mag.accuracy = sensorValue.status;
        data_mag.x = sensorValue.un.magneticField.x;
        data_mag.y = sensorValue.un.magneticField.y;
        data_mag.z = sensorValue.un.magneticField.z;
        break;

      case SH2_ROTATION_VECTOR:
        data_rot.accuracy = sensorValue.status;
        data_rot.x = sensorValue.un.rotationVector.i;
        data_rot.y = sensorValue.un.rotationVector.j;
        data_rot.z = sensorValue.un.rotationVector.k;
        data_rot.w = sensorValue.un.rotationVector.real;
        break;
    }

    // 5) 送信（50Hz：Rotation更新タイミング）
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      if (currentMode == MODE_MEASURE) {
        String csv =
          String(data_accel.x, 2) + "," + String(data_accel.y, 2) + "," + String(data_accel.z, 2) + "," +
          String(data_gyro.x,  2) + "," + String(data_gyro.y,  2) + "," + String(data_gyro.z,  2) + "," +
          String(data_mag.x,   2) + "," + String(data_mag.y,   2) + "," + String(data_mag.z,   2) + "," +
          String(data_rot.x,   4) + "," + String(data_rot.y,   4) + "," + String(data_rot.z,   4) + "," +
          String(data_rot.w,   4);

        safePrintln(csv); // 失敗ならclose → 次loopで再接続
      }
      else if (currentMode == MODE_CALIB) {
        String report =
          "CALIB > Accel:" + String(data_accel.accuracy) +
          " Gyro:" + String(data_gyro.accuracy) +
          " Mag:" + String(data_mag.accuracy) +
          " Rot:" + String(data_rot.accuracy);
        safePrintln(report);
      }
    }
  }
}

