#include <Wire.h>
#include "Adafruit_seesaw.h"
#include <ModbusMaster.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

// ----- STEMMA (I2C) -----
Adafruit_seesaw ss;
const uint8_t SEESAW_ADDR = 0x36;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ----- Primary soil sensor (RS485 Modbus) -----
#define RX2_PIN    16
#define TX2_PIN    17
#define RE_DE_PIN  4
static const uint8_t SLAVE_ID = 1;
static const uint32_t MODBUS_BAUD = 4800;
ModbusMaster node;

// ----- Moisture gate -----
static const float MOIST_LO = 0.33f;    // 800
static const float MOIST_HI = 0.46f;    // 1064
static const uint8_t STABLE_CONSEC_SEC = 10;   // 10 s at 1 Hz in band
static const uint8_t AVG_SAMPLES = 10;         // 10 s of averaging at 1 Hz

// ----- Watchdog -----
static const uint32_t WDT_TIMEOUT_SEC = 30;

RTC_DATA_ATTR uint32_t rtcBootCount = 0;

enum RunPhase : uint8_t {
  PHASE_WAIT_MOIST,
  PHASE_AVG,
  PHASE_DONE
};

static RunPhase g_phase = PHASE_WAIT_MOIST;
static uint8_t g_stableConsec = 0;
static uint8_t g_avgCount = 0;
static double g_sumMoist = 0, g_sumTemp = 0;
static double g_sumEc = 0, g_sumPh = 0, g_sumN = 0;
static uint16_t g_nMb = 0;

void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(400);
}

void postTransmission() {
  delayMicroseconds(400);
  digitalWrite(RE_DE_PIN, LOW);
}

const unsigned long LOG_INTERVAL_MS = 1000;
unsigned long runStartMs;
unsigned long lastLogMs;

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

static void printBootDiagnostics() {
  rtcBootCount++;
  Serial.print("Reset reason: ");
  Serial.print(resetReasonStr(esp_reset_reason()));
  Serial.print(" | RTC boot count: ");
  Serial.println(rtcBootCount);
}

static bool initWatchdog() {
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_SEC * 1000u;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;

  esp_err_t e = esp_task_wdt_init(&cfg);
  if (e == ESP_ERR_INVALID_STATE) {
    e = esp_task_wdt_add(NULL);
    if (e != ESP_OK) {
      Serial.print("esp_task_wdt_add failed: ");
      Serial.println((int)e);
      return false;
    }
    return true;
  }
  if (e != ESP_OK) {
    Serial.print("esp_task_wdt_init failed: ");
    Serial.println((int)e);
    return false;
  }
  e = esp_task_wdt_add(NULL);
  if (e != ESP_OK) {
    Serial.print("esp_task_wdt_add failed: ");
    Serial.println((int)e);
    return false;
  }
  return true;
}

static const char* modbusResultTag(const ModbusMaster& m, uint8_t r) {
  if (r == m.ku8MBSuccess) return "OK";
  if (r == m.ku8MBResponseTimedOut) return "TO";
  if (r == m.ku8MBInvalidCRC) return "CRC";
  if (r == m.ku8MBInvalidSlaveID) return "BAD_SLAVE";
  if (r == m.ku8MBInvalidFunction) return "BAD_FN";
  if (r == m.ku8MBIllegalFunction) return "EX_ILLEGAL_FN";
  if (r == m.ku8MBIllegalDataAddress) return "EX_BAD_ADDR";
  if (r == m.ku8MBIllegalDataValue) return "EX_BAD_VAL";
  if (r == m.ku8MBSlaveDeviceFailure) return "EX_SLAVE_FAIL";
  static char buf[6];
  snprintf(buf, sizeof(buf), "0x%02X", r);
  return buf;
}

static bool primaryValuesInRange(float ec_uScm, float ph, float n_mgkg) {
  if (ph < 0.0f || ph > 14.0f) return false;
  if (ec_uScm < 0.0f || ec_uScm > 200000.0f) return false;
  if (n_mgkg < 0.0f || n_mgkg > 100000.0f) return false;
  return true;
}

static bool stemmaTempInRange(float tempC) {
  return tempC >= -40.0f && tempC <= 85.0f;
}

static void printAveragesAndFinish() {
  Serial.println("=== AVERAGES (10 s window) ===");
  Serial.print("Moist_avg\t");
  Serial.println(g_sumMoist / (double)AVG_SAMPLES, 4);
  Serial.print("TempC_avg\t");
  Serial.println(g_sumTemp / (double)AVG_SAMPLES, 2);
  if (g_nMb > 0) {
    double inv = 1.0 / (double)g_nMb;
    Serial.print("EC_uS_avg\t");
    Serial.println(g_sumEc * inv, 1);
    Serial.print("pH_avg\t\t");
    Serial.println(g_sumPh * inv, 2);
    Serial.print("N_mgkg_avg\t");
    Serial.println(g_sumN * inv, 1);
    Serial.print("(Modbus samples in avg: ");
    Serial.print(g_nMb);
    Serial.print(" / ");
    Serial.print(AVG_SAMPLES);
    Serial.println(")");
  } else {
    Serial.println("EC/pH/N_avg\tNA (no successful Modbus reads in window)");
  }
  Serial.println("=== Sampling stopped ===");
  g_phase = PHASE_DONE;
}

static void runOneSecondTick() {
  unsigned long now = millis();
  unsigned long elapsedSec = (now - runStartMs) / 1000;

  uint16_t raw = ss.touchRead(0);
  float moistureNorm = (raw - 200.0f) / 1800.0f;
  moistureNorm = constrain(moistureNorm, 0.0f, 1.0f);
  float tempC = ss.getTemp();
  bool stTempOk = stemmaTempInRange(tempC);

  float ec_uScm = 0, ph = 0, n_mgkg = 0;
  uint8_t mbResult = node.readHoldingRegisters(0x0002, 3);
  bool mbOk = (mbResult == node.ku8MBSuccess);
  if (mbOk) {
    ec_uScm = (float)node.getResponseBuffer(0);
    ph = node.getResponseBuffer(1) * 0.1f;
    n_mgkg = (float)node.getResponseBuffer(2);
  }
  bool primRangeOk = mbOk && primaryValuesInRange(ec_uScm, ph, n_mgkg);

  const char* dataq;
  if (!mbOk) {
    dataq = "NA_MB";
  } else if (!primRangeOk) {
    dataq = "RANGE_BAD";
  } else if (!stTempOk) {
    dataq = "STEMMA_TEMP";
  } else {
    dataq = "OK";
  }

  const char* phaseTag =
      (g_phase == PHASE_WAIT_MOIST) ? "WAIT" : (g_phase == PHASE_AVG) ? "AVG" : "DONE";

  if (g_phase == PHASE_WAIT_MOIST) {
    if (moistureNorm >= MOIST_LO && moistureNorm <= MOIST_HI) {
      g_stableConsec++;
    } else {
      g_stableConsec = 0;
    }
  }

  Serial.print(phaseTag);
  Serial.print("\t");
  Serial.print(elapsedSec);
  Serial.print("\t");
  Serial.print(moistureNorm, 3);
  Serial.print("\t");
  Serial.print(tempC, 1);
  Serial.print("\t");
  if (mbOk) {
    Serial.print(ec_uScm, 0);
    Serial.print("\t");
    Serial.print(ph, 1);
    Serial.print("\t");
    Serial.print(n_mgkg, 0);
  } else {
    Serial.print("NA\tNA\tNA");
  }
  Serial.print("\t");
  Serial.print(modbusResultTag(node, mbResult));
  Serial.print("\t");
  Serial.print(dataq);
  Serial.print("\t");
  if (g_phase == PHASE_WAIT_MOIST) {
    Serial.println(g_stableConsec);
  } else if (g_phase == PHASE_AVG) {
    Serial.println(g_avgCount + 1);
  } else {
    Serial.println("-");
  }

  if (g_phase == PHASE_WAIT_MOIST) {
    if (g_stableConsec >= STABLE_CONSEC_SEC) {
      Serial.println("REACHED");
      g_stableConsec = 0;
      g_sumMoist = g_sumTemp = g_sumEc = g_sumPh = g_sumN = 0;
      g_nMb = 0;
      g_avgCount = 0;
      g_phase = PHASE_AVG;
    }
    return;
  }

  if (g_phase == PHASE_AVG) {
    g_sumMoist += (double)moistureNorm;
    g_sumTemp += (double)tempC;
    if (mbOk) {
      g_sumEc += (double)ec_uScm;
      g_sumPh += (double)ph;
      g_sumN += (double)n_mgkg;
      g_nMb++;
    }
    g_avgCount++;
    if (g_avgCount >= AVG_SAMPLES) {
      printAveragesAndFinish();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  printBootDiagnostics();

  if (!initWatchdog()) {
    Serial.println("Watchdog not enabled; continuing without WDT.");
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ss.begin(SEESAW_ADDR)) {
    Serial.println("STEMMA sensor not found");
    while (1) {
      esp_task_wdt_reset();
      delay(500);
    }
  }
  Serial.println("STEMMA sensor OK");

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  delay(750);
  Serial.println("Primary sensor (Modbus) link initialized");

  g_phase = PHASE_WAIT_MOIST;
  g_stableConsec = 0;
  runStartMs = millis();
  lastLogMs = runStartMs;

  Serial.println("=== Moisture in [0.33, 0.46] for 10 s -> REACHED; then 10 s avg ===");
  Serial.println(
      "Phase\tsec\tMoist\tTempC\tEC_uS\tpH\tN_mgkg\tModbus\tDataQ\tst|avg#");
  Serial.println(
      "-------------------------------------------------------------------------");
}

void loop() {
  esp_task_wdt_reset();

  if (g_phase == PHASE_DONE) {
    delay(500);
    return;
  }

  unsigned long now = millis();
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    runOneSecondTick();
  }

  delay(50);
}
