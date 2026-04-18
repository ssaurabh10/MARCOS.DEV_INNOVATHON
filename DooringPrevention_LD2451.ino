/*
 * ================================================================
 *  🚗 DOORING ACCIDENT PREVENTION SYSTEM
 *  Using HiLink LD2451 24GHz mmWave Radar + Arduino UNO R3
 * ================================================================
 *
 *  PURPOSE:
 *    Prevents "dooring" accidents by detecting approaching cyclists,
 *    e-scooters, motorcycles, and vehicles when a parked car's door
 *    is about to be opened. Provides multi-level visual + audio
 *    warnings to the car occupant.
 *
 *  HOW IT WORKS:
 *    1. Radar is mounted on the B-pillar / side mirror facing
 *       backward along the traffic lane.
 *    2. System continuously scans for approaching objects.
 *    3. Calculates Time-To-Collision (TTC) = distance / speed.
 *    4. Three alert levels:
 *       🟢 SAFE     — No approaching target, or TTC > 6s
 *       🟡 CAUTION  — Target approaching, TTC 3-6s
 *       🔴 DANGER   — Target approaching, TTC < 3s (DO NOT OPEN!)
 *
 *  WIRING:
 *    LD2451 VCC  →  5V
 *    LD2451 GND  →  GND
 *    LD2451 TX   →  Pin 10  (SoftSerial RX)
 *    LD2451 RX   →  Pin 11  (SoftSerial TX)
 *    Green  LED  →  Pin 5   (SAFE indicator)
 *    Yellow LED  →  Pin 6   (CAUTION indicator)
 *    Red    LED  →  Pin 7   (DANGER indicator)
 *
 *  SERIAL MONITOR:
 *    Open at 115200 baud to see live status, settings menu,
 *    and ASCII radar display.
 *
 * ================================================================
 */

#include <SoftwareSerial.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
#define RADAR_RX_PIN    10    // Arduino RX ← LD2451 TX
#define RADAR_TX_PIN    11    // Arduino TX → LD2451 RX
#define LED_GREEN       5     // SAFE
#define LED_YELLOW      6     // CAUTION
#define LED_RED         7     // DANGER

// ═══════════════════════════════════════════════════════════════
//  CONFIGURATION  (tweak these for your setup)
// ═══════════════════════════════════════════════════════════════
#define RADAR_BAUD      115200  // LD2451 default baud rate

bool    ENABLE_HARDWARE = false; // Set to true if you wire up LEDs
bool    DEBUG_RAW       = false; // Set to true to print raw hex data from radar

// Safety thresholds
float   TTC_DANGER      = 3.0;   // seconds — DANGER if TTC < this
float   TTC_CAUTION     = 6.0;   // seconds — CAUTION if TTC < this
uint8_t MIN_SPEED_KMH   = 5;     // Ignore targets slower than this
uint8_t MAX_DETECT_DIST = 80;    // Max distance to care about (meters)
uint8_t MIN_SNR         = 10;    // Minimum signal quality to trust

// ═══════════════════════════════════════════════════════════════
//  LD2451 DATA PROTOCOL
// ═══════════════════════════════════════════════════════════════
/*
 * LD2451 Data Frame:
 *   Header : F4 F3 F2 F1
 *   Length  : 2 bytes (LE)
 *   Data   : [target_count] + [5 bytes per target]
 *   Footer : F8 F7 F6 F5
 *
 * Per-target (5 bytes):
 *   [0] Angle     — raw - 0x80 = signed degrees (±20°)
 *   [1] Distance  — meters (0-100)
 *   [2] Direction — 0x00=receding, 0x01=approaching
 *   [3] Speed     — km/h
 *   [4] SNR       — signal quality (0-255)
 */

SoftwareSerial radarSerial(RADAR_RX_PIN, RADAR_TX_PIN);

const uint8_t LD_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t LD_FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

// Command frame constants (for configuration)
const uint8_t CMD_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CMD_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};

#define MAX_TARGETS 5

// ═══════════════════════════════════════════════════════════════
//  DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════
struct Target {
  int8_t  angle;        // degrees (±20°)
  uint8_t distance;     // meters
  uint8_t direction;    // 0=receding, 1=approaching
  uint8_t speed;        // km/h
  uint8_t snr;          // signal quality
  float   ttc;          // time-to-collision (seconds), 999 = no collision
};

enum AlertLevel {
  ALERT_SAFE    = 0,
  ALERT_CAUTION = 1,
  ALERT_DANGER  = 2
};

// Global state
Target      targets[MAX_TARGETS];
uint8_t     targetCount = 0;
AlertLevel  currentAlert = ALERT_SAFE;
AlertLevel  prevAlert = ALERT_SAFE;

// Statistics
uint32_t    dangerEvents = 0;
uint32_t    cautionEvents = 0;
uint32_t    totalDetections = 0;
uint8_t     peakSpeedSeen = 0;
float       closestTTC = 999.0;

// Display mode
bool        liveMode = true;
bool        radarViewMode = false;
bool        quietMode = false;     // suppress serial output in live mode
bool        simMode = false;       // simulation mode (no radar needed)

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  // Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  // Radar serial
  radarSerial.begin(RADAR_BAUD);

  // Output pins
  if (ENABLE_HARDWARE) {
    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED,    OUTPUT);

    // Startup LED test
    startupAnimation();
  }

  // Print welcome
  Serial.println(F("\n"));
  Serial.println(F("╔══════════════════════════════════════════════╗"));
  Serial.println(F("║  🚗  DOORING ACCIDENT PREVENTION SYSTEM     ║"));
  Serial.println(F("║      LD2451 24GHz mmWave Radar               ║"));
  Serial.println(F("╠══════════════════════════════════════════════╣"));
  Serial.println(F("║  System Status: ACTIVE                       ║"));
  Serial.print  (F("║  Radar Baud:    ")); Serial.print(RADAR_BAUD);
  Serial.println(F("                     ║"));
  Serial.print  (F("║  TTC Danger:    <")); Serial.print(TTC_DANGER, 1);
  Serial.println(F("s                       ║"));
  Serial.print  (F("║  TTC Caution:   <")); Serial.print(TTC_CAUTION, 1);
  Serial.println(F("s                       ║"));
  Serial.println(F("╚══════════════════════════════════════════════╝"));
  Serial.println(F("Type 'M' for settings menu. System is monitoring.\n"));

  // Start in safe state
  setAlertLevel(ALERT_SAFE);
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') handleCommand(c);
  }

  // Read radar data
  if (readTargetData()) {
    // Process targets and determine alert level
    AlertLevel threat = evaluateThreat();

    setAlertLevel(threat);

    // Display
    if (radarViewMode) {
      drawASCIIRadar();
    } else if (liveMode && !quietMode) {
      printLiveStatus();
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  LD2451 DATA PARSER (Non-Blocking State Machine)
// ═══════════════════════════════════════════════════════════════
enum ParserState { WAIT_HEADER, WAIT_LENGTH, WAIT_PAYLOAD, WAIT_FOOTER };
ParserState pState = WAIT_HEADER;
uint16_t expectedLen = 0;
uint16_t payloadIdx = 0;
uint8_t rxBuf[128];
uint8_t footBuf[4];

bool readTargetData() {
  bool packetReady = false;

  while (radarSerial.available()) {
    uint8_t b = radarSerial.read();

    if (DEBUG_RAW) {
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }

    switch (pState) {
      case WAIT_HEADER:
        rxBuf[payloadIdx++] = b;
        if (payloadIdx >= 4) {
          if (rxBuf[payloadIdx - 4] == 0xF4 && rxBuf[payloadIdx - 3] == 0xF3 &&
              rxBuf[payloadIdx - 2] == 0xF2 && rxBuf[payloadIdx - 1] == 0xF1) {
            pState = WAIT_LENGTH;
            payloadIdx = 0;
          } else {
            rxBuf[0] = rxBuf[1];
            rxBuf[1] = rxBuf[2];
            rxBuf[2] = rxBuf[3];
            payloadIdx = 3;
          }
        }
        break;

      case WAIT_LENGTH:
        rxBuf[payloadIdx++] = b;
        if (payloadIdx == 2) {
          expectedLen = rxBuf[0] | ((uint16_t)rxBuf[1] << 8);
          payloadIdx = 0;
          if (expectedLen == 0 || expectedLen > 64) {
            pState = WAIT_HEADER; // Invalid length, reset
          } else {
            pState = WAIT_PAYLOAD;
          }
        }
        break;

      case WAIT_PAYLOAD:
        rxBuf[payloadIdx++] = b;
        if (payloadIdx == expectedLen) {
          pState = WAIT_FOOTER;
          payloadIdx = 0;
        }
        break;

      case WAIT_FOOTER:
        footBuf[payloadIdx++] = b;
        if (payloadIdx == 4) {
          pState = WAIT_HEADER;
          payloadIdx = 0;
          if (footBuf[0] == 0xF8 && footBuf[1] == 0xF7 && footBuf[2] == 0xF6 && footBuf[3] == 0xF5) {
            if (DEBUG_RAW) Serial.println(F(" [VALID]"));

            // Parse valid payload
            targetCount = rxBuf[0];
            if (targetCount > MAX_TARGETS) targetCount = MAX_TARGETS;

            for (uint8_t i = 0; i < targetCount; i++) {
              // Base offset is 2 because rxBuf[0] is targetCount and rxBuf[1] is Alarm Info
              uint8_t base = 2 + (i * 5);
              if (base + 4 >= expectedLen) { targetCount = i; break; }

              targets[i].angle     = (int8_t)(rxBuf[base + 0] - 0x80);
              targets[i].distance  = rxBuf[base + 1];
              targets[i].direction = rxBuf[base + 2];
              targets[i].speed     = rxBuf[base + 3];
              targets[i].snr       = rxBuf[base + 4];

              // Calculate Time-To-Collision
              if (targets[i].direction == 1 && targets[i].speed > 0) {
                float speed_ms = targets[i].speed * 1000.0 / 3600.0;
                targets[i].ttc = targets[i].distance / speed_ms;
              } else {
                targets[i].ttc = 999.0;
              }
            }
            packetReady = true;
          } else {
            if (DEBUG_RAW) Serial.println(F(" [BAD FOOTER]"));
          }
        }
        break;
    }
  }

  return packetReady;
}

// ═══════════════════════════════════════════════════════════════
//  THREAT EVALUATION ENGINE
// ═══════════════════════════════════════════════════════════════
AlertLevel evaluateThreat() {
  AlertLevel worstThreat = ALERT_SAFE;
  float worstTTC = 999.0;

  for (uint8_t i = 0; i < targetCount; i++) {
    // Filter: only care about approaching targets
    if (targets[i].direction != 1) continue;

    // Filter: ignore very slow objects (pedestrians, etc.)
    if (targets[i].speed < MIN_SPEED_KMH) continue;

    // Filter: ignore targets beyond max distance
    if (targets[i].distance > MAX_DETECT_DIST) continue;

    // Filter: ignore low-quality signals
    if (targets[i].snr < MIN_SNR) continue;

    // Track statistics
    totalDetections++;
    if (targets[i].speed > peakSpeedSeen) peakSpeedSeen = targets[i].speed;

    float ttc = targets[i].ttc;
    if (ttc < worstTTC) worstTTC = ttc;

    // Determine threat level for this target
    if (ttc < TTC_DANGER) {
      dangerEvents++;
      worstThreat = ALERT_DANGER;
    } else if (ttc < TTC_CAUTION) {
      cautionEvents++;
      if (worstThreat < ALERT_DANGER) worstThreat = ALERT_CAUTION;
    }
  }

  if (worstTTC < closestTTC) closestTTC = worstTTC;

  return worstThreat;
}

// ═══════════════════════════════════════════════════════════════
//  ALERT OUTPUT — LEDs + BUZZER
// ═══════════════════════════════════════════════════════════════
void setAlertLevel(AlertLevel level) {
  prevAlert = currentAlert;
  currentAlert = level;

  if (ENABLE_HARDWARE) {
    switch (level) {
      case ALERT_SAFE:
        digitalWrite(LED_GREEN,  HIGH);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_RED,    LOW);
        break;

      case ALERT_CAUTION:
        digitalWrite(LED_GREEN,  LOW);
        digitalWrite(LED_YELLOW, HIGH);
        digitalWrite(LED_RED,    LOW);
        break;

      case ALERT_DANGER:
        digitalWrite(LED_GREEN,  LOW);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_RED,    HIGH);
        break;
    }
  }

  // Print alert change
  if (level != prevAlert && !quietMode) {
    Serial.print(F("\n>>> ALERT: "));
    printAlertName(level);
    Serial.println();
  }
}

// ═══════════════════════════════════════════════════════════════
//  LIVE STATUS DISPLAY
// ═══════════════════════════════════════════════════════════════
void printLiveStatus() {
  // Alert bar
  Serial.print(F("["));
  printAlertName(currentAlert);
  Serial.print(F("]"));

  Serial.print(F("  Targets: "));
  Serial.print(targetCount);

  if (targetCount == 0) {
    Serial.println(F("  — Clear"));
    return;
  }

  Serial.println();

  // Print each relevant target
  for (uint8_t i = 0; i < targetCount; i++) {
    if (targets[i].direction != 1) continue;  // only show approaching
    if (targets[i].speed < MIN_SPEED_KMH) continue;

    Serial.print(F("  T")); Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(targets[i].distance); Serial.print(F("m"));
    Serial.print(F("  ")); Serial.print(targets[i].speed); Serial.print(F("km/h"));
    Serial.print(F("  ")); Serial.print(targets[i].angle); Serial.print(F("°"));
    Serial.print(F("  TTC:")); Serial.print(targets[i].ttc, 1); Serial.print(F("s"));
    Serial.print(F("  SNR:")); Serial.print(targets[i].snr);

    // Threat indicator
    if (targets[i].ttc < TTC_DANGER)       Serial.print(F("  <<<DANGER>>>"));
    else if (targets[i].ttc < TTC_CAUTION) Serial.print(F("  !CAUTION!"));

    Serial.println();
  }
}

void printAlertName(AlertLevel level) {
  switch (level) {
    case ALERT_SAFE:     Serial.print(F("SAFE")); break;
    case ALERT_CAUTION:  Serial.print(F("CAUTION")); break;
    case ALERT_DANGER:   Serial.print(F("DANGER")); break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  ASCII RADAR DISPLAY
// ═══════════════════════════════════════════════════════════════
/*
 *  Top-down view showing sensor at bottom, targets as symbols.
 *  '-' = approaching       'o' = receding
 *  Field of view: ±20° forward, up to MAX_DETECT_DIST meters
 *
 *           .  .  .  .  .  80m
 *           .  .  .  .  .
 *           .  .  -  .  .   ← cyclist at 40m, 5° left
 *           .  .  .  .  .
 *           .  .  S  .  .   0m  (sensor / car door)
 */
void drawASCIIRadar() {
  // ANSI clear screen
  Serial.print(F("\033[2J\033[H"));

  // Header with alert status
  Serial.print(F("╔═══ RADAR VIEW ═══╗  Alert: "));
  printAlertName(currentAlert);
  Serial.println();

  // Grid: 11 rows × 21 cols
  // Row 0 = far (MAX_DETECT_DIST), Row 10 = sensor (0m)
  const uint8_t ROWS = 11;
  const uint8_t COLS = 21;
  char grid[ROWS][COLS + 1];

  // Fill grid with dots
  for (uint8_t r = 0; r < ROWS; r++) {
    for (uint8_t c = 0; c < COLS; c++) {
      grid[r][c] = '.';
    }
    grid[r][COLS] = '\0';
  }

  // Mark sensor position
  grid[ROWS - 1][COLS / 2] = 'S';

  // Zones can optionally be handled here if we want to draw boundary lines in the future

  // Plot targets
  for (uint8_t i = 0; i < targetCount; i++) {
    if (targets[i].distance == 0) continue;

    // Map distance to row (0=far, ROWS-1=close)
    int row = ROWS - 1 - map(targets[i].distance, 0, MAX_DETECT_DIST, 0, ROWS - 1);
    row = constrain(row, 0, ROWS - 1);

    // Map angle to column (-20° left, +20° right)
    int col = COLS / 2 + map(targets[i].angle, -20, 20, -(COLS / 2), COLS / 2);
    col = constrain(col, 0, COLS - 1);

    // Symbol based on threat level
    char sym;
    if (targets[i].direction == 1) {  // approaching
      if (targets[i].ttc < TTC_DANGER)       sym = 'X';  // DANGER
      else if (targets[i].ttc < TTC_CAUTION) sym = '!';  // CAUTION
      else                                   sym = 'v';  // approaching (safe)
    } else {
      sym = '^';  // receding
    }

    grid[row][col] = sym;
  }

  // Print grid with distance labels
  for (uint8_t r = 0; r < ROWS; r++) {
    uint8_t dist = map(r, 0, ROWS - 1, MAX_DETECT_DIST, 0);

    // Zone coloring via prefix
    if (dist <= (uint8_t)(TTC_DANGER * 15))   Serial.print(F(" !! "));
    else if (dist <= (uint8_t)(TTC_CAUTION * 15)) Serial.print(F("  ! "));
    else Serial.print(F("    "));

    // Distance label
    if (dist < 10) Serial.print(F("  ")); else if (dist < 100) Serial.print(F(" "));
    Serial.print(dist);
    Serial.print(F("m |"));

    // Grid row
    for (uint8_t c = 0; c < COLS; c++) {
      Serial.print(' ');
      Serial.print(grid[r][c]);
    }

    Serial.println(F(" |"));
  }

  // Legend
  Serial.println(F("      ──┴─────────────────────────┘"));
  Serial.println(F("  X=DANGER  !=CAUTION  v=approach  ^=recede  S=sensor"));

  // Target details
  if (targetCount > 0) {
    Serial.println(F("  ─── Target Details ───"));
    for (uint8_t i = 0; i < targetCount; i++) {
      Serial.print(F("  T")); Serial.print(i + 1);
      Serial.print(F(": ")); Serial.print(targets[i].distance); Serial.print(F("m"));
      Serial.print(F(" ")); Serial.print(targets[i].speed); Serial.print(F("km/h"));
      Serial.print(F(" ")); Serial.print(targets[i].angle); Serial.print(F("°"));
      Serial.print(F(" TTC:")); Serial.print(targets[i].ttc, 1); Serial.print(F("s"));
      Serial.print(F(" ")); Serial.println(targets[i].direction == 1 ? F("APPROACH") : F("RECEDE"));
    }
  } else {
    Serial.println(F("\n  No targets detected — SAFE to open door"));
  }
}

// ═══════════════════════════════════════════════════════════════
//  CONFIGURATION COMMANDS (to LD2451)
// ═══════════════════════════════════════════════════════════════
void sendCommand(uint16_t cmd, const uint8_t* data, uint16_t dataLen) {
  radarSerial.write(CMD_HEADER, 4);
  uint16_t totalLen = 2 + dataLen;
  radarSerial.write((uint8_t)(totalLen & 0xFF));
  radarSerial.write((uint8_t)(totalLen >> 8));
  radarSerial.write((uint8_t)(cmd & 0xFF));
  radarSerial.write((uint8_t)(cmd >> 8));
  if (data && dataLen > 0) radarSerial.write(data, dataLen);
  radarSerial.write(CMD_FOOTER, 4);
}

int readCmdResponse(uint8_t* buf, uint16_t bufSize, uint16_t timeoutMs = 1000) {
  uint32_t start = millis();
  uint8_t headerBuf[4] = {0};
  uint16_t idx = 0;

  while (millis() - start < timeoutMs) {
    if (radarSerial.available()) {
      uint8_t b = radarSerial.read();
      headerBuf[idx % 4] = b;
      idx++;
      if (idx >= 4) {
        bool found = true;
        for (int i = 0; i < 4; i++)
          if (headerBuf[(idx - 4 + i) % 4] != CMD_HEADER[i]) { found = false; break; }
        if (found) break;
      }
    }
  }
  if (millis() - start >= timeoutMs) return -1;

  while (radarSerial.available() < 2 && millis() - start < timeoutMs);
  if (radarSerial.available() < 2) return -1;
  uint8_t lenL = radarSerial.read();
  uint8_t lenH = radarSerial.read();
  uint16_t payloadLen = (uint16_t)lenL | ((uint16_t)lenH << 8);
  if (payloadLen > bufSize) return -1;

  uint16_t received = 0;
  while (received < payloadLen && millis() - start < timeoutMs) {
    if (radarSerial.available()) buf[received++] = radarSerial.read();
  }

  // Drain footer
  uint8_t foot = 0;
  while (foot < 4 && millis() - start < timeoutMs)
    if (radarSerial.available()) { radarSerial.read(); foot++; }

  return (int)payloadLen;
}

void sendAndPrint(uint16_t cmd, const uint8_t* data, uint16_t dataLen, const char* label) {
  uint8_t respBuf[128];
  Serial.print(F("[>>] ")); Serial.println(label);
  sendCommand(cmd, data, dataLen);
  int len = readCmdResponse(respBuf, sizeof(respBuf));
  if (len < 0) { Serial.println(F("  No response.")); return; }
  Serial.print(F("  Response (")); Serial.print(len); Serial.print(F(" bytes): "));
  for (int i = 0; i < len; i++) {
    if (respBuf[i] < 0x10) Serial.print('0');
    Serial.print(respBuf[i], HEX); Serial.print(' ');
  }
  Serial.println();
  if (len >= 4) {
    if (respBuf[3] == 0x00) Serial.println(F("  Status: OK"));
    else Serial.println(F("  Status: ERROR"));
  }
}

void enableConfig() {
  const uint8_t payload[] = {0x01, 0x00};
  sendAndPrint(0x00FF, payload, 2, "Enable Config Mode");
}

void disableConfig() {
  sendAndPrint(0x00FE, nullptr, 0, "End Config Mode");
}

// Configure sensor for optimal dooring prevention
void configureForDooring() {
  Serial.println(F("\n=== Configuring LD2451 for Dooring Prevention ==="));
  enableConfig();

  // Set direction filter: only approaching (0x01)
  uint8_t dirPayload[] = {0x01};
  sendAndPrint(0x0005, dirPayload, 1, "Direction: Approaching only");

  // Set max detection distance
  uint8_t distPayload[] = {MAX_DETECT_DIST};
  sendAndPrint(0x0001, distPayload, 1, "Set Max Detection Distance");

  // Set minimum speed (filter out pedestrians)
  uint8_t spdPayload[] = {MIN_SPEED_KMH};
  sendAndPrint(0x0003, spdPayload, 1, "Set Min Speed Threshold");

  disableConfig();
  Serial.println(F("=== Configuration Complete ===\n"));
}

// ═══════════════════════════════════════════════════════════════
//  SERIAL MENU
// ═══════════════════════════════════════════════════════════════
int readIntFromSerial(const char* prompt, int minVal, int maxVal) {
  Serial.print(prompt);
  while (!Serial.available());
  int v = Serial.parseInt();
  v = constrain(v, minVal, maxVal);
  Serial.println(v);
  return v;
}

void printMenu() {
  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  🚗  DOORING PREVENTION — SETTINGS MENU          ║"));
  Serial.println(F("╠═══════════════════════════════════════════════════╣"));
  Serial.println(F("║  ── Monitoring ──                                 ║"));
  Serial.println(F("║  L – Toggle Live Status Display                   ║"));
  Serial.println(F("║  D – ASCII Radar View (30s)                       ║"));
  Serial.println(F("║  Q – Toggle Quiet Mode (LEDs + buzzer only)       ║"));
  Serial.println(F("║  ── Safety Thresholds ──                          ║"));
  Serial.println(F("║  1 – Set DANGER TTC threshold (seconds)           ║"));
  Serial.println(F("║  2 – Set CAUTION TTC threshold (seconds)          ║"));
  Serial.println(F("║  3 – Set Min Speed Filter (km/h)                  ║"));
  Serial.println(F("║  4 – Set Max Detection Distance (m)               ║"));
  Serial.println(F("║  5 – Set Min SNR Filter                           ║"));
  Serial.println(F("║  ── Sensor Commands ──                            ║"));
  Serial.println(F("║  I – Auto-Configure Sensor for Dooring            ║"));
  Serial.println(F("║  V – Get Firmware Version                         ║"));
  Serial.println(F("║  R – Restart Sensor                               ║"));
  Serial.println(F("║  F – Factory Reset Sensor                         ║"));
  Serial.println(F("║  B – Turn Bluetooth ON / OFF                      ║"));
  Serial.println(F("║  ── Diagnostics ──                                ║"));
  Serial.println(F("║  S – Show Statistics                              ║"));
  Serial.println(F("║  T – Self-Test (LED + Buzzer)                     ║"));
  Serial.println(F("║  G – CSV Data Logger (for analysis)               ║"));
  Serial.println(F("║  P – Serial Plotter Mode (60s)                    ║"));
  Serial.println(F("║  ── System ──                                     ║"));
  Serial.println(F("║  X – Toggle Raw Hex Debug (Data Viewer)           ║"));
  Serial.println(F("║  E – SIMULATION Demo (no radar needed)            ║"));
  Serial.println(F("║  M – Show This Menu                               ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝"));
  Serial.print(F("> "));
}

void handleCommand(char cmd) {
  switch (cmd) {
    // ── Monitoring ──
    case 'L': case 'l':
      liveMode = !liveMode;
      radarViewMode = false;
      Serial.print(F("Live display: ")); Serial.println(liveMode ? F("ON") : F("OFF"));
      break;

    case 'D': case 'd': {
      radarViewMode = true;
      liveMode = false;
      Serial.println(F("Radar View — 30s (press key to stop)"));
      uint32_t end = millis() + 30000UL;
      while (millis() < end && !Serial.available()) {
        if (readTargetData()) {
          AlertLevel threat = evaluateThreat();
          setAlertLevel(threat);
          drawASCIIRadar();
          delay(200);
        }
      }
      if (Serial.available()) Serial.read();
      radarViewMode = false;
      liveMode = true;
      Serial.println(F("\nRadar view ended."));
      break;
    }

    case 'Q': case 'q':
      quietMode = !quietMode;
      Serial.print(F("Quiet mode: ")); Serial.println(quietMode ? F("ON (LEDs only)") : F("OFF"));
      break;

    // ── Safety Thresholds ──
    case '1':
      TTC_DANGER = readIntFromSerial("DANGER TTC (1-10 sec): ", 1, 10);
      Serial.print(F("Danger TTC set to: ")); Serial.print(TTC_DANGER, 1); Serial.println(F("s"));
      break;

    case '2':
      TTC_CAUTION = readIntFromSerial("CAUTION TTC (2-20 sec): ", 2, 20);
      Serial.print(F("Caution TTC set to: ")); Serial.print(TTC_CAUTION, 1); Serial.println(F("s"));
      break;

    case '3':
      MIN_SPEED_KMH = (uint8_t)readIntFromSerial("Min speed filter (0-30 km/h): ", 0, 30);
      Serial.print(F("Min speed: ")); Serial.print(MIN_SPEED_KMH); Serial.println(F(" km/h"));
      break;

    case '4':
      MAX_DETECT_DIST = (uint8_t)readIntFromSerial("Max distance (10-100 m): ", 10, 100);
      Serial.print(F("Max distance: ")); Serial.print(MAX_DETECT_DIST); Serial.println(F("m"));
      break;

    case '5':
      MIN_SNR = (uint8_t)readIntFromSerial("Min SNR (0-100): ", 0, 100);
      Serial.print(F("Min SNR: ")); Serial.println(MIN_SNR);
      break;

    // ── Sensor Commands ──
    case 'I': case 'i':
      configureForDooring();
      break;

    case 'V': case 'v':
      enableConfig();
      sendAndPrint(0x0000, nullptr, 0, "Get Firmware Version");
      disableConfig();
      break;

    case 'R': case 'r':
      enableConfig();
      sendAndPrint(0x00A3, nullptr, 0, "Restart Sensor");
      disableConfig();
      break;

    case 'F': case 'f':
      enableConfig();
      sendAndPrint(0x00A2, nullptr, 0, "Factory Reset");
      disableConfig();
      break;

    case 'B': case 'b': {
      int choice = readIntFromSerial("\nSet Bluetooth—1=ON, 0=OFF (will restart sensor): ", 0, 1);
      uint8_t btPayload[2] = {(uint8_t)choice, 0x00};
      enableConfig();
      sendAndPrint(0x00A4, btPayload, 2, choice ? "Enable Bluetooth" : "Disable Bluetooth");
      sendAndPrint(0x00A3, nullptr, 0, "Restarting to apply changes");
      disableConfig();
      break;
    }

    // ── Diagnostics ──
    case 'S': case 's':
      printStatistics();
      break;

    case 'T': case 't':
      selfTest();
      break;

    case 'G': case 'g': {
      Serial.println(F("CSV Logger — press key to stop."));
      Serial.println(F("millis,target,angle_deg,distance_m,direction,speed_kmh,snr,ttc_s,alert"));
      while (!Serial.available()) {
        if (readTargetData()) {
          AlertLevel threat = evaluateThreat();
          setAlertLevel(threat);
          for (uint8_t i = 0; i < targetCount; i++) {
            Serial.print(millis()); Serial.print(',');
            Serial.print(i + 1); Serial.print(',');
            Serial.print(targets[i].angle); Serial.print(',');
            Serial.print(targets[i].distance); Serial.print(',');
            Serial.print(targets[i].direction == 1 ? F("approach") : F("recede")); Serial.print(',');
            Serial.print(targets[i].speed); Serial.print(',');
            Serial.print(targets[i].snr); Serial.print(',');
            Serial.print(targets[i].ttc, 2); Serial.print(',');
            printAlertName(threat);
            Serial.println();
          }
        }
      }
      Serial.read();
      Serial.println(F("Logging stopped."));
      break;
    }

    case 'P': case 'p': {
      Serial.println(F("Serial Plotter — 60s. Open Tools > Serial Plotter."));
      Serial.println(F("Closest_Distance,Fastest_Speed,TTC"));
      uint32_t end = millis() + 60000UL;
      while (millis() < end) {
        if (readTargetData()) {
          AlertLevel threat = evaluateThreat();
          setAlertLevel(threat);
          uint8_t closestDist = 255;
          uint8_t fastestSpd = 0;
          float bestTTC = 999;
          for (uint8_t i = 0; i < targetCount; i++) {
            if (targets[i].direction == 1) {
              if (targets[i].distance < closestDist) closestDist = targets[i].distance;
              if (targets[i].speed > fastestSpd) fastestSpd = targets[i].speed;
              if (targets[i].ttc < bestTTC) bestTTC = targets[i].ttc;
            }
          }
          Serial.print(closestDist == 255 ? 0 : closestDist); Serial.print(',');
          Serial.print(fastestSpd); Serial.print(',');
          Serial.println(bestTTC > 100 ? 0 : (int)(bestTTC * 10));
        }
      }
      break;
    }

    case 'X': case 'x':
      DEBUG_RAW = !DEBUG_RAW;
      Serial.print(F("Raw Hex Debug: "));
      Serial.println(DEBUG_RAW ? F("ON") : F("OFF"));
      break;

    case 'E': case 'e':
      runSimulation();
      break;

    case 'M': case 'm':
      printMenu();
      return;  // don't print menu again

    default:
      Serial.print(F("Unknown: ")); Serial.println(cmd);
      break;
  }
  printMenu();
}

// ═══════════════════════════════════════════════════════════════
//  STATISTICS
// ═══════════════════════════════════════════════════════════════
void printStatistics() {
  Serial.println(F("\n╔════════════ SESSION STATISTICS ════════════╗"));
  Serial.print(F("║ Total Detections  : ")); Serial.println(totalDetections);
  Serial.print(F("║ Danger Events     : ")); Serial.println(dangerEvents);
  Serial.print(F("║ Caution Events    : ")); Serial.println(cautionEvents);
  Serial.print(F("║ Peak Speed Seen   : ")); Serial.print(peakSpeedSeen); Serial.println(F(" km/h"));
  Serial.print(F("║ Closest TTC       : "));
  if (closestTTC < 900) { Serial.print(closestTTC, 1); Serial.println(F(" s")); }
  else Serial.println(F("N/A"));
  Serial.print(F("║ Current Alert     : ")); printAlertName(currentAlert); Serial.println();
  Serial.print(F("║ Uptime            : ")); Serial.print(millis() / 1000); Serial.println(F(" sec"));
  Serial.println(F("╚═══════════════════════════════════════════╝"));
}

// ═══════════════════════════════════════════════════════════════
//  SELF-TEST & STARTUP
// ═══════════════════════════════════════════════════════════════
void startupAnimation() {
  if (!ENABLE_HARDWARE) return;
  // Sequential LED test
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN, HIGH); delay(150); digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH); delay(150); digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH); delay(150); digitalWrite(LED_RED, LOW);
  }
  // Set to safe
  digitalWrite(LED_GREEN, HIGH);
}

void selfTest() {
  Serial.println(F("\n=== SELF TEST ==="));

  if (!ENABLE_HARDWARE) {
    Serial.println(F("  Hardware disabled (ENABLE_HARDWARE = false)."));
    Serial.println(F("  Skipping LED tests."));
  } else {
    Serial.println(F("  Testing GREEN LED..."));
    digitalWrite(LED_GREEN, HIGH); delay(500); digitalWrite(LED_GREEN, LOW);

    Serial.println(F("  Testing YELLOW LED..."));
    digitalWrite(LED_YELLOW, HIGH); delay(500); digitalWrite(LED_YELLOW, LOW);

    Serial.println(F("  Testing RED LED..."));
    digitalWrite(LED_RED, HIGH); delay(500); digitalWrite(LED_RED, LOW);
  }

  // Try talking to radar
  Serial.println(F("  Pinging radar..."));
  enableConfig();
  sendAndPrint(0x0000, nullptr, 0, "Firmware Version");
  disableConfig();

  Serial.println(F("=== SELF TEST COMPLETE ===\n"));
  setAlertLevel(ALERT_SAFE);
}

// ═══════════════════════════════════════════════════════════════
//  SIMULATION MODE — Demo without radar hardware
// ═══════════════════════════════════════════════════════════════
/*
 * Simulates a cyclist approaching from 80m away at ~22 km/h.
 * The system cycles through all alert levels:
 *   SAFE → CAUTION → DANGER → cyclist passes → SAFE
 *
 * LEDs work exactly as they would with real radar data.
 * Press any key to exit.
 */
void runSimulation() {
  simMode = true;
  Serial.println(F("\n╔═══════════════════════════════════════════════╗"));
  Serial.println(F("║  SIMULATION MODE — No Radar Required           ║"));
  Serial.println(F("║  Simulating cyclist approach scenario           ║"));
  Serial.println(F("║  Press any key to exit                          ║"));
  Serial.println(F("╚═══════════════════════════════════════════════╝\n"));

  const uint8_t SIM_SPEED = 22;  // cyclist speed km/h
  float simDist = 80.0;          // start 80m away
  float speed_ms = SIM_SPEED * 1000.0 / 3600.0;  // ~6.1 m/s
  bool passing = false;
  uint8_t simCycles = 0;

  while (!Serial.available()) {
    // Update simulated position
    if (!passing) {
      simDist -= speed_ms * 0.3;  // 300ms tick
      if (simDist < 2) {
        passing = true;  // cyclist reached us
      }
    } else {
      simDist += speed_ms * 0.3;  // receding
      if (simDist > 80) {
        // Reset for next cycle
        passing = false;
        simDist = 80.0;
        simCycles++;
        Serial.print(F("\n--- Cycle ")); Serial.print(simCycles + 1); Serial.println(F(" ---"));
      }
    }

    // Build simulated target
    targetCount = 1;
    targets[0].angle = -3 + (int8_t)(2.0 * sin((double)millis() / 500.0));
    targets[0].distance = (uint8_t)(simDist > 1.0f ? simDist : 1.0f);
    targets[0].direction = passing ? 0 : 1;  // 1=approaching, 0=receding
    targets[0].speed = SIM_SPEED;
    targets[0].snr = 100;

    // Calculate TTC
    if (!passing && targets[0].distance > 0) {
      targets[0].ttc = targets[0].distance / speed_ms;
    } else {
      targets[0].ttc = 999.0;
    }

    // Evaluate threat
    AlertLevel threat = evaluateThreat();
    setAlertLevel(threat);

    // Print status line
    Serial.print(F("[SIM] "));
    printAlertName(currentAlert);
    Serial.print(F("  Dist:")); Serial.print(targets[0].distance); Serial.print(F("m"));
    Serial.print(F("  Spd:")); Serial.print(SIM_SPEED); Serial.print(F("km/h"));
    Serial.print(F("  TTC:")); Serial.print(targets[0].ttc, 1); Serial.print(F("s"));
    Serial.print(F("  Dir:")); Serial.print(passing ? F("RECEDE") : F("APPROACH"));
    Serial.println();

    delay(300);
  }

  // Clean up
  Serial.read();  // consume the exit key
  simMode = false;
  targetCount = 0;
  setAlertLevel(ALERT_SAFE);
  Serial.println(F("\nSimulation ended.\n"));
}
