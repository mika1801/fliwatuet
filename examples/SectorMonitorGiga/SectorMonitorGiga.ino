#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "RPLidarA1Giga.h"

// ============================================================================
// USER SETTINGS — edit this block first
// ============================================================================

// IMPORTANT:
// On Arduino GIGA R1 WiFi, choose the HardwareSerial instance that actually maps
// to the pins you are using for the lidar UART. The hardware here is wired as:
//   RPLidar TX -> D0
//   RPLidar RX -> D1
//   MOTO_CTRL  -> D2
// If your local GIGA core maps D0/D1 to a different serial port, change only
// the line below.
#define LIDAR_SERIAL             Serial1

#define USB_SERIAL               Serial
#define USB_BAUDRATE             115200
#define LIDAR_BAUDRATE           115200
#define PIN_LIDAR_MOTOR_CTRL     2

#define MOTOR_CTRL_ACTIVE_HIGH   true
#define INITIAL_MOTOR_PWM        160

// Human-facing sector view
#define INITIAL_SECTOR_CENTER_DEG   0.0f
#define INITIAL_SECTOR_WIDTH_DEG   45.0f
#define INITIAL_ANGLE_OFFSET_DEG    0.0f
#define INITIAL_MIRROR_ANGLE       false

// Reporting / filtering
#define INITIAL_REPORT_INTERVAL_MS 1000UL
#define INITIAL_STALE_AFTER_MS     400UL
#define INITIAL_MIN_QUALITY        0

// table  : readable degree-by-degree sector dump
// summary: only a compact summary, much less Serial load
#define INITIAL_REPORT_MODE_TABLE  true

// ============================================================================
// INTERNAL TYPES
// ============================================================================

struct AngleBin {
    uint16_t distanceMm = 0;
    uint8_t quality = 0;
    uint32_t stampMs = 0;
    bool valid = false;
};

struct RuntimeStats {
    uint32_t totalNodes = 0;
    uint32_t totalValidNodes = 0;
    uint32_t totalInvalidNodes = 0;
    uint32_t totalStartBits = 0;

    uint32_t windowNodes = 0;
    uint32_t windowValidNodes = 0;
    uint32_t windowInvalidNodes = 0;
    uint32_t windowStartBits = 0;
    uint32_t windowLoops = 0;
    uint32_t windowMaxBurst = 0;
    uint32_t lastStatsResetMs = 0;
};

enum ReportMode : uint8_t {
    REPORT_OFF = 0,
    REPORT_SUMMARY = 1,
    REPORT_TABLE = 2,
};

struct Config {
    float sectorCenterDeg = INITIAL_SECTOR_CENTER_DEG;
    float sectorWidthDeg = INITIAL_SECTOR_WIDTH_DEG;
    float angleOffsetDeg = INITIAL_ANGLE_OFFSET_DEG;
    bool mirrorAngle = INITIAL_MIRROR_ANGLE;
    uint32_t reportIntervalMs = INITIAL_REPORT_INTERVAL_MS;
    uint32_t staleAfterMs = INITIAL_STALE_AFTER_MS;
    uint8_t minQuality = INITIAL_MIN_QUALITY;
    uint8_t motorPwm = INITIAL_MOTOR_PWM;
    ReportMode reportMode = INITIAL_REPORT_MODE_TABLE ? REPORT_TABLE : REPORT_SUMMARY;
};

// ============================================================================
// GLOBALS
// ============================================================================

RPLidarA1Giga lidar;
AngleBin gBins[360];
RuntimeStats gStats;
Config gCfg;

uint32_t gLastReportMs = 0;
char gCmdBuf[96];
size_t gCmdLen = 0;

// ============================================================================
// HELPERS
// ============================================================================

static float normalize360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

static float shortestSignedAngleDelta(float fromDeg, float toDeg) {
    float d = normalize360(toDeg) - normalize360(fromDeg);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

static bool sectorContains(float centerDeg, float widthDeg, int testDeg) {
    const float half = widthDeg * 0.5f;
    const float d = shortestSignedAngleDelta(centerDeg, (float)testDeg);
    return fabsf(d) <= half;
}

static int logicalBinFromPhysical(float physicalDeg) {
    float logical = normalize360(physicalDeg);
    if (gCfg.mirrorAngle) {
        logical = normalize360(360.0f - logical);
    }
    logical = normalize360(logical + gCfg.angleOffsetDeg);

    int bin = (int)lroundf(logical);
    if (bin >= 360) bin -= 360;
    if (bin < 0) bin += 360;
    return bin;
}

static const char* reportModeName(ReportMode mode) {
    switch (mode) {
        case REPORT_OFF: return "off";
        case REPORT_SUMMARY: return "summary";
        case REPORT_TABLE: return "table";
        default: return "unknown";
    }
}

static const char* healthText(uint8_t status) {
    switch (status) {
        case 0: return "OK";
        case 1: return "WARNING";
        case 2: return "ERROR";
        default: return "UNKNOWN";
    }
}

static void printDivider() {
    USB_SERIAL.println(F("------------------------------------------------------------------"));
}

static void printHelp() {
    printDivider();
    USB_SERIAL.println(F("RPLidar A1 sector monitor — commands"));
    printDivider();
    USB_SERIAL.println(F("help                     -> show this help"));
    USB_SERIAL.println(F("status                   -> show current settings"));
    USB_SERIAL.println(F("dump                     -> print sector immediately"));
    USB_SERIAL.println(F("stats                    -> print performance counters immediately"));
    USB_SERIAL.println(F("sector <center> <width>  -> e.g. sector 0 45"));
    USB_SERIAL.println(F("offset <deg>             -> rotate logical frame, e.g. offset 180"));
    USB_SERIAL.println(F("mirror on|off            -> flip angle direction"));
    USB_SERIAL.println(F("interval <ms>            -> report interval"));
    USB_SERIAL.println(F("stale <ms>               -> max age of a bin before shown as stale"));
    USB_SERIAL.println(F("quality <0..63>          -> minimum accepted quality"));
    USB_SERIAL.println(F("mode table|summary|off   -> output style / serial load"));
    USB_SERIAL.println(F("pwm <0..255>             -> set MOTO_CTRL PWM"));
    USB_SERIAL.println(F("motor on|off             -> enable or disable motor control pin"));
    USB_SERIAL.println(F("clear                    -> clear 360 bins"));
    printDivider();
}

static void clearBins() {
    for (int i = 0; i < 360; ++i) {
        gBins[i] = AngleBin{};
    }
}

static void resetStatsWindow() {
    gStats.windowNodes = 0;
    gStats.windowValidNodes = 0;
    gStats.windowInvalidNodes = 0;
    gStats.windowStartBits = 0;
    gStats.windowLoops = 0;
    gStats.windowMaxBurst = 0;
    gStats.lastStatsResetMs = millis();
}

static void printStatus() {
    printDivider();
    USB_SERIAL.println(F("Current settings"));
    printDivider();
    USB_SERIAL.print(F("sector center deg : ")); USB_SERIAL.println(gCfg.sectorCenterDeg, 1);
    USB_SERIAL.print(F("sector width deg  : ")); USB_SERIAL.println(gCfg.sectorWidthDeg, 1);
    USB_SERIAL.print(F("angle offset deg  : ")); USB_SERIAL.println(gCfg.angleOffsetDeg, 1);
    USB_SERIAL.print(F("mirror angle      : ")); USB_SERIAL.println(gCfg.mirrorAngle ? F("on") : F("off"));
    USB_SERIAL.print(F("report interval   : ")); USB_SERIAL.println(gCfg.reportIntervalMs);
    USB_SERIAL.print(F("stale after ms    : ")); USB_SERIAL.println(gCfg.staleAfterMs);
    USB_SERIAL.print(F("min quality       : ")); USB_SERIAL.println(gCfg.minQuality);
    USB_SERIAL.print(F("report mode       : ")); USB_SERIAL.println(reportModeName(gCfg.reportMode));
    USB_SERIAL.print(F("motor pwm         : ")); USB_SERIAL.println(gCfg.motorPwm);
    USB_SERIAL.print(F("motor state       : ")); USB_SERIAL.println(lidar.motorIsOn() ? F("on") : F("off"));
    USB_SERIAL.print(F("lidar scanning    : ")); USB_SERIAL.println(lidar.isScanning() ? F("yes") : F("no"));
    printDivider();
}

static void printStats(bool resetWindowAfterPrint) {
    const uint32_t now = millis();
    uint32_t dt = now - gStats.lastStatsResetMs;
    if (dt == 0) dt = 1;

    const float nodesPerSec = (1000.0f * gStats.windowNodes) / (float)dt;
    const float validPerSec = (1000.0f * gStats.windowValidNodes) / (float)dt;
    const float scansPerSec = (1000.0f * gStats.windowStartBits) / (float)dt;
    const float loopHz = (1000.0f * gStats.windowLoops) / (float)dt;

    printDivider();
    USB_SERIAL.println(F("Performance / load view"));
    printDivider();
    USB_SERIAL.print(F("window ms         : ")); USB_SERIAL.println(dt);
    USB_SERIAL.print(F("nodes/s           : ")); USB_SERIAL.println(nodesPerSec, 1);
    USB_SERIAL.print(F("valid nodes/s     : ")); USB_SERIAL.println(validPerSec, 1);
    USB_SERIAL.print(F("scans/s           : ")); USB_SERIAL.println(scansPerSec, 2);
    USB_SERIAL.print(F("loop Hz           : ")); USB_SERIAL.println(loopHz, 1);
    USB_SERIAL.print(F("max read burst    : ")); USB_SERIAL.println(gStats.windowMaxBurst);
    USB_SERIAL.print(F("total nodes       : ")); USB_SERIAL.println(gStats.totalNodes);
    USB_SERIAL.print(F("total valid       : ")); USB_SERIAL.println(gStats.totalValidNodes);
    USB_SERIAL.print(F("total invalid     : ")); USB_SERIAL.println(gStats.totalInvalidNodes);
    USB_SERIAL.print(F("total start bits  : ")); USB_SERIAL.println(gStats.totalStartBits);
    printDivider();

    if (resetWindowAfterPrint) {
        resetStatsWindow();
    }
}

static void printSectorReport() {
    const uint32_t now = millis();
    int validCount = 0;
    int staleCount = 0;
    uint16_t nearestMm = 0xFFFF;
    int nearestDeg = -1;

    printDivider();
    USB_SERIAL.println(F("Sector report"));
    printDivider();
    USB_SERIAL.print(F("logical center/width : "));
    USB_SERIAL.print(gCfg.sectorCenterDeg, 1);
    USB_SERIAL.print(F(" / "));
    USB_SERIAL.println(gCfg.sectorWidthDeg, 1);
    USB_SERIAL.print(F("offset / mirror      : "));
    USB_SERIAL.print(gCfg.angleOffsetDeg, 1);
    USB_SERIAL.print(F(" / "));
    USB_SERIAL.println(gCfg.mirrorAngle ? F("on") : F("off"));

    for (int deg = 0; deg < 360; ++deg) {
        if (!sectorContains(gCfg.sectorCenterDeg, gCfg.sectorWidthDeg, deg)) {
            continue;
        }

        const AngleBin& bin = gBins[deg];
        const bool fresh = bin.valid && ((now - bin.stampMs) <= gCfg.staleAfterMs);
        if (fresh) {
            ++validCount;
            if (bin.distanceMm > 0 && bin.distanceMm < nearestMm) {
                nearestMm = bin.distanceMm;
                nearestDeg = deg;
            }
        } else {
            ++staleCount;
        }
    }

    USB_SERIAL.print(F("fresh bins          : ")); USB_SERIAL.println(validCount);
    USB_SERIAL.print(F("stale/empty bins    : ")); USB_SERIAL.println(staleCount);
    if (nearestDeg >= 0) {
        USB_SERIAL.print(F("nearest             : "));
        USB_SERIAL.print(nearestMm);
        USB_SERIAL.print(F(" mm @ "));
        USB_SERIAL.print(nearestDeg);
        USB_SERIAL.println(F(" deg"));
    } else {
        USB_SERIAL.println(F("nearest             : none"));
    }

    if (gCfg.reportMode == REPORT_TABLE) {
        printDivider();
        USB_SERIAL.println(F("deg | distance | quality | age"));
        printDivider();

        for (int deg = 0; deg < 360; ++deg) {
            if (!sectorContains(gCfg.sectorCenterDeg, gCfg.sectorWidthDeg, deg)) {
                continue;
            }

            const AngleBin& bin = gBins[deg];
            USB_SERIAL.print(deg);
            USB_SERIAL.print(F(" | "));

            if (bin.valid) {
                const uint32_t age = now - bin.stampMs;
                if (age <= gCfg.staleAfterMs) {
                    USB_SERIAL.print(bin.distanceMm);
                    USB_SERIAL.print(F(" mm | q="));
                    USB_SERIAL.print(bin.quality);
                    USB_SERIAL.print(F(" | "));
                    USB_SERIAL.print(age);
                    USB_SERIAL.println(F(" ms"));
                } else {
                    USB_SERIAL.print(F("stale | q="));
                    USB_SERIAL.print(bin.quality);
                    USB_SERIAL.print(F(" | "));
                    USB_SERIAL.print(age);
                    USB_SERIAL.println(F(" ms"));
                }
            } else {
                USB_SERIAL.println(F("---- | ---- | ----"));
            }
        }
    }

    printDivider();
}

static void ingestMeasurement(const RPLidarA1Giga::Measurement& m) {
    ++gStats.totalNodes;
    ++gStats.windowNodes;

    if (m.startBit) {
        ++gStats.totalStartBits;
        ++gStats.windowStartBits;
    }

    if (!m.valid || m.distanceMm == 0 || m.quality < gCfg.minQuality) {
        ++gStats.totalInvalidNodes;
        ++gStats.windowInvalidNodes;
        return;
    }

    ++gStats.totalValidNodes;
    ++gStats.windowValidNodes;

    const int bin = logicalBinFromPhysical(m.angleDeg);
    AngleBin& dst = gBins[bin];
    dst.distanceMm = m.distanceMm;
    dst.quality = m.quality;
    dst.stampMs = millis();
    dst.valid = true;
}

static bool parseBoolWord(const char* s, bool* out) {
    if (!s || !out) return false;
    if (strcmp(s, "on") == 0 || strcmp(s, "1") == 0 || strcmp(s, "true") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(s, "off") == 0 || strcmp(s, "0") == 0 || strcmp(s, "false") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static void lowercaseInPlace(char* s) {
    if (!s) return;
    while (*s) {
        *s = (char)tolower((unsigned char)*s);
        ++s;
    }
}

static void handleCommand(char* line) {
    lowercaseInPlace(line);

    char* cmd = strtok(line, " \t");
    if (!cmd) return;

    if (strcmp(cmd, "help") == 0) {
        printHelp();
        return;
    }

    if (strcmp(cmd, "status") == 0) {
        printStatus();
        return;
    }

    if (strcmp(cmd, "dump") == 0) {
        printSectorReport();
        return;
    }

    if (strcmp(cmd, "stats") == 0) {
        printStats(false);
        return;
    }

    if (strcmp(cmd, "clear") == 0) {
        clearBins();
        USB_SERIAL.println(F("[OK] bins cleared"));
        return;
    }

    if (strcmp(cmd, "sector") == 0) {
        char* a = strtok(nullptr, " \t");
        char* b = strtok(nullptr, " \t");
        if (!a || !b) {
            USB_SERIAL.println(F("[ERR] usage: sector <center> <width>"));
            return;
        }
        gCfg.sectorCenterDeg = normalize360((float)atof(a));
        gCfg.sectorWidthDeg = constrain((int)lroundf((float)atof(b)), 1, 359);
        USB_SERIAL.println(F("[OK] sector updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "offset") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
            USB_SERIAL.println(F("[ERR] usage: offset <deg>"));
            return;
        }
        gCfg.angleOffsetDeg = normalize360((float)atof(a));
        USB_SERIAL.println(F("[OK] angle offset updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "mirror") == 0) {
        char* a = strtok(nullptr, " \t");
        bool v = false;
        if (!parseBoolWord(a, &v)) {
            USB_SERIAL.println(F("[ERR] usage: mirror on|off"));
            return;
        }
        gCfg.mirrorAngle = v;
        USB_SERIAL.println(F("[OK] mirror updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "interval") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
            USB_SERIAL.println(F("[ERR] usage: interval <ms>"));
            return;
        }
        gCfg.reportIntervalMs = max((uint32_t)100, (uint32_t)atol(a));
        USB_SERIAL.println(F("[OK] report interval updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "stale") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
            USB_SERIAL.println(F("[ERR] usage: stale <ms>"));
            return;
        }
        gCfg.staleAfterMs = max((uint32_t)100, (uint32_t)atol(a));
        USB_SERIAL.println(F("[OK] stale limit updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "quality") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
            USB_SERIAL.println(F("[ERR] usage: quality <0..63>"));
            return;
        }
        long q = atol(a);
        if (q < 0) q = 0;
        if (q > 63) q = 63;
        gCfg.minQuality = (uint8_t)q;
        USB_SERIAL.println(F("[OK] min quality updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "mode") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
            USB_SERIAL.println(F("[ERR] usage: mode table|summary|off"));
            return;
        }
        if (strcmp(a, "table") == 0) {
            gCfg.reportMode = REPORT_TABLE;
        } else if (strcmp(a, "summary") == 0) {
            gCfg.reportMode = REPORT_SUMMARY;
        } else if (strcmp(a, "off") == 0) {
            gCfg.reportMode = REPORT_OFF;
        } else {
            USB_SERIAL.println(F("[ERR] usage: mode table|summary|off"));
            return;
        }
        USB_SERIAL.println(F("[OK] report mode updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "pwm") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
            USB_SERIAL.println(F("[ERR] usage: pwm <0..255>"));
            return;
        }
        long pwm = atol(a);
        if (pwm < 0) pwm = 0;
        if (pwm > 255) pwm = 255;
        gCfg.motorPwm = (uint8_t)pwm;
        lidar.setMotorPwm(gCfg.motorPwm);
        USB_SERIAL.println(F("[OK] motor pwm updated"));
        printStatus();
        return;
    }

    if (strcmp(cmd, "motor") == 0) {
        char* a = strtok(nullptr, " \t");
        bool on = false;
        if (!parseBoolWord(a, &on)) {
            USB_SERIAL.println(F("[ERR] usage: motor on|off"));
            return;
        }
        if (on) {
            lidar.motorOn();
        } else {
            lidar.motorOff();
        }
        USB_SERIAL.println(F("[OK] motor state updated"));
        printStatus();
        return;
    }

    USB_SERIAL.println(F("[ERR] unknown command — type help"));
}

static void serviceCommands() {
    while (USB_SERIAL.available() > 0) {
        const int c = USB_SERIAL.read();
        if (c < 0) return;

        if (c == '\r') continue;

        if (c == '\n') {
            gCmdBuf[gCmdLen] = '\0';
            if (gCmdLen > 0) {
                handleCommand(gCmdBuf);
            }
            gCmdLen = 0;
            continue;
        }

        if (gCmdLen + 1 < sizeof(gCmdBuf)) {
            gCmdBuf[gCmdLen++] = (char)c;
        }
    }
}

static bool bringUpLidar() {
    lidar.begin(LIDAR_SERIAL, LIDAR_BAUDRATE, PIN_LIDAR_MOTOR_CTRL, MOTOR_CTRL_ACTIVE_HIGH);
    lidar.setMotorPwm(gCfg.motorPwm);
    lidar.motorOn();
    delay(200);

    RPLidarA1Giga::DeviceInfo info;
    if (!lidar.getDeviceInfo(info, 300)) {
        USB_SERIAL.print(F("[ERR] getDeviceInfo failed: "));
        USB_SERIAL.println(lidar.lastErrorText());
        return false;
    }

    RPLidarA1Giga::Health health;
    if (!lidar.getHealth(health, 300)) {
        USB_SERIAL.print(F("[ERR] getHealth failed: "));
        USB_SERIAL.println(lidar.lastErrorText());
        return false;
    }

    printDivider();
    USB_SERIAL.println(F("RPLidar detected"));
    printDivider();
    USB_SERIAL.print(F("model             : 0x"));
    USB_SERIAL.println(info.model, HEX);
    USB_SERIAL.print(F("firmware          : "));
    USB_SERIAL.print((info.firmwareVersion >> 8) & 0xFF);
    USB_SERIAL.print('.');
    USB_SERIAL.println(info.firmwareVersion & 0xFF);
    USB_SERIAL.print(F("hardware          : "));
    USB_SERIAL.println(info.hardwareVersion);
    USB_SERIAL.print(F("health            : "));
    USB_SERIAL.print(healthText(health.status));
    USB_SERIAL.print(F(" (code "));
    USB_SERIAL.print(health.errorCode);
    USB_SERIAL.println(F(")"));
    printDivider();

    if (!lidar.startScan(false, 400)) {
        USB_SERIAL.print(F("[ERR] startScan failed: "));
        USB_SERIAL.println(lidar.lastErrorText());
        return false;
    }

    USB_SERIAL.println(F("[OK] scan started"));
    return true;
}

void setup() {
    USB_SERIAL.begin(USB_BAUDRATE);
    delay(2000);

    clearBins();
    resetStatsWindow();

    printDivider();
    USB_SERIAL.println(F("RPLidar A1M8 sector monitor for Arduino GIGA R1 WiFi"));
    USB_SERIAL.println(F("Goal: readable sector output instead of unreadable 360-degree spam"));
    printDivider();

    printHelp();
    printStatus();

    if (!bringUpLidar()) {
        USB_SERIAL.println(F("[FATAL] lidar init failed. Check UART wiring and baudrate mapping."));
    }
}

void loop() {
    ++gStats.windowLoops;
    serviceCommands();

    uint32_t burst = 0;
    const uint32_t sliceStartUs = micros();

    while ((micros() - sliceStartUs) < 10000UL) {
        RPLidarA1Giga::Measurement m;
        if (!lidar.readMeasurement(m, 5)) {
            continue;
        }
        ingestMeasurement(m);
    }

    if (burst > gStats.windowMaxBurst) {
        gStats.windowMaxBurst = burst;
    }

    const uint32_t now = millis();
    if (gCfg.reportMode != REPORT_OFF && (now - gLastReportMs) >= gCfg.reportIntervalMs) {
        if (gCfg.reportMode == REPORT_SUMMARY || gCfg.reportMode == REPORT_TABLE) {
            printSectorReport();
            printStats(true);
        }
        gLastReportMs = now;
    }
}
