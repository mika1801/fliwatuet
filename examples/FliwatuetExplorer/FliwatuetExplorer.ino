/*
 * ============================================================
 *  FliwatuetExplorer.ino — Autonomous exploration robot
 *  Board  : Arduino GIGA R1 WiFi
 *  Shield : Arduino Motor Shield Rev3
 *  LiDAR  : RPLidar A1M8 (360° 2D)
 *  Motors : 2x Pololu 4751 (19:1, 64 CPR encoder)
 *
 *  This sketch initializes all subsystems and runs the
 *  autonomous exploration state machine.
 *
 *  Threading model:
 *    M7 main thread   — setup() + loop() → nav loop at 20 Hz
 *    rtos::Thread #1   — WiFi UDP telemetry logger at 10 Hz
 *    mbed::Ticker      — encoder polling ISR at 25 kHz
 *
 *  WiFi telemetry is optional. Set ENABLE_WIFI to false to
 *  disable it and skip the WiFi connection phase.
 * ============================================================
 */

#include <FliwaRobot.h>
#include "Secrets.h"   // must define WIFI_SSID, WIFI_PASSWORD, LOG_TARGET_IP
                       // copy Secrets.h.example → Secrets.h and fill in your values

// ============================================================
//  WiFi CONFIGURATION
// ============================================================
#define ENABLE_WIFI        true

// ============================================================
//  GLOBAL ROBOT INSTANCE
// ============================================================
fliwa::FliwaRobot robot;

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);  // Allow Serial Monitor to connect

    Serial.println(F(""));
    Serial.println(F("============================================"));
    Serial.println(F("  FLIWATUET — Autonomous Explorer"));
    Serial.println(F("  Arduino GIGA R1 WiFi"));
    Serial.println(F("============================================"));
    Serial.println(F(""));

    // Initialize robot subsystems
    bool ok;
    if (ENABLE_WIFI) {
        ok = robot.begin(WIFI_SSID, WIFI_PASSWORD, LOG_TARGET_IP);
    } else {
        ok = robot.begin();
    }

    if (ok) {
        Serial.println(F("[MAIN] All systems nominal"));
    } else {
        Serial.println(F("[MAIN] WARNING: LiDAR init failed, running in degraded mode"));
    }

    Serial.println(F("[MAIN] Starting navigation loop..."));
    Serial.println(F(""));
    Serial.println(F("  Serial commands:"));
    Serial.println(F("    e  = Emergency Stop"));
    Serial.println(F("    r  = Resume (after ESTOP)"));
    Serial.println(F("    s  = Print status"));
    Serial.println(F(""));
}

// ============================================================
//  LOOP — runs at full speed, internally rate-limited
// ============================================================
void loop() {
    // Process serial commands
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        while (Serial.available()) Serial.read();  // flush

        switch (cmd) {
            case 'e':
            case 'E':
                robot.emergencyStop();
                break;

            case 'r':
            case 'R':
                robot.resume();
                break;

            case 's':
            case 'S': {
                fliwa::Pose2D p = robot.pose();
                Serial.println(F("--- STATUS ---"));
                Serial.print(F("State : "));
                Serial.println(fliwa::robotStateName(robot.state()));
                Serial.print(F("Pose  : x="));
                Serial.print(p.x, 3);
                Serial.print(F(" y="));
                Serial.print(p.y, 3);
                Serial.print(F(" θ="));
                Serial.println(p.theta, 2);
                Serial.print(F("V     : "));
                Serial.print(robot.odometry().linearVelocity(), 3);
                Serial.print(F(" m/s  ω="));
                Serial.print(robot.odometry().angularVelocity(), 3);
                Serial.println(F(" rad/s"));
                Serial.print(F("PWM   : L="));
                Serial.print(robot.motor().lastPwmLeft());
                Serial.print(F(" R="));
                Serial.println(robot.motor().lastPwmRight());
                Serial.print(F("Near  : "));
                Serial.print(robot.lidar().nearestObstacleMm(400), 0);
                Serial.println(F(" mm"));
                Serial.print(F("DWA   : score="));
                Serial.println(robot.planner().lastBestScore(), 2);
                Serial.print(F("WiFi  : "));
                Serial.println(robot.logger().isConnected() ? F("connected") : F("disconnected"));
                Serial.println(F("--------------"));
                break;
            }
        }
    }

    // Main robot tick
    robot.update();
}

/*
 * ============================================================
 *  UDP LISTENER (run on your laptop to receive telemetry)
 *
 *  Python 3:
 *    import socket, json
 *    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 *    sock.bind(('0.0.0.0', 9000))
 *    while True:
 *        data, addr = sock.recvfrom(1024)
 *        print(json.loads(data.decode()))
 *
 *  netcat:
 *    nc -u -l 9000
 * ============================================================
 *
 *  HARDWARE WIRING SUMMARY
 *
 *  Motor Shield Rev3:
 *    Left  Motor  (Ch A) : DIR=D12  PWM=D3   BRAKE=D9
 *    Right Motor  (Ch B) : DIR=D13  PWM=D11  BRAKE=D8
 *
 *  Encoders:
 *    Left  : A=D4  B=D5
 *    Right : A=D6  B=D7
 *
 *  RPLidar A1M8:
 *    UART     : Serial1 (D0=RX, D1=TX)
 *    MOTO_CTRL: D2
 *
 *  Power:
 *    12V to Motor Shield VIN (motors)
 *    USB to GIGA R1 (logic)
 *    RPLidar 5V from GIGA R1 5V pin
 * ============================================================
 */
