#pragma once
// ============================================================================
//  FliwaLogger.h — Asynchronous WiFi UDP telemetry logger
//
//  Runs in a dedicated Mbed OS rtos::Thread so that network latency
//  never blocks the navigation loop. Shared data is protected by a Mutex.
//
//  Sends JSON-formatted telemetry packets via UDP to a configurable host.
// ============================================================================

#include <Arduino.h>
#include <mbed.h>
#include <rtos.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FliwaConfig.h"
#include "FliwaOdometry.h"

namespace fliwa {

// ============================================================================
//  Telemetry snapshot — written by nav thread, read by logger thread
// ============================================================================
struct TelemetryData {
    // Pose
    float x_m        = 0.0f;
    float y_m        = 0.0f;
    float theta_rad  = 0.0f;

    // Velocities
    float v_ms       = 0.0f;
    float omega_rs   = 0.0f;

    // Motor
    int pwm_left     = 0;
    int pwm_right    = 0;
    float target_v   = 0.0f;
    float target_w   = 0.0f;

    // Lidar
    float nearest_mm = 0.0f;

    // State machine
    uint8_t state    = 0;

    // DWA
    float dwa_score  = 0.0f;

    // Timestamp
    uint32_t uptime_ms = 0;
};

// ============================================================================
//  FliwaLogger
// ============================================================================
class FliwaLogger {
public:
    FliwaLogger();

    /// Connect to WiFi and start the logger thread.
    /// ssid/password: WiFi credentials.
    /// remoteIP: IP address of the UDP listener (e.g., your laptop).
    /// port: UDP port (default LOG_UDP_PORT).
    bool begin(const char* ssid, const char* password,
               const char* remoteIP, uint16_t port = LOG_UDP_PORT);

    /// Update the telemetry snapshot (call from nav thread).
    /// Thread-safe (protected by mutex).
    void updateTelemetry(const TelemetryData& data);

    /// Check if WiFi is connected.
    bool isConnected() const;

    /// Stop the logger thread.
    void stop();

private:
    /// Thread entry point — runs the UDP send loop.
    static void threadFunc(FliwaLogger* self);

    void sendPacket();

    rtos::Thread _thread;
    rtos::Mutex  _mutex;

    WiFiUDP  _udp;
    IPAddress _remoteIP;
    uint16_t  _port;

    TelemetryData _data;
    bool _running;
    bool _connected;

    char _ssid[64];
    char _password[64];
};

} // namespace fliwa
