#include "FliwaLogger.h"
#include <cstdio>
#include <cstring>

namespace fliwa {

FliwaLogger::FliwaLogger()
    : _thread(osPriorityBelowNormal, 4096, nullptr, "FliwaLog"),
      _port(LOG_UDP_PORT), _running(false), _connected(false) {
    memset(_ssid, 0, sizeof(_ssid));
    memset(_password, 0, sizeof(_password));
}

bool FliwaLogger::begin(const char* ssid, const char* password,
                         const char* remoteIP, uint16_t port) {
    strncpy(_ssid, ssid, sizeof(_ssid) - 1);
    strncpy(_password, password, sizeof(_password) - 1);
    _port = port;

    // Parse remote IP
    {
        int a, b, c, d;
        if (sscanf(remoteIP, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
            _remoteIP = IPAddress(a, b, c, d);
        } else {
            return false;
        }
    }

    // Connect to WiFi (blocking — do this at startup before nav loop)
    Serial.print(F("[LOG] Connecting to WiFi: "));
    Serial.println(_ssid);

    WiFi.begin(_ssid, _password);

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
        if (millis() - startMs > 15000) {
            Serial.println(F("\n[LOG] WiFi connection timeout"));
            return false;
        }
    }

    Serial.print(F("\n[LOG] WiFi connected, IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("[LOG] UDP target: "));
    Serial.print(_remoteIP);
    Serial.print(':');
    Serial.println(_port);

    _udp.begin(_port + 1);   // local port (different from remote to avoid collision)
    _connected = true;
    _running   = true;

    // Send a test packet so you can verify reception immediately
    _udp.beginPacket(_remoteIP, _port);
    const char* hello = "{\"msg\":\"fliwatuet online\"}";
    _udp.write((const uint8_t*)hello, strlen(hello));
    _udp.endPacket();
    Serial.println(F("[LOG] Test packet sent"));

    // Start the logger thread
    _thread.start(mbed::callback(threadFunc, this));

    return true;
}

void FliwaLogger::updateTelemetry(const TelemetryData& data) {
    _mutex.lock();
    _data = data;
    _mutex.unlock();
}

bool FliwaLogger::isConnected() const {
    return _connected;
}

void FliwaLogger::stop() {
    _running = false;
    // Thread will exit on next iteration
}

// ============================================================================
//  threadFunc() — Logger thread main loop
//
//  Runs independently from the navigation loop.
//  Sends a JSON telemetry packet at LOG_INTERVAL_MS intervals.
// ============================================================================
void FliwaLogger::threadFunc(FliwaLogger* self) {
    while (self->_running) {
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(LOG_INTERVAL_MS));

        if (WiFi.status() != WL_CONNECTED) {
            self->_connected = false;
            continue;
        }
        self->_connected = true;
        self->sendPacket();
    }
}

// ============================================================================
//  sendPacket() — Format and send a single UDP telemetry packet
// ============================================================================
void FliwaLogger::sendPacket() {
    TelemetryData snap;
    _mutex.lock();
    snap = _data;
    _mutex.unlock();

    char buf[LOG_BUFFER_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "{\"t\":%lu,"
        "\"x\":%.3f,\"y\":%.3f,\"th\":%.3f,"
        "\"v\":%.3f,\"w\":%.3f,"
        "\"tv\":%.3f,\"tw\":%.3f,"
        "\"pl\":%d,\"pr\":%d,"
        "\"near\":%.0f,"
        "\"st\":%u,"
        "\"sc\":%.2f}",
        (unsigned long)snap.uptime_ms,
        snap.x_m, snap.y_m, snap.theta_rad,
        snap.v_ms, snap.omega_rs,
        snap.target_v, snap.target_w,
        snap.pwm_left, snap.pwm_right,
        snap.nearest_mm,
        snap.state,
        snap.dwa_score
    );

    if (len > 0 && len < (int)sizeof(buf)) {
        _udp.beginPacket(_remoteIP, _port);
        _udp.write((const uint8_t*)buf, len);
        _udp.endPacket();
    }
}

} // namespace fliwa
