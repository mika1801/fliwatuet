#pragma once

#include <Arduino.h>

class RPLidarA1Giga {
public:
    struct DeviceInfo {
        uint8_t model = 0;
        uint16_t firmwareVersion = 0;
        uint8_t hardwareVersion = 0;
        uint8_t serialNumber[16] = {0};
    };

    struct Health {
        uint8_t status = 0;
        uint16_t errorCode = 0;
    };

    struct Measurement {
        float angleDeg = 0.0f;       // physical angle reported by the lidar, 0..360
        uint16_t distanceMm = 0;     // 0 == invalid / no return
        uint8_t quality = 0;         // 0..63 for standard scan mode
        bool startBit = false;       // true at the beginning of a new revolution
        bool valid = false;
    };

    enum ErrorCode : uint8_t {
        ERROR_NONE = 0,
        ERROR_NOT_INITIALIZED,
        ERROR_TIMEOUT,
        ERROR_BAD_HEADER,
        ERROR_BAD_RESPONSE_TYPE,
        ERROR_BAD_RESPONSE_SIZE,
        ERROR_BAD_NODE,
        ERROR_WRITE_FAILED,
        ERROR_READ_FAILED,
        ERROR_ARGUMENT,
    };

    RPLidarA1Giga();

    bool begin(HardwareSerial& serial,
               uint32_t baudrate,
               uint8_t motorCtrlPin,
               bool motorCtrlActiveHigh = true);

    void end();

    void setMotorPwm(uint8_t pwm);
    uint8_t motorPwm() const;
    void motorOn();
    void motorOff();
    bool motorIsOn() const;

    bool reset(uint32_t timeoutMs = 200);
    bool stop(uint32_t settleMs = 20);
    bool getDeviceInfo(DeviceInfo& info, uint32_t timeoutMs = 200);
    bool getHealth(Health& health, uint32_t timeoutMs = 200);
    bool startScan(bool force = false, uint32_t timeoutMs = 250);
    bool readMeasurement(Measurement& out, uint32_t timeoutMs = 25);

    bool isScanning() const;

    ErrorCode lastError() const;
    const char* lastErrorText() const;

private:
    static constexpr uint8_t CMD_SYNC = 0xA5;
    static constexpr uint8_t CMD_FLAG_HAS_PAYLOAD = 0x80;
    static constexpr uint8_t CMD_STOP = 0x25;
    static constexpr uint8_t CMD_SCAN = 0x20;
    static constexpr uint8_t CMD_FORCE_SCAN = 0x21;
    static constexpr uint8_t CMD_RESET = 0x40;
    static constexpr uint8_t CMD_GET_INFO = 0x50;
    static constexpr uint8_t CMD_GET_HEALTH = 0x52;

    static constexpr uint8_t ANS_SYNC_1 = 0xA5;
    static constexpr uint8_t ANS_SYNC_2 = 0x5A;
    static constexpr uint8_t ANS_TYPE_DEVINFO = 0x04;
    static constexpr uint8_t ANS_TYPE_DEVHEALTH = 0x06;
    static constexpr uint8_t ANS_TYPE_MEASUREMENT = 0x81;

    static constexpr uint32_t ANS_SIZE_MASK = 0x3FFFFFFFUL;
    static constexpr uint8_t ANS_SUBTYPE_SHIFT = 30;
    static constexpr uint32_t ANS_FLAG_LOOP = 0x1UL;

    struct ResponseHeader {
        uint32_t size = 0;
        uint8_t type = 0;
        bool loop = false;
    };

    bool ensureReady();
    void setError(ErrorCode code);
    void clearError();

    void clearInputBuffer();
    bool sendCommand(uint8_t cmd, const void* payload = nullptr, uint8_t payloadSize = 0);
    bool waitResponseHeader(ResponseHeader& header, uint32_t timeoutMs);
    bool readExact(void* dst, size_t len, uint32_t timeoutMs);
    bool readByte(uint8_t& value, uint32_t timeoutMs);

    static uint16_t le16(const uint8_t* p);
    static uint32_t le32(const uint8_t* p);

private:
    HardwareSerial* _serial;
    uint32_t _baudrate;
    uint8_t _motorCtrlPin;
    bool _motorCtrlActiveHigh;
    uint8_t _motorPwm;
    bool _started;
    bool _motorOn;
    bool _scanning;
    ErrorCode _lastError;
};
