
#include "RPLidarA1Giga.h"

#include <string.h>

typedef struct __attribute__((packed)) {
    uint8_t model;
    uint16_t firmware_version;
    uint8_t hardware_version;
    uint8_t serialnum[16];
} rplidar_device_info_t;

typedef struct __attribute__((packed)) {
    uint8_t status;
    uint16_t error_code;
} rplidar_device_health_t;

typedef struct __attribute__((packed)) {
    uint8_t sync_quality;
    uint16_t angle_q6_checkbit;
    uint16_t distance_q2;
} rplidar_measurement_node_t;

static bool decodeMeasurementNode(const uint8_t raw[5], RPLidarA1Giga::Measurement& out) {
    const uint8_t syncQuality = raw[0];
    const uint16_t angleQ6Checkbit =
        static_cast<uint16_t>(raw[1]) |
        (static_cast<uint16_t>(raw[2]) << 8);
    const uint16_t distanceQ2 =
        static_cast<uint16_t>(raw[3]) |
        (static_cast<uint16_t>(raw[4]) << 8);

    const bool syncBit    = (syncQuality & 0x01u) != 0;
    const bool syncBitInv = (syncQuality & 0x02u) != 0;
    const bool checkBit   = (angleQ6Checkbit & 0x0001u) != 0;

    if ((syncBit == syncBitInv) || !checkBit) {
        return false;
    }

    out.startBit   = syncBit;
    out.quality    = static_cast<uint8_t>(syncQuality >> 2);
    out.angleDeg   = static_cast<float>(angleQ6Checkbit >> 1) / 64.0f;
    out.distanceMm = static_cast<uint16_t>(distanceQ2 >> 2);
    out.valid      = true;
    return true;
}

RPLidarA1Giga::RPLidarA1Giga()
    : _serial(nullptr),
      _baudrate(0),
      _motorCtrlPin(255),
      _motorCtrlActiveHigh(true),
      _motorPwm(255),
      _started(false),
      _motorOn(false),
      _scanning(false),
      _lastError(ERROR_NOT_INITIALIZED) {}

bool RPLidarA1Giga::begin(HardwareSerial& serial,
                          uint32_t baudrate,
                          uint8_t motorCtrlPin,
                          bool motorCtrlActiveHigh) {
    _serial = &serial;
    _baudrate = baudrate;
    _motorCtrlPin = motorCtrlPin;
    _motorCtrlActiveHigh = motorCtrlActiveHigh;
    _motorPwm = 255;
    _motorOn = false;
    _scanning = false;

    pinMode(_motorCtrlPin, OUTPUT);
    motorOff();

    _serial->begin(_baudrate);
    _started = true;
    clearInputBuffer();
    clearError();
    return true;
}

void RPLidarA1Giga::end() {
    if (_started) {
        stop(20);
        motorOff();
        _serial->end();
    }
    _started = false;
    _serial = nullptr;
    _scanning = false;
    _motorOn = false;
    setError(ERROR_NOT_INITIALIZED);
}

void RPLidarA1Giga::setMotorPwm(uint8_t pwm) {
    _motorPwm = pwm;
    if (_motorOn) {
        analogWrite(_motorCtrlPin, _motorCtrlActiveHigh ? _motorPwm : (255 - _motorPwm));
    }
}

uint8_t RPLidarA1Giga::motorPwm() const {
    return _motorPwm;
}

void RPLidarA1Giga::motorOn() {
    if (!_started) return;
    analogWrite(_motorCtrlPin, _motorCtrlActiveHigh ? _motorPwm : (255 - _motorPwm));
    _motorOn = true;
}

void RPLidarA1Giga::motorOff() {
    if (!_started) return;
    analogWrite(_motorCtrlPin, _motorCtrlActiveHigh ? 0 : 255);
    _motorOn = false;
}

bool RPLidarA1Giga::motorIsOn() const {
    return _motorOn;
}

bool RPLidarA1Giga::reset(uint32_t timeoutMs) {
    (void)timeoutMs;
    if (!ensureReady()) return false;
    _scanning = false;
    clearInputBuffer();
    if (!sendCommand(CMD_RESET)) return false;
    delay(20);
    clearInputBuffer();
    clearError();
    return true;
}

bool RPLidarA1Giga::stop(uint32_t settleMs) {
    if (!ensureReady()) return false;
    sendCommand(CMD_STOP);
    delay(settleMs);
    clearInputBuffer();
    _scanning = false;
    clearError();
    return true;
}

bool RPLidarA1Giga::getDeviceInfo(DeviceInfo& info, uint32_t timeoutMs) {
    if (!ensureReady()) return false;
    stop(20);
    clearInputBuffer();

    if (!sendCommand(CMD_GET_INFO)) return false;

    ResponseHeader header;
    if (!waitResponseHeader(header, timeoutMs)) return false;
    if (header.type != ANS_TYPE_DEVINFO) {
        setError(ERROR_BAD_RESPONSE_TYPE);
        return false;
    }
    if (header.size < sizeof(rplidar_device_info_t)) {
        setError(ERROR_BAD_RESPONSE_SIZE);
        return false;
    }

    rplidar_device_info_t raw{};
    if (!readExact(&raw, sizeof(raw), timeoutMs)) return false;

    if (header.size > sizeof(raw)) {
        uint8_t dummy[16];
        size_t remain = header.size - sizeof(raw);
        while (remain) {
            size_t chunk = remain > sizeof(dummy) ? sizeof(dummy) : remain;
            if (!readExact(dummy, chunk, timeoutMs)) return false;
            remain -= chunk;
        }
    }

    info.model = raw.model;
    info.firmwareVersion = raw.firmware_version;
    info.hardwareVersion = raw.hardware_version;
    memcpy(info.serialNumber, raw.serialnum, sizeof(info.serialNumber));

    clearError();
    return true;
}

bool RPLidarA1Giga::getHealth(Health& health, uint32_t timeoutMs) {
    if (!ensureReady()) return false;
    stop(20);
    clearInputBuffer();

    if (!sendCommand(CMD_GET_HEALTH)) return false;

    ResponseHeader header;
    if (!waitResponseHeader(header, timeoutMs)) return false;
    if (header.type != ANS_TYPE_DEVHEALTH) {
        setError(ERROR_BAD_RESPONSE_TYPE);
        return false;
    }
    if (header.size < sizeof(rplidar_device_health_t)) {
        setError(ERROR_BAD_RESPONSE_SIZE);
        return false;
    }

    rplidar_device_health_t raw{};
    if (!readExact(&raw, sizeof(raw), timeoutMs)) return false;

    if (header.size > sizeof(raw)) {
        uint8_t dummy[8];
        size_t remain = header.size - sizeof(raw);
        while (remain) {
            size_t chunk = remain > sizeof(dummy) ? sizeof(dummy) : remain;
            if (!readExact(dummy, chunk, timeoutMs)) return false;
            remain -= chunk;
        }
    }

    health.status = raw.status;
    health.errorCode = raw.error_code;
    clearError();
    return true;
}

bool RPLidarA1Giga::startScan(bool force, uint32_t timeoutMs) {
    if (!ensureReady()) return false;
    stop(20);
    clearInputBuffer();

    if (!motorIsOn()) {
        motorOn();
        delay(10);
    }

    if (!sendCommand(force ? CMD_FORCE_SCAN : CMD_SCAN)) return false;

    ResponseHeader header;
    if (!waitResponseHeader(header, timeoutMs)) return false;
    if (header.type != ANS_TYPE_MEASUREMENT) {
        setError(ERROR_BAD_RESPONSE_TYPE);
        return false;
    }
    if (header.size != sizeof(rplidar_measurement_node_t)) {
        setError(ERROR_BAD_RESPONSE_SIZE);
        return false;
    }
    if (!header.loop) {
        setError(ERROR_BAD_HEADER);
        return false;
    }

    _scanning = true;
    clearInputBuffer();   // start clean at node boundaries as well as possible
    clearError();
    return true;
}

bool RPLidarA1Giga::readMeasurement(Measurement& out, uint32_t timeoutMs) {
    out = Measurement{};

    if (!ensureReady()) return false;
    if (!_scanning) {
        setError(ERROR_ARGUMENT);
        return false;
    }

    uint8_t window[sizeof(rplidar_measurement_node_t)] = {0};
    const uint32_t startMs = millis();

    if (!readExact(window, sizeof(window), timeoutMs)) {
        return false;
    }

    while (true) {
        Measurement candidate{};
        if (decodeMeasurementNode(window, candidate)) {
            out = candidate;
            clearError();
            return true;
        }

        memmove(window, window + 1, sizeof(window) - 1);

        const uint32_t elapsedMs = millis() - startMs;
        if (elapsedMs >= timeoutMs) {
            setError(ERROR_BAD_NODE);
            return false;
        }

        const uint32_t remainingMs = timeoutMs - elapsedMs;
        if (!readByte(window[sizeof(window) - 1], remainingMs)) {
            return false;
        }
    }
}

bool RPLidarA1Giga::isScanning() const {
    return _scanning;
}

RPLidarA1Giga::ErrorCode RPLidarA1Giga::lastError() const {
    return _lastError;
}

const char* RPLidarA1Giga::lastErrorText() const {
    switch (_lastError) {
        case ERROR_NONE: return "OK";
        case ERROR_NOT_INITIALIZED: return "not initialized";
        case ERROR_TIMEOUT: return "timeout";
        case ERROR_BAD_HEADER: return "bad response header";
        case ERROR_BAD_RESPONSE_TYPE: return "unexpected response type";
        case ERROR_BAD_RESPONSE_SIZE: return "unexpected response size";
        case ERROR_BAD_NODE: return "invalid scan node";
        case ERROR_WRITE_FAILED: return "serial write failed";
        case ERROR_READ_FAILED: return "serial read failed";
        case ERROR_ARGUMENT: return "bad argument or invalid state";
        default: return "unknown";
    }
}

bool RPLidarA1Giga::ensureReady() {
    if (!_started || _serial == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return false;
    }
    return true;
}

void RPLidarA1Giga::setError(ErrorCode code) {
    _lastError = code;
}

void RPLidarA1Giga::clearError() {
    _lastError = ERROR_NONE;
}

void RPLidarA1Giga::clearInputBuffer() {
    if (!_serial) return;
    while (_serial->available() > 0) {
        _serial->read();
    }
}

bool RPLidarA1Giga::sendCommand(uint8_t cmd, const void* payload, uint8_t payloadSize) {
    if (!ensureReady()) return false;

    const bool hasPayload = payload != nullptr && payloadSize > 0;
    uint8_t checksum = 0;

    if (_serial->write(CMD_SYNC) != 1) {
        setError(ERROR_WRITE_FAILED);
        return false;
    }
    checksum ^= CMD_SYNC;

    if (_serial->write(cmd) != 1) {
        setError(ERROR_WRITE_FAILED);
        return false;
    }
    checksum ^= cmd;

    if (hasPayload) {
        if (_serial->write(payloadSize) != 1) {
            setError(ERROR_WRITE_FAILED);
            return false;
        }
        checksum ^= payloadSize;

        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(payload);
        for (uint8_t i = 0; i < payloadSize; ++i) {
            if (_serial->write(bytes[i]) != 1) {
                setError(ERROR_WRITE_FAILED);
                return false;
            }
            checksum ^= bytes[i];
        }

        if (_serial->write(checksum) != 1) {
            setError(ERROR_WRITE_FAILED);
            return false;
        }
    }

    _serial->flush();
    clearError();
    return true;
}

bool RPLidarA1Giga::waitResponseHeader(ResponseHeader& header, uint32_t timeoutMs) {
    uint8_t raw[7] = {0};
    if (!readExact(raw, sizeof(raw), timeoutMs)) return false;

    if (raw[0] != ANS_SYNC_1 || raw[1] != ANS_SYNC_2) {
        setError(ERROR_BAD_HEADER);
        return false;
    }

    const uint32_t sizeQ30Subtype = le32(raw + 2);
    header.loop = ((sizeQ30Subtype >> ANS_SUBTYPE_SHIFT) & ANS_FLAG_LOOP) != 0;
    header.size = sizeQ30Subtype & ANS_SIZE_MASK;
    header.type = raw[6];

    clearError();
    return true;
}

bool RPLidarA1Giga::readExact(void* dst, size_t len, uint32_t timeoutMs) {
    uint8_t* p = reinterpret_cast<uint8_t*>(dst);
    const uint32_t start = millis();
    size_t done = 0;

    while (done < len) {
        if (_serial->available() > 0) {
            const int c = _serial->read();
            if (c < 0) {
                setError(ERROR_READ_FAILED);
                return false;
            }
            p[done++] = static_cast<uint8_t>(c);
            continue;
        }

        if ((millis() - start) >= timeoutMs) {
            setError(ERROR_TIMEOUT);
            return false;
        }
        delayMicroseconds(50);
    }

    clearError();
    return true;
}

bool RPLidarA1Giga::readByte(uint8_t& value, uint32_t timeoutMs) {
    return readExact(&value, 1, timeoutMs);
}

uint16_t RPLidarA1Giga::le16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t RPLidarA1Giga::le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
