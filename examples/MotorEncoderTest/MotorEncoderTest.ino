/*
 * ============================================================
 *  MOTOR & ENCODER WIRING TEST SKETCH
 *  Board  : Arduino GIGA R1 WiFi
 *  Shield : Arduino Motor Shield Rev3
 *  Motors : 2x Pololu 19:1 Metal Gearmotor 37Dx68L 12V
 *           with integrated 64 CPR quadrature encoder
 *  Drive  : Differential drive — Left = Channel A, Right = Channel B
 * ============================================================
 *
 *  ENCODER COUNTING: polling only via mbed::Ticker at 40 µs (25 kHz)
 *  attachInterrupt() is intentionally NOT used (GIGA instability).
 *
 *  SPEED HEADROOM (40 µs poll):
 *    Max reliable output speed = 617 RPM  (530 RPM target = 86 % of max)
 *    Calculation: 1/(2×40µs×64counts/rev)/19 gearbox × 60 = 617 RPM
 *
 *  MOTOR SHIELD REV3 PINS:
 *    Channel A (Left)  — DIR=D12  PWM=D3   BRAKE=D9
 *    Channel B (Right) — DIR=D13  PWM=D11  BRAKE=D8
 *
 *  ENCODER PINS:
 *    Left  A=D4  B=D5
 *    Right A=D6  B=D7
 *
 *  ENCODER RESOLUTION:
 *    64 CPR at motor shaft (= 4x quadrature counting of 16 pulse/rev)
 *    19:1 gearbox  ->  64 x 19 = 1216 counts per output shaft revolution
 * ============================================================
 */

#include <mbed.h>   // Required for mbed::Ticker on Arduino GIGA R1 WiFi

// ============================================================
//  USER SETTINGS — edit only this section before uploading
// ============================================================

// Set true if a motor spins the wrong direction for "forward"
#define LEFT_MOTOR_INVERT    true
#define RIGHT_MOTOR_INVERT   false

// Set true if an encoder counts in the wrong direction
#define LEFT_ENCODER_INVERT  true
#define RIGHT_ENCODER_INVERT false

// Encoder poll interval in microseconds — DO NOT increase beyond 46 µs for 530 RPM!
//
//  Poll µs | Poll kHz | Max output RPM (19:1, 64 CPR)
//  --------|----------|------------------------------
//    25 µs |  40 kHz  |  986 RPM   (solid headroom)
//    40 µs |  25 kHz  |  617 RPM   (good for 530 RPM target, ~16% reserve)
//    50 µs |  20 kHz  |  494 RPM   (INSUFFICIENT for 530 RPM — do not use)
//   100 µs |  10 kHz  |  247 RPM   (only safe up to ~half speed)
//   200 µs |   5 kHz  |  123 RPM   (too slow for this motor)
#define POLL_INTERVAL_US     40

// PWM for all test phases and default manual speed (0–255)
#define TEST_PWM             150

// Duration of each auto-test movement phase (ms)
#define TEST_DURATION_MS     2000UL

// Status print interval during any running phase (ms)
#define PRINT_INTERVAL_MS    250UL

// ============================================================
//  PIN DEFINITIONS — Motor Shield Rev3
// ============================================================
#define PIN_A_DIR     12   // Channel A direction  (left motor)
#define PIN_A_PWM      3   // Channel A PWM/enable
#define PIN_A_BRAKE    9   // Channel A brake

#define PIN_B_DIR     13   // Channel B direction  (right motor)
#define PIN_B_PWM     11   // Channel B PWM/enable
#define PIN_B_BRAKE    8   // Channel B brake

// ============================================================
//  PIN DEFINITIONS — Encoders
// ============================================================
#define PIN_ENC_L_A    4
#define PIN_ENC_L_B    5
#define PIN_ENC_R_A    6
#define PIN_ENC_R_B    7

// ============================================================
//  QUADRATURE DECODE LOOKUP TABLE
//
//  state = (A << 1) | B         (2-bit value, 0–3)
//  index = (prevState << 2) | newState   (4-bit, 0–15)
//
//  Forward rotation (A leads B by 90 deg):
//    state sequence: 00 -> 10 -> 11 -> 01 -> 00 -> ...  each step = +1
//  Reverse rotation:
//    state sequence: 00 -> 01 -> 11 -> 10 -> 00 -> ...  each step = -1
//  No change or invalid (skipped step): 0
// ============================================================
static const int8_t QEM[16] = {
//  prev\new:  00   01   10   11
/* 00 */        0,  -1,  +1,   0,
/* 01 */       +1,   0,   0,  -1,
/* 10 */       -1,   0,   0,  +1,
/* 11 */        0,  +1,  -1,   0
};

// ============================================================
//  VOLATILE ENCODER STATE
//  Written by the Ticker callback, read by loop() via snapshot.
// ============================================================
volatile long    gEncL      = 0;   // Left  encoder count
volatile long    gEncR      = 0;   // Right encoder count
volatile uint8_t gEncLState = 0;   // Left  last sampled AB state
volatile uint8_t gEncRState = 0;   // Right last sampled AB state

// ============================================================
//  TICKER OBJECT
// ============================================================
mbed::Ticker encTicker;

// ============================================================
//  RUNTIME STATE (loop() only — no volatile needed here)
// ============================================================
int  gCurrentPWM  = TEST_PWM;
int  gLeftDir     = 0;    // +1=fwd, -1=rev, 0=stop
int  gRightDir    = 0;
bool gLeftActive  = false;
bool gRightActive = false;

const char* gPhaseName = "INIT";

// For delta/cps calculation — updated by printStatus()
long          gLastEncL   = 0;
long          gLastEncR   = 0;
unsigned long gLastPrintT = 0;

// ============================================================
//  encSample() — Ticker callback, fires every POLL_INTERVAL_US µs
//  Runs in interrupt context on the STM32H747 M7 core.
//  Keep it short: no heap allocation, no Serial, no delay.
//  CPU budget at 40 µs / 480 MHz: ~19,200 cycles per call — more than enough.
// ============================================================
void encSample() {
    // --- Left encoder ---
    uint8_t la    = digitalRead(PIN_ENC_L_A) ? 1u : 0u;
    uint8_t lb    = digitalRead(PIN_ENC_L_B) ? 1u : 0u;
    uint8_t newLS = (la << 1) | lb;
    int8_t  dL    = QEM[(gEncLState << 2) | newLS];
    gEncLState    = newLS;
    gEncL        += LEFT_ENCODER_INVERT ? -dL : dL;

    // --- Right encoder ---
    uint8_t ra    = digitalRead(PIN_ENC_R_A) ? 1u : 0u;
    uint8_t rb    = digitalRead(PIN_ENC_R_B) ? 1u : 0u;
    uint8_t newRS = (ra << 1) | rb;
    int8_t  dR    = QEM[(gEncRState << 2) | newRS];
    gEncRState    = newRS;
    gEncR        += RIGHT_ENCODER_INVERT ? -dR : dR;
}

// ============================================================
//  ATOMIC ENCODER SNAPSHOT
//  Disables interrupts briefly to get a consistent reading.
// ============================================================
void snapshotEncoders(long &lc, long &rc, uint8_t &ls, uint8_t &rs) {
    noInterrupts();
    lc = gEncL;
    rc = gEncR;
    ls = gEncLState;
    rs = gEncRState;
    interrupts();
}

// ============================================================
//  LOW-LEVEL MOTOR HELPERS
// ============================================================

// Brake: PWM to 0 first, then assert brake line.
void motorHardStop(uint8_t pinDir, uint8_t pinPWM, uint8_t pinBrake) {
    analogWrite(pinPWM, 0);
    digitalWrite(pinBrake, HIGH);
}

// Drive: release brake, set direction, then apply PWM.
void motorDrive(uint8_t pinDir, uint8_t pinPWM, uint8_t pinBrake,
                bool forward, uint8_t pwm) {
    digitalWrite(pinBrake, LOW);
    digitalWrite(pinDir,   forward ? HIGH : LOW);
    analogWrite(pinPWM,    pwm);
}

// ============================================================
//  HIGH-LEVEL MOTOR CONTROL (handles INVERT flags)
// ============================================================

// direction: +1=forward, -1=backward, 0=stop
void setLeftMotor(int direction, int pwm) {
    gLeftDir = direction;
    if (direction == 0) {
        motorHardStop(PIN_A_DIR, PIN_A_PWM, PIN_A_BRAKE);
        gLeftActive = false;
    } else {
        bool fwd = (direction > 0);
        if (LEFT_MOTOR_INVERT) fwd = !fwd;
        motorDrive(PIN_A_DIR, PIN_A_PWM, PIN_A_BRAKE, fwd, (uint8_t)pwm);
        gLeftActive = true;
    }
}

void setRightMotor(int direction, int pwm) {
    gRightDir = direction;
    if (direction == 0) {
        motorHardStop(PIN_B_DIR, PIN_B_PWM, PIN_B_BRAKE);
        gRightActive = false;
    } else {
        bool fwd = (direction > 0);
        if (RIGHT_MOTOR_INVERT) fwd = !fwd;
        motorDrive(PIN_B_DIR, PIN_B_PWM, PIN_B_BRAKE, fwd, (uint8_t)pwm);
        gRightActive = true;
    }
}

void stopAll() {
    setLeftMotor(0, 0);
    setRightMotor(0, 0);
}

// ============================================================
//  DELTA BASELINE RESET
//  Call after every phase/command change so dL/dR start fresh.
// ============================================================
void resetDeltaBaseline() {
    long lc, rc; uint8_t ls, rs;
    snapshotEncoders(lc, rc, ls, rs);
    gLastEncL   = lc;
    gLastEncR   = rc;
    gLastPrintT = millis();
}

// ============================================================
//  STATUS LINE PRINTER
//  One line per call, aligned columns. Called from loop() and
//  runPhase(). Never called from interrupt context.
//
//  Column layout:
//    Phase(14) | L:dir pwm | R:dir pwm
//    || Lenc:count  dL:delta[sym] AB:state
//     | Renc:count  dR:delta[sym] AB:state
//     | cps: L:xxx R:xxx
// ============================================================
void printStatus() {
    long    lc, rc;
    uint8_t ls, rs;
    snapshotEncoders(lc, rc, ls, rs);

    unsigned long now = millis();
    unsigned long dt  = now - gLastPrintT;
    if (dt < 1) dt = 1;

    long  dL   = lc - gLastEncL;
    long  dR   = rc - gLastEncR;
    float lCps = (float)dL * 1000.0f / (float)dt;
    float rCps = (float)dR * 1000.0f / (float)dt;

    gLastEncL   = lc;
    gLastEncR   = rc;
    gLastPrintT = now;

    // Direction symbol derived from encoder delta
    char lSym = (dL > 0) ? '+' : (dL < 0) ? '-' : '0';
    char rSym = (dR > 0) ? '+' : (dR < 0) ? '-' : '0';

    // AB state as 2-char string: bit1=A, bit0=B
    char lAB[3] = { (char)('0' + ((ls >> 1) & 1)), (char)('0' + (ls & 1)), '\0' };
    char rAB[3] = { (char)('0' + ((rs >> 1) & 1)), (char)('0' + (rs & 1)), '\0' };

    const char* lDs = (gLeftDir  > 0) ? "FWD" : (gLeftDir  < 0) ? "REV" : "STP";
    const char* rDs = (gRightDir > 0) ? "FWD" : (gRightDir < 0) ? "REV" : "STP";
    int lPWM = gLeftActive  ? gCurrentPWM : 0;
    int rPWM = gRightActive ? gCurrentPWM : 0;

    char buf[180];
    snprintf(buf, sizeof(buf),
        "%-14s | L:%s %3d | R:%s %3d"
        " || Lenc:%+9ld dL:%+6ld[%c] AB:%s"
        "  | Renc:%+9ld dR:%+6ld[%c] AB:%s"
        "  | cps L:%+8.1f R:%+8.1f",
        gPhaseName,
        lDs, lPWM,
        rDs, rPWM,
        lc, dL, lSym, lAB,
        rc, dR, rSym, rAB,
        lCps, rCps);

    Serial.println(buf);
}

// ============================================================
//  AUTO-TEST PHASE RUNNER
// ============================================================
void runPhase(const char* name, int leftDir, int rightDir,
              unsigned long durationMs) {
    gPhaseName = name;

    Serial.print(F("\n>>> PHASE: "));
    Serial.print(name);
    Serial.print(F("    L="));
    Serial.print(leftDir  > 0 ? "FWD" : leftDir  < 0 ? "REV" : "STOP");
    Serial.print(F("    R="));
    Serial.println(rightDir > 0 ? "FWD" : rightDir < 0 ? "REV" : "STOP");

    setLeftMotor(leftDir,  gCurrentPWM);
    setRightMotor(rightDir, gCurrentPWM);
    resetDeltaBaseline();

    unsigned long start = millis();
    while (millis() - start < durationMs) {
        if (millis() - gLastPrintT >= PRINT_INTERVAL_MS) {
            printStatus();
        }
    }
}

// ============================================================
//  SERIAL STARTUP BANNER
// ============================================================
void printBanner() {
    Serial.println();
    Serial.println(F("==========================================================================="));
    Serial.println(F("  MOTOR & ENCODER WIRING TEST"));
    Serial.println(F("  Arduino GIGA R1 WiFi  +  Arduino Motor Shield Rev3"));
    Serial.println(F("==========================================================================="));
    Serial.println(F(""));
    Serial.println(F("  WIRING:"));
    Serial.println(F("    Left  Motor   Channel A :  DIR=D12   PWM=D3    BRAKE=D9"));
    Serial.println(F("    Right Motor   Channel B :  DIR=D13   PWM=D11   BRAKE=D8"));
    Serial.println(F("    Left  Encoder           :  A=D4   B=D5"));
    Serial.println(F("    Right Encoder           :  A=D6   B=D7"));
    Serial.println(F(""));
    Serial.println(F("  DIRECTION CONVENTION (viewed from OUTSIDE the robot, looking inward):"));
    Serial.println(F("    FORWARD:  Left wheel CCW,   Right wheel CW"));
    Serial.println(F("    REVERSE:  Left wheel CW,    Right wheel CCW"));
    Serial.println(F(""));
    Serial.println(F("  ENCODER RESOLUTION:"));
    Serial.println(F("    Motor shaft : 64 CPR  (4x quadrature = both edges, both channels)"));
    Serial.println(F("    Output shaft: 64 x 19 = 1216 counts/revolution"));
    Serial.println(F(""));
    Serial.println(F("  ENCODER POLLING:"));
    Serial.print  (F("    Poll interval: ")); Serial.print(POLL_INTERVAL_US); Serial.println(F(" µs"));
    Serial.print  (F("    Poll rate    : ")); Serial.print(1000000UL / POLL_INTERVAL_US); Serial.println(F(" Hz"));
    Serial.println(F("    Max reliable output speed: ~617 RPM  (target 530 RPM = 86% of max)"));
    Serial.println(F(""));
    Serial.println(F("  ACTIVE SETTINGS:"));

    Serial.print  (F("    LEFT_MOTOR_INVERT    = "));
    Serial.println(LEFT_MOTOR_INVERT    ? F("true   <- SW flips DIR for left motor")    : F("false"));
    Serial.print  (F("    RIGHT_MOTOR_INVERT   = "));
    Serial.println(RIGHT_MOTOR_INVERT   ? F("true   <- SW flips DIR for right motor")   : F("false"));
    Serial.print  (F("    LEFT_ENCODER_INVERT  = "));
    Serial.println(LEFT_ENCODER_INVERT  ? F("true   <- SW flips count sign, left enc")  : F("false"));
    Serial.print  (F("    RIGHT_ENCODER_INVERT = "));
    Serial.println(RIGHT_ENCODER_INVERT ? F("true   <- SW flips count sign, right enc") : F("false"));

    Serial.print  (F("    TEST_PWM             = ")); Serial.println(TEST_PWM);
    Serial.print  (F("    TEST_DURATION_MS     = ")); Serial.println(TEST_DURATION_MS);
    Serial.println(F(""));
    Serial.println(F("  MANUAL COMMANDS (active after auto-test, send via Serial Monitor):"));
    Serial.println(F("    s  = STOP all motors"));
    Serial.println(F("    f  = BOTH motors FORWARD"));
    Serial.println(F("    b  = BOTH motors REVERSE"));
    Serial.println(F("    l  = LEFT forward only      L  = LEFT reverse only"));
    Serial.println(F("    r  = RIGHT forward only     R  = RIGHT reverse only"));
    Serial.println(F("    +  = PWM +10                -  = PWM -10"));
    Serial.println(F("    z  = Zero encoder counts"));
    Serial.println(F(""));
    Serial.println(F("  OUTPUT COLUMNS:"));
    Serial.println(F("    Phase(14) | L:dir pwm | R:dir pwm"));
    Serial.println(F("    || Lenc:count  dL:delta[+/-/0] AB:ab"));
    Serial.println(F("     | Renc:count  dR:delta[+/-/0] AB:ab"));
    Serial.println(F("     | cps L:xxx R:xxx"));
    Serial.println(F(""));
    Serial.println(F("    [+] = encoder counting up   (positive direction)"));
    Serial.println(F("    [-] = encoder counting down (negative direction)"));
    Serial.println(F("    [0] = no encoder movement detected"));
    Serial.println(F("==========================================================================="));
    Serial.println();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);   // Allow Serial Monitor to connect

    // --- Motor output pins ---
    pinMode(PIN_A_DIR,   OUTPUT);
    pinMode(PIN_A_PWM,   OUTPUT);
    pinMode(PIN_A_BRAKE, OUTPUT);
    pinMode(PIN_B_DIR,   OUTPUT);
    pinMode(PIN_B_PWM,   OUTPUT);
    pinMode(PIN_B_BRAKE, OUTPUT);

    // --- Encoder input pins ---
    // Pololu Hall encoders have push-pull outputs, so INPUT is sufficient.
    // INPUT_PULLUP used defensively to avoid floating if a wire is loose.
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    // --- Safe initial motor state ---
    stopAll();

    // --- Seed encoder state machines from current pin levels ---
    // This prevents a phantom count on the first poll tick.
    gEncLState = ((digitalRead(PIN_ENC_L_A) ? 1u : 0u) << 1)
               |  (digitalRead(PIN_ENC_L_B) ? 1u : 0u);
    gEncRState = ((digitalRead(PIN_ENC_R_A) ? 1u : 0u) << 1)
               |  (digitalRead(PIN_ENC_R_B) ? 1u : 0u);
    gEncL = 0;
    gEncR = 0;

    // --- Start encoder polling Ticker ---
    // mbed 6 API: attach(callback, chrono::duration)
    // attach_us() is deprecated in mbed 6 and must not be used.
    encTicker.attach(encSample, std::chrono::microseconds(POLL_INTERVAL_US));

    printBanner();
    delay(500);

    // ============================================================
    //  AUTO-TEST SEQUENCE
    //  Tests each motor and encoder individually, then both together.
    // ============================================================
    runPhase("1-INIT-STOP",    0,  0,  500UL);
    runPhase("2-LEFT-FWD",    +1,  0,  TEST_DURATION_MS);
    runPhase("3-STOP",         0,  0,  500UL);
    runPhase("4-RIGHT-FWD",    0, +1,  TEST_DURATION_MS);
    runPhase("5-STOP",         0,  0,  500UL);
    runPhase("6-BOTH-FWD",    +1, +1,  TEST_DURATION_MS);
    runPhase("7-STOP",         0,  0,  500UL);
    runPhase("8-BOTH-REV",    -1, -1,  TEST_DURATION_MS);
    runPhase("9-FINAL-STOP",   0,  0,  500UL);

    Serial.println(F("\n>>> AUTO-TEST COMPLETE — entering manual mode."));
    Serial.println(F("    Type a command (s/f/b/l/r/L/R/+/-/z) then press Enter."));
    Serial.println();

    gPhaseName = "MANUAL";
    resetDeltaBaseline();
}

// ============================================================
//  LOOP — manual serial command mode
// ============================================================
void loop() {
    // --- Process incoming serial command ---
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        while (Serial.available()) Serial.read();   // flush rest of line

        bool recognized = true;

        switch (cmd) {
            case 's':
                stopAll();
                gPhaseName = "MAN-STOP";
                Serial.println(F("[CMD] STOP"));
                break;

            case 'f':
                setLeftMotor(+1, gCurrentPWM);
                setRightMotor(+1, gCurrentPWM);
                gPhaseName = "MAN-BOTH-FWD";
                Serial.println(F("[CMD] BOTH FORWARD"));
                break;

            case 'b':
                setLeftMotor(-1, gCurrentPWM);
                setRightMotor(-1, gCurrentPWM);
                gPhaseName = "MAN-BOTH-REV";
                Serial.println(F("[CMD] BOTH REVERSE"));
                break;

            case 'l':
                setLeftMotor(+1, gCurrentPWM);
                setRightMotor(0, 0);
                gPhaseName = "MAN-L-FWD";
                Serial.println(F("[CMD] LEFT FORWARD"));
                break;

            case 'r':
                setLeftMotor(0, 0);
                setRightMotor(+1, gCurrentPWM);
                gPhaseName = "MAN-R-FWD";
                Serial.println(F("[CMD] RIGHT FORWARD"));
                break;

            case 'L':
                setLeftMotor(-1, gCurrentPWM);
                setRightMotor(0, 0);
                gPhaseName = "MAN-L-REV";
                Serial.println(F("[CMD] LEFT REVERSE"));
                break;

            case 'R':
                setLeftMotor(0, 0);
                setRightMotor(-1, gCurrentPWM);
                gPhaseName = "MAN-R-REV";
                Serial.println(F("[CMD] RIGHT REVERSE"));
                break;

            case '+':
                gCurrentPWM = min(255, gCurrentPWM + 10);
                if (gLeftActive)  setLeftMotor(gLeftDir,   gCurrentPWM);
                if (gRightActive) setRightMotor(gRightDir, gCurrentPWM);
                Serial.print(F("[CMD] PWM = ")); Serial.println(gCurrentPWM);
                break;

            case '-':
                gCurrentPWM = max(0, gCurrentPWM - 10);
                if (gLeftActive)  setLeftMotor(gLeftDir,   gCurrentPWM);
                if (gRightActive) setRightMotor(gRightDir, gCurrentPWM);
                Serial.print(F("[CMD] PWM = ")); Serial.println(gCurrentPWM);
                break;

            case 'z':
                noInterrupts();
                gEncL = 0;
                gEncR = 0;
                interrupts();
                gLastEncL = 0;
                gLastEncR = 0;
                Serial.println(F("[CMD] ENCODER COUNTS ZEROED"));
                break;

            default:
                recognized = false;
                break;
        }

        if (recognized) {
            resetDeltaBaseline();
        }
    }

    // --- Periodic status print ---
    if (millis() - gLastPrintT >= PRINT_INTERVAL_MS) {
        printStatus();
    }
}

// ============================================================
//  TROUBLESHOOTING GUIDE
// ============================================================
//
//  SYMPTOM: A motor spins in the WRONG DIRECTION for "forward"
//  FIX    : Set the corresponding MOTOR_INVERT flag to true:
//             LEFT_MOTOR_INVERT  = true     (for left motor)
//             RIGHT_MOTOR_INVERT = true     (for right motor)
//           This inverts the DIR signal in software. No rewiring needed.
//
//  SYMPTOM: Encoder counts correctly but with WRONG SIGN
//           (e.g., motor commanded FWD but count goes negative)
//  FIX    : Set the corresponding ENCODER_INVERT flag to true:
//             LEFT_ENCODER_INVERT  = true
//             RIGHT_ENCODER_INVERT = true
//
//  SYMPTOM: Encoder A and B channels appear SWAPPED
//           (swapping A<->B is equivalent to inverting count direction)
//  FIX    : Try ENCODER_INVERT first. If counts are erratic/noisy rather
//           than smoothly inverted, physically swap the A and B wires, or
//           swap PIN_ENC_L_A <-> PIN_ENC_L_B in the pin definitions above.
//
//  SYMPTOM: Motor runs but encoder count stays at 0
//  CHECK  : 1. Encoder VCC connected to 5V (not 3.3V)?
//           2. Encoder GND shared with Arduino GND?
//           3. Signal wires on correct pins (D4/D5 left, D6/D7 right)?
//           4. Turn wheel by hand — do counts change?
//              Yes -> motor spins too fast at this PWM (unlikely at 150)
//              No  -> wiring fault on signal line
//
//  SYMPTOM: Encoder counts jump randomly / very large delta values
//  CHECK  : Electrical noise on encoder lines.
//           Add a 100 nF ceramic cap between each signal line and GND,
//           placed close to the Arduino header pins.
//
//  HOW TO READ THE AB COLUMN:
//    AB stays at "00" always          -> VCC or signal wire not connected
//    AB cycles 00->10->11->01->00     -> forward direction, encoder healthy
//    AB cycles 00->01->11->10->00     -> reverse direction, encoder healthy
//    AB only ever shows two values     -> one channel broken/disconnected
//
//  POLLING RATE AND SPEED HEADROOM:
//    At 25 kHz (40 µs), the minimum detectable transition gap is:
//      1 / (2 × 25000) = 20 µs  →  max transition rate = 25,000 /s
//    Max motor shaft speed: 25000 / 64 × 60 = 23,437 RPM (theoretical)
//    Max output shaft speed: 23437 / 19 = 1233 RPM (theoretical)
//    Real-world safety limit (Nyquist × 2 margin): ~617 RPM output
//
//    The Pololu 37D 12V 19:1 no-load max output speed is ~530 RPM.
//    At 40 µs this is covered with ~16% reserve — solid.
//
//    To change the rate, edit POLL_INTERVAL_US in USER SETTINGS.
//    See the table there for speed limits at other intervals.
//    API: encTicker.attach(encSample, std::chrono::microseconds(POLL_INTERVAL_US))
//
// ============================================================
