#include <M5Unified.h>
// NimBLE-Arduino — install via Arduino Library Manager
#include <NimBLEDevice.h>

// Button A (face)   = short press: send key for current mode+tilt
//                    long press (1 s): cycle key mode
// Button B (side)   = short press: cycle device  /  hold 2 s: enter pairing mode

static const uint8_t hidReportMap[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05,
    0x95, 0x05, 0x75, 0x01, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
    0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
    0xC0
};

#define MOD_LEFT_GUI          0x08
#define MOD_LEFT_SHIFT        0x02
#define KEY_ENTER             0x28
#define KEY_LBRACKET          0x2F  // [
#define KEY_RBRACKET          0x30  // ]
#define KEY_SPACE             0x2C
#define KEY_RIGHT             0x4F
#define KEY_LEFT              0x50
#define KEY_DOWN              0x51
#define KEY_UP                0x52
#define KEY_F10               0x43  // Mute (macOS)
#define KEY_F11               0x44  // Volume down
#define KEY_F12               0x45  // Volume up / camera shutter
#define NO_CONN               0xFFFF

// Hold Button A for this long to cycle key modes.
#define MODE_HOLD_MS  1000UL

// ── Key modes ─────────────────────────────────────────────────────────────
// Each mode maps the three possible inputs (center, tilt-left, tilt-right)
// to a modifier+keycode pair plus a short display label.

struct KeyMode {
    const char* name;
    uint8_t     centerMod, centerKey;
    uint8_t     leftMod,   leftKey;
    uint8_t     rightMod,  rightKey;
    const char* centerLabel;
    const char* leftLabel;
    const char* rightLabel;
};

static const KeyMode keyModes[] = {
    {
        "Claude",
        MOD_LEFT_GUI,                        KEY_ENTER,
        MOD_LEFT_GUI | MOD_LEFT_SHIFT,       KEY_RBRACKET,
        MOD_LEFT_GUI | MOD_LEFT_SHIFT,       KEY_LBRACKET,
        "Cmd+Enter", "< Cmd+Shift+]", "Cmd+Shift+[ >"
    },
    {
        "Slide",
        0, KEY_ENTER,
        0, KEY_RIGHT,
        0, KEY_LEFT,
        "Enter", "< Next", "Prev >"
    },
    {
        "Scroll",
        0, KEY_SPACE,
        0, KEY_DOWN,
        0, KEY_UP,
        "Space", "v Down", "Up ^"
    },
    {
        "Volume",
        0, KEY_F10,
        0, KEY_F12,
        0, KEY_F11,
        "Mute", "< Vol+", "Vol- >"
    },
};
static const int NUM_MODES = (int)(sizeof(keyModes) / sizeof(keyModes[0]));

// Roll threshold in G. Tune this to adjust tilt sensitivity.
// 0.30 G ≈ 17°, 0.50 G ≈ 30°. Lower = more sensitive.
#define TILT_THRESHOLD        0.30f

// Directed advertising lasts this long before falling back to undirected.
// Long enough to let the user wake the target Mac.
#define DIRECTED_ADV_TIMEOUT_MS  30000UL

// Hold Button B for this long to enter pairing mode.
#define PAIRING_HOLD_MS  2000UL

// After rejecting a wrong-device connection, wait briefly before re-advertising
// to let the rejected device stop trying immediately.
#define REJECTION_BACKOFF_MS  800UL

// ── Shared state ──────────────────────────────────────────────────────────

static NimBLECharacteristic* inputReport     = nullptr;
static bool     bleConnected                 = false;
static uint16_t activeConnHandle             = NO_CONN;
static int      targetBondIdx                = 0;
static uint32_t directedAdvStartMs           = 0;
static bool     pairingMode                  = false;

// Set from BLE callbacks; consumed in loop() for display updates.
// All M5.Display calls happen in the main task only.
static volatile bool displayDirty           = false;

// Signals onDisconnect that the disconnect was a deliberate rejection.
static volatile bool lastWasRejection       = false;

// Advertising restart request from BLE callbacks.
// Non-zero = "restart advertising at this millis() value (or later)".
// loop() is the only place that calls startAdvertising(), avoiding concurrent
// calls from the BLE task and the main task that would crash NimBLE.
static volatile uint32_t advRestartAtMs     = 0;

// Tilt state from accelerometer — determines what Button A sends
enum TiltState : uint8_t { TILT_CENTER, TILT_LEFT, TILT_RIGHT };
static TiltState currentTilt = TILT_CENTER;

// Button B timing for long-press detection
static uint32_t btnBDownMs                   = 0;
static bool     btnBActionDone               = false;

// Button A timing (short press = send key, long press = cycle mode)
static uint32_t btnADownMs                   = 0;
static bool     btnAActionDone               = false;

// Current key mode index
static int      currentMode                  = 0;

// ── Forward declarations ───────────────────────────────────────────────────

void startAdvertising();
void updateStatusDisplay();
void drawButtonHints();

// ── BLE callbacks ──────────────────────────────────────────────────────────

class BLECallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& info) override {
        NimBLEAddress addr = info.getAddress();
        int n = NimBLEDevice::getNumBonds();
        int connIdx = -1;

        for (int i = 0; i < n; i++) {
            if (NimBLEDevice::getBondedAddress(i) == addr) {
                connIdx = i;
                break;
            }
        }

        // Normal mode: reject any device that is not the current target
        if (!pairingMode && n > 1 && connIdx >= 0 && connIdx != targetBondIdx) {
            Serial.printf("[BLE] reject Dev%d (want Dev%d)\n", connIdx + 1, targetBondIdx + 1);
            lastWasRejection = true;
            pServer->disconnect(info.getConnHandle());
            return;
        }

        // Pairing mode: only accept genuinely new (unknown) devices.
        // Without this, the nearest already-bonded Mac reconnects immediately,
        // blocking the new device from ever pairing.
        if (pairingMode && connIdx >= 0) {
            Serial.printf("[PAIR] reject known Dev%d — waiting for new device\n", connIdx + 1);
            lastWasRejection = true;
            pServer->disconnect(info.getConnHandle());
            return;
        }

        pairingMode    = false;
        bleConnected   = true;
        activeConnHandle = info.getConnHandle();
        directedAdvStartMs = 0;
        if (connIdx >= 0) targetBondIdx = connIdx;

        Serial.printf("[BLE] connected handle=%d idx=%d\n",
                      activeConnHandle, targetBondIdx);
        NimBLEDevice::startSecurity(activeConnHandle);
        displayDirty = true;
    }

    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
        bool rejected = lastWasRejection;
        lastWasRejection = false;
        bleConnected     = false;
        activeConnHandle = NO_CONN;

        Serial.printf("[BLE] disconnected reason=0x%02x  next=Dev%d%s\n",
                      reason, targetBondIdx + 1, rejected ? " (rejected)" : "");

        // Schedule advertising restart in the main loop.
        // We must NOT call startAdvertising() here: doing so from the BLE task
        // while the main task may also call it (e.g. Button B press during the
        // rejection backoff) causes concurrent adv->stop()/start() and crashes NimBLE.
        uint32_t backoff = rejected ? REJECTION_BACKOFF_MS : 0;
        advRestartAtMs = millis() + backoff;
        displayDirty   = true;
    }
};

// ── Advertising ────────────────────────────────────────────────────────────

void startAdvertising() {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->stop();

    int n = NimBLEDevice::getNumBonds();

    if (n == 0 || pairingMode) {
        adv->start();
        directedAdvStartMs = 0;
        Serial.println(pairingMode
                       ? "[ADV] pairing mode — undirected"
                       : "[ADV] no bonds — undirected");
    } else {
        if (targetBondIdx >= n) targetBondIdx = 0;
        NimBLEAddress target = NimBLEDevice::getBondedAddress(targetBondIdx);
        Serial.printf("[ADV] directed -> Dev%d/%d  addr=%s\n",
                      targetBondIdx + 1, n, target.toString().c_str());
        bool ok = adv->start(DIRECTED_ADV_TIMEOUT_MS, &target);
        if (ok) {
            directedAdvStartMs = millis();
        } else {
            Serial.println("[ADV] directed start failed — undirected fallback");
            adv->start();
            directedAdvStartMs = 0;
        }
    }
}

// ── Device switching / pairing ────────────────────────────────────────────

void switchToNextDevice() {
    int n = NimBLEDevice::getNumBonds();
    if (n == 0) return;

    targetBondIdx = (targetBondIdx + 1) % n;
    Serial.printf("[SW] -> Dev%d/%d\n", targetBondIdx + 1, n);

    if (bleConnected && activeConnHandle != NO_CONN) {
        NimBLEDevice::getServer()->disconnect(activeConnHandle);
        // onDisconnect → startAdvertising() toward new target
    } else {
        startAdvertising();
        displayDirty = true;
    }
}

void enterPairingMode() {
    Serial.println("[PAIR] entering pairing mode — hold B again to cancel");
    pairingMode = true;
    if (bleConnected && activeConnHandle != NO_CONN) {
        NimBLEDevice::getServer()->disconnect(activeConnHandle);
        // onDisconnect → startAdvertising() in pairing mode
    } else {
        startAdvertising();
        displayDirty = true;
    }
}

void cancelPairingMode() {
    Serial.println("[PAIR] cancelled");
    pairingMode = false;
    startAdvertising();
    displayDirty = true;
}

// ── BLE setup ─────────────────────────────────────────────────────────────

void setupBLE() {
    NimBLEDevice::init("Claude Keyboard");
    NimBLEDevice::setSecurityAuth(true, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new BLECallbacks());

    // Device Information Service (0x180A)
    NimBLEService* dis = server->createService("180A");
    dis->createCharacteristic("2A29", NIMBLE_PROPERTY::READ)->setValue("M5Stack");
    dis->createCharacteristic("2A24", NIMBLE_PROPERTY::READ)->setValue("M5StickS3");
    uint8_t pnpId[] = {0x01, 0xE5, 0x02, 0x00, 0x00, 0x01, 0x00};
    dis->createCharacteristic("2A50", NIMBLE_PROPERTY::READ)->setValue(pnpId, sizeof(pnpId));
    dis->start();

    // HID Service (0x1812)
    NimBLEService* hid = server->createService("1812");
    uint8_t mode = 1;
    hid->createCharacteristic("2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR)
       ->setValue(&mode, 1);
    hid->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC)
       ->setValue(hidReportMap, sizeof(hidReportMap));

    inputReport = hid->createCharacteristic(
        "2A4D",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC);
    uint8_t inRef[] = {0x01, 0x01};
    inputReport->createDescriptor("2908", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC, 2)
               ->setValue(inRef, 2);

    NimBLECharacteristic* outputRpt = hid->createCharacteristic(
        "2A4D",
        NIMBLE_PROPERTY::READ  | NIMBLE_PROPERTY::WRITE    | NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC);
    uint8_t outRef[] = {0x01, 0x02};
    outputRpt->createDescriptor("2908", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC, 2)
             ->setValue(outRef, 2);

    uint8_t info[] = {0x11, 0x01, 0x00, 0x03};
    hid->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ)->setValue(info, sizeof(info));
    hid->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);
    hid->start();

    // Battery Service (0x180F)
    NimBLEService* batt = server->createService("180F");
    uint8_t lvl = 100;
    batt->createCharacteristic(
        "2A19",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC)
        ->setValue(&lvl, 1);
    batt->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName("Claude Keyboard");
    adv->addServiceUUID("1812");
    adv->setAppearance(0x03C1);

    startAdvertising();
}

// ── Key sending ────────────────────────────────────────────────────────────

void sendKey(uint8_t modifier, uint8_t keycode) {
    if (!bleConnected || inputReport == nullptr) return;
    uint8_t down[8] = {modifier, 0x00, keycode};
    inputReport->setValue(down, sizeof(down));
    inputReport->notify();
    delay(10);
    uint8_t up[8] = {};
    inputReport->setValue(up, sizeof(up));
    inputReport->notify();
}

// ── Display helpers (main task only) ─────────────────────────────────────

// Battery indicator — top-right corner, refreshed periodically.
void drawBattery() {
    int w = M5.Display.width();
    int batt = M5.Power.getBatteryLevel();
    bool chg  = M5.Power.isCharging();

    char buf[8];
    if (batt < 0) {
        return;  // no battery info available
    }
    snprintf(buf, sizeof(buf), chg ? "+%d%%" : "%d%%", batt);

    int bw = strlen(buf) * 6 + 2;
    M5.Display.fillRect(w - bw, 0, bw, 12, BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(batt > 50 ? GREEN : batt > 20 ? YELLOW : RED, BLACK);
    M5.Display.setCursor(w - bw + 1, 3);
    M5.Display.print(buf);
    M5.Display.setTextColor(WHITE, BLACK);
}

// Draw the dynamic A-button hint at the bottom: mode name (top) + tilt label (bottom).
// Called from drawButtonHints() and also directly from loop() when tilt or mode changes.
void drawTiltIndicator() {
    int w = M5.Display.width();
    int h = M5.Display.height();
    int usableW = w - 18;  // exclude right B-hint strip

    const KeyMode& m = keyModes[currentMode];
    const char* tiltLabel;
    uint32_t    tiltColor;
    switch (currentTilt) {
        case TILT_LEFT:  tiltLabel = m.leftLabel;   tiltColor = YELLOW;       break;
        case TILT_RIGHT: tiltLabel = m.rightLabel;  tiltColor = YELLOW;       break;
        default:         tiltLabel = m.centerLabel; tiltColor = TFT_DARKGREY; break;
    }

    M5.Display.fillRect(0, h - 20, usableW, 20, BLACK);
    M5.Display.setTextSize(1);

    // Mode name row
    char modeBuf[12];
    snprintf(modeBuf, sizeof(modeBuf), "[%s]", m.name);
    int modeW = strlen(modeBuf) * 6;
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.setCursor((usableW - modeW) / 2, h - 19);
    M5.Display.print(modeBuf);

    // Tilt action row
    int tiltW = strlen(tiltLabel) * 6;
    M5.Display.setTextColor(tiltColor, BLACK);
    M5.Display.setCursor((usableW - tiltW) / 2, h - 9);
    M5.Display.print(tiltLabel);

    M5.Display.setTextColor(WHITE, BLACK);
}

// Draw Button B hint as two-line vertical text on the right edge.
// The two lines are packed into a single sprite (sprH=18) so one pushRotateZoom
// handles both. With 270° rotation each line reads top→bottom on screen.
static void drawBHintVertical() {
    int w = M5.Display.width();
    int h = M5.Display.height();

    const char* line1 = "Click:Next Device";   // 17 chars × 6px = 102px
    const char* line2 = "Hold:Add Device";     // 15 chars × 6px =  90px
    int sprW = strlen(line1) * 6;              // wider of the two
    int sprH = 18;                             // 8 (line1) + 2 (gap) + 8 (line2)

    M5Canvas spr(&M5.Display);
    if (!spr.createSprite(sprW, sprH)) return;

    spr.fillScreen(TFT_BLACK);
    spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
    spr.setTextSize(1);
    spr.setCursor(0, 0);
    spr.print(line1);
    spr.setCursor(0, 10);
    spr.print(line2);

    // Position in the lower portion of the screen, clamped so nothing clips.
    int centerY = h / 2 + 15;
    int maxY    = h - sprW / 2 - 2;
    if (centerY > maxY) centerY = maxY;
    spr.pushRotateZoom(w - 9, centerY, 270, 1.0, 1.0);
    spr.deleteSprite();
}

void drawButtonHints() {
    // A hint: dynamic tilt indicator (centered at bottom)
    drawTiltIndicator();
    // B hint: vertical on right edge (two lines, 18px wide strip)
    drawBHintVertical();
}

// Layout (landscape 240×135):
//   y=  5  "Claude"    (size 2, static)
//   y= 21  "Keyboard"  (size 2, static)
//   y= 55  "v0.6.0"    (size 1, static)
//   y= 68  status      (dynamic)
//   y= 92  action      (dynamic)
//   y=h-19 mode name   (A hint line 1, cyan)
//   y=h-9  tilt label  (A hint line 2, yellow/grey)
//   right  B hint      (vertical strip, 18 px)
#define DYNAMIC_AREA_TOP  65
#define STATUS_Y          68
#define ACTION_Y          92

void updateStatusDisplay() {
    int w = M5.Display.width();
    int h = M5.Display.height();

    // Clear everything from below the static title to the bottom
    M5.Display.fillRect(0, DYNAMIC_AREA_TOP, w, h - DYNAMIC_AREA_TOP, BLACK);

    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, STATUS_Y);

    int n   = NimBLEDevice::getNumBonds();
    char buf[40];

    if (pairingMode) {
        snprintf(buf, sizeof(buf), "Pair new device\n(Side Click:Cancel)");
        M5.Display.setTextColor(MAGENTA, BLACK);
    } else if (bleConnected) {
        snprintf(buf, sizeof(buf), "[Dev%d/%d] Connected!", targetBondIdx + 1, n);
        M5.Display.setTextColor(GREEN, BLACK);
    } else if (directedAdvStartMs > 0) {
        snprintf(buf, sizeof(buf), "-> Dev%d/%d ...", targetBondIdx + 1, n);
        M5.Display.setTextColor(CYAN, BLACK);
    } else if (n == 0) {
        snprintf(buf, sizeof(buf), "Pairing mode...");
        M5.Display.setTextColor(YELLOW, BLACK);
    } else {
        snprintf(buf, sizeof(buf), "Waiting (%d bond%s)...", n, n == 1 ? "" : "s");
        M5.Display.setTextColor(YELLOW, BLACK);
    }
    M5.Display.println(buf);
    M5.Display.setTextColor(WHITE, BLACK);

    drawBattery();
    drawButtonHints();
}

void drawAction(const char* label) {
    int w = M5.Display.width();
    // Preserve the right 18px strip (two-line B-hint vertical text)
    M5.Display.fillRect(0, ACTION_Y, w - 18, 10, BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, ACTION_Y);
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(label);
}

// ── Arduino entry points ───────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 5);
    M5.Display.println("Claude");
    M5.Display.println("Keyboard");
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 55);
    M5.Display.println("v0.6.0");

    setupBLE();
    updateStatusDisplay();
    drawBattery();
}

void loop() {
    M5.update();

    // Display updates must happen in the main task
    if (displayDirty) {
        displayDirty = false;
        updateStatusDisplay();
    }

    // Advertising restart requested by onDisconnect (see advRestartAtMs).
    // Centralising startAdvertising() in the main loop prevents concurrent
    // calls from the BLE task and the main task that crashed NimBLE before.
    if (advRestartAtMs > 0 && millis() >= advRestartAtMs) {
        advRestartAtMs = 0;
        if (!bleConnected) {
            startAdvertising();
            displayDirty = true;
        }
    }

    // Refresh battery indicator every 30 s
    static uint32_t lastBattMs = 0;
    if (millis() - lastBattMs >= 30000) {
        lastBattMs = millis();
        drawBattery();
    }

    // Tilt detection — update A-hint and button behaviour every 200 ms.
    // Uses the X-axis acceleration (roll).  If left/right are swapped on your
    // device, negate the comparison (ax > threshold → TILT_LEFT, etc.).
    // Serial log: hold the device and watch "[TILT]" lines to find the right axis.
    static uint32_t lastImuMs = 0;
    if (millis() - lastImuMs >= 200) {
        lastImuMs = millis();
        if (M5.Imu.update()) {
            auto d = M5.Imu.getImuData();
            // Serial.printf("[TILT] x=%.2f y=%.2f z=%.2f\n",
            //               d.accel.x, d.accel.y, d.accel.z);
            TiltState newTilt;
            if      (d.accel.y >  TILT_THRESHOLD) newTilt = TILT_LEFT;
            else if (d.accel.y < -TILT_THRESHOLD) newTilt = TILT_RIGHT;
            else                                    newTilt = TILT_CENTER;
            if (newTilt != currentTilt) {
                currentTilt = newTilt;
                drawTiltIndicator();
            }
        }
    }

    // Directed advertising timeout → fall back to undirected
    if (!bleConnected && directedAdvStartMs > 0 &&
        (millis() - directedAdvStartMs) > DIRECTED_ADV_TIMEOUT_MS) {
        Serial.println("[ADV] directed timeout — undirected fallback");
        directedAdvStartMs = 0;
        NimBLEDevice::getAdvertising()->stop();
        NimBLEDevice::getAdvertising()->start();
        displayDirty = true;
    }

    // ── Button A: short press = send key, long press (1 s) = cycle mode ─────
    if (M5.BtnA.wasPressed()) {
        btnADownMs    = millis();
        btnAActionDone = false;
    }

    if (M5.BtnA.isPressed() && !btnAActionDone && btnADownMs > 0) {
        if (millis() - btnADownMs >= MODE_HOLD_MS) {
            btnAActionDone = true;
            currentMode = (currentMode + 1) % NUM_MODES;
            drawTiltIndicator();
            char buf[24];
            snprintf(buf, sizeof(buf), "Mode: %s", keyModes[currentMode].name);
            drawAction(buf);
            delay(800);
            drawAction("");
        }
    }

    if (M5.BtnA.wasReleased()) {
        if (!btnAActionDone && btnADownMs > 0) {
            const KeyMode& m = keyModes[currentMode];
            if (bleConnected) {
                switch (currentTilt) {
                    case TILT_LEFT:
                        sendKey(m.leftMod, m.leftKey);
                        drawAction(m.leftLabel);
                        break;
                    case TILT_RIGHT:
                        sendKey(m.rightMod, m.rightKey);
                        drawAction(m.rightLabel);
                        break;
                    default:
                        sendKey(m.centerMod, m.centerKey);
                        drawAction(m.centerLabel);
                        break;
                }
                delay(500);
                drawAction("");
            } else if (!pairingMode) {
                drawAction("Not connected");
                delay(500);
                drawAction("");
            }
        }
        btnADownMs    = 0;
        btnAActionDone = false;
    }

    // ── Button B: short press = switch device, hold = pairing mode ────────
    if (M5.BtnB.wasPressed()) {
        btnBDownMs    = millis();
        btnBActionDone = false;
    }

    // While held (normal mode only): fire enterPairingMode at 2 s
    if (M5.BtnB.isPressed() && !btnBActionDone && btnBDownMs > 0 && !pairingMode) {
        if (millis() - btnBDownMs >= PAIRING_HOLD_MS) {
            btnBActionDone = true;
            enterPairingMode();
        }
    }

    // On release: short press action depends on current mode
    if (M5.BtnB.wasReleased()) {
        if (!btnBActionDone && btnBDownMs > 0) {
            if (pairingMode) {
                // Click cancels pairing mode
                cancelPairingMode();
            } else if (NimBLEDevice::getNumBonds() > 0) {
                switchToNextDevice();
            }
        }
        btnBDownMs    = 0;
        btnBActionDone = false;
    }

    delay(10);
}
