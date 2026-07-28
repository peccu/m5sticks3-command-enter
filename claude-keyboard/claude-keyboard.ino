#include <M5Unified.h>
// NimBLE-Arduino — install via Arduino Library Manager
#include <NimBLEDevice.h>

// Button A (face)   = Command+Enter
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
#define KEY_ENTER             0x28
#define NO_CONN               0xFFFF

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

// Signals onDisconnect that the disconnect was a deliberate rejection
// (not a real disconnect), so advertising should restart after a backoff.
static volatile bool lastWasRejection       = false;

// Button B timing for long-press detection
static uint32_t btnBDownMs                   = 0;
static bool     btnBActionDone               = false;

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

        // In normal (non-pairing) mode with multiple bonds, reject the wrong device.
        // directed advertising in NimBLE-Arduino does not always enforce the filter
        // at the BLE layer, so we guard here instead.
        if (!pairingMode && n > 1 && connIdx >= 0 && connIdx != targetBondIdx) {
            Serial.printf("[BLE] reject Dev%d (want Dev%d) — disconnecting\n",
                          connIdx + 1, targetBondIdx + 1);
            lastWasRejection = true;
            pServer->disconnect(info.getConnHandle());
            return;  // onDisconnect will restart advertising after backoff
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

        if (rejected) {
            // Give the rejected device a moment to stop hammering us
            vTaskDelay(pdMS_TO_TICKS(REJECTION_BACKOFF_MS));
        }
        startAdvertising();
        displayDirty = true;
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

void drawButtonHints() {
    int h = M5.Display.height();
    int w = M5.Display.width();
    // Two-line legend at the bottom
    M5.Display.fillRect(0, h - 18, w, 18, BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_DARKGREY, BLACK);
    M5.Display.setCursor(2, h - 17);
    M5.Display.print("A: Cmd+Enter");
    M5.Display.setCursor(2, h - 8);
    M5.Display.print("B: Next  Hold: Add Dev");
    M5.Display.setTextColor(WHITE, BLACK);
}

void updateStatusDisplay() {
    int w = M5.Display.width();
    int h = M5.Display.height();

    // Clear dynamic area (below title, above hints)
    M5.Display.fillRect(0, 48, w, h - 48 - 19, BLACK);

    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 50);

    int n   = NimBLEDevice::getNumBonds();
    char buf[40];

    if (pairingMode) {
        snprintf(buf, sizeof(buf), "PAIRING... B:cancel");
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

    drawButtonHints();
}

void drawAction(const char* label) {
    int w = M5.Display.width();
    M5.Display.fillRect(0, 65, w, 10, BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 65);
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
    M5.Display.println("BT Keyboard");
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 35);
    M5.Display.println("v0.5.0");

    setupBLE();
    updateStatusDisplay();
}

void loop() {
    M5.update();

    // Display updates must happen in the main task
    if (displayDirty) {
        displayDirty = false;
        updateStatusDisplay();
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

    // ── Button A: send Command+Enter ──────────────────────────────────────
    if (M5.BtnA.wasPressed()) {
        if (bleConnected) {
            sendKey(MOD_LEFT_GUI, KEY_ENTER);
            drawAction("Cmd+Enter sent");
            delay(500);
            drawAction("");
        } else {
            drawAction("Not connected");
            delay(500);
            drawAction("");
        }
    }

    // ── Button B: short press = switch device, hold = pairing mode ────────
    if (M5.BtnB.wasPressed()) {
        btnBDownMs    = millis();
        btnBActionDone = false;
    }

    // While held: fire pairing action at 2 s
    if (M5.BtnB.isPressed() && !btnBActionDone && btnBDownMs > 0) {
        if (millis() - btnBDownMs >= PAIRING_HOLD_MS) {
            btnBActionDone = true;
            if (pairingMode) {
                cancelPairingMode();
            } else {
                enterPairingMode();
            }
        }
    }

    // On release: if no action was taken yet → short press → switch device
    if (M5.BtnB.wasReleased()) {
        if (!btnBActionDone && btnBDownMs > 0) {
            if (NimBLEDevice::getNumBonds() > 0) {
                switchToNextDevice();
            }
        }
        btnBDownMs    = 0;
        btnBActionDone = false;
    }

    delay(10);
}
