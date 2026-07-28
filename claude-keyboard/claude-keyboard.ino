#include <M5Unified.h>
// NimBLE-Arduino — install via Arduino Library Manager
#include <NimBLEDevice.h>

// Button A = Command+Enter
// Button B = switch to next bonded device via directed advertising

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
#define DIRECTED_ADV_TIMEOUT_MS 30000  // 30 s — long enough for user to wake Mac

static NimBLECharacteristic* inputReport   = nullptr;
static bool     bleConnected               = false;
static uint16_t activeConnHandle           = NO_CONN;
static int      targetBondIdx              = 0;
static uint32_t directedAdvStartMs         = 0;

// Flag to request a display refresh from the main loop.
// BLE callbacks run in a separate task; touching M5.Display there causes
// display corruption. Set this flag instead and let loop() do the update.
static volatile bool displayDirty = false;

void startAdvertising();

// ── BLE callbacks ──────────────────────────────────────────────────────────

class BLECallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
        bleConnected     = true;
        activeConnHandle = info.getConnHandle();
        directedAdvStartMs = 0;

        // Sync targetBondIdx to whichever device actually connected
        NimBLEAddress addr = info.getAddress();
        int n = NimBLEDevice::getNumBonds();
        for (int i = 0; i < n; i++) {
            if (NimBLEDevice::getBondedAddress(i) == addr) {
                targetBondIdx = i;
                break;
            }
        }
        Serial.printf("[BLE] connected  handle=%d  targetIdx=%d\n",
                      activeConnHandle, targetBondIdx);
        NimBLEDevice::startSecurity(activeConnHandle);
        displayDirty = true;
    }

    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
        bleConnected     = false;
        activeConnHandle = NO_CONN;
        Serial.printf("[BLE] disconnected  reason=0x%02x  next=Dev%d\n",
                      reason, targetBondIdx + 1);
        startAdvertising();
        displayDirty = true;
    }
};

// ── Advertising ────────────────────────────────────────────────────────────

void startAdvertising() {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->stop();

    int n = NimBLEDevice::getNumBonds();
    if (n == 0) {
        adv->start();
        directedAdvStartMs = 0;
        Serial.println("[ADV] undirected (no bonds)");
    } else {
        if (targetBondIdx >= n) targetBondIdx = 0;
        NimBLEAddress target = NimBLEDevice::getBondedAddress(targetBondIdx);
        Serial.printf("[ADV] directed -> Dev%d/%d  addr=%s\n",
                      targetBondIdx + 1, n, target.toString().c_str());
        bool ok = adv->start(DIRECTED_ADV_TIMEOUT_MS, &target);
        if (ok) {
            directedAdvStartMs = millis();
        } else {
            // Directed advertising not supported or failed — fall back
            Serial.println("[ADV] directed failed, falling back to undirected");
            adv->start();
            directedAdvStartMs = 0;
        }
    }
}

void switchToNextDevice() {
    int n = NimBLEDevice::getNumBonds();
    if (n == 0) return;

    targetBondIdx = (targetBondIdx + 1) % n;
    Serial.printf("[SW] switching -> Dev%d/%d\n", targetBondIdx + 1, n);

    if (bleConnected && activeConnHandle != NO_CONN) {
        NimBLEDevice::getServer()->disconnect(activeConnHandle);
        // startAdvertising() will be called from onDisconnect
    } else {
        startAdvertising();
        displayDirty = true;
    }
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
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR |
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

// ── Display (called only from main loop) ──────────────────────────────────

void updateStatusDisplay() {
    M5.Display.fillRect(0, 70, M5.Display.width(), 20, BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 70);

    int n = NimBLEDevice::getNumBonds();
    char buf[32];

    if (bleConnected) {
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
}

void drawAction(const char* label) {
    M5.Display.fillRect(0, 95, M5.Display.width(), 15, BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 95);
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
    M5.Display.setCursor(10, 20);
    M5.Display.println("BT Keyboard");

    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 55);
    M5.Display.println("v0.4.0  BtnB=switch");

    setupBLE();
    updateStatusDisplay();
}

void loop() {
    M5.update();

    // Update display from main task only (BLE callbacks set displayDirty)
    if (displayDirty) {
        displayDirty = false;
        updateStatusDisplay();
    }

    // Directed advertising timeout → fall back to undirected
    if (!bleConnected && directedAdvStartMs > 0 &&
        (millis() - directedAdvStartMs) > DIRECTED_ADV_TIMEOUT_MS) {
        Serial.println("[ADV] directed timeout, falling back to undirected");
        directedAdvStartMs = 0;
        NimBLEDevice::getAdvertising()->stop();
        NimBLEDevice::getAdvertising()->start();
        displayDirty = true;
    }

    // Button A: send Command+Enter
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

    // Button B: cycle to next bonded device
    if (M5.BtnB.wasPressed()) {
        if (NimBLEDevice::getNumBonds() > 0) {
            switchToNextDevice();
        }
    }

    delay(10);
}
